#include "SmartServoController.hpp"
#include "Bsp.hpp"
#include "ControlLink.hpp"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "utils/Debug.hpp"
#include "utils/TickTimer.hpp"

enum SmartServoState { IDLE, TX, RX };

static SmartServoState gState = IDLE;
static std::array<uint8_t, 16> gBuff;
static size_t gBuffSize = 0;
static size_t gBuffIndex = 0;
static bool gExpectResponse = false;

static CoprocStat_SmartServoStat gResponse;
static bool gResponseReady = false;
static TickTimer gResponseTimeout;

static TickTimer gStateTimeout;

void smartServoInit() {
    LL_USART_InitTypeDef init;
    LL_USART_StructInit(&init);
    init.BaudRate = 115200;
    init.DataWidth = LL_USART_DATAWIDTH_8B;
    init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    init.Parity = LL_USART_PARITY_NONE;
    init.StopBits = LL_USART_STOPBITS_1;
    init.TransferDirection = LL_USART_DIRECTION_TX;

    if (LL_USART_Init(servoUart, &init) != SUCCESS)
        abort();

    LL_USART_ConfigHalfDuplexMode(servoUart);
    LL_USART_Enable(servoUart);

    HAL_NVIC_SetPriority(servoUartIRQn, servoUartIrqPrio, 0);
    HAL_NVIC_EnableIRQ(servoUartIRQn);

    LL_USART_DisableIT_ERROR(servoUart);
    LL_USART_DisableIT_IDLE(servoUart);

    pinInit(
        servoUartTxRxPin, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
}

static void smartServoSetResponseReady() {
    if (gResponseReady) {
        DEBUGLN(
            "Invalid smartServoSetResponseReady call, it was already ready");
    }
    gResponseReady = true;
}

static void smartServoStopRx() {
    LL_USART_DisableIT_TC(servoUart);
    LL_USART_DisableIT_RXNE(servoUart);
    smartServoSetResponseReady();
    gState = IDLE;
    gResponseTimeout.stop();
}

void smartServoPoll() {
    if (gResponseReady) {
        const CoprocStat stat = {
            .which_payload = CoprocStat_smartServoStat_tag,
            .payload = {
                .smartServoStat = gResponse,
            },
        };
        gResponseReady = false;
        controlLinkTx(stat);
    }

    if (gResponseTimeout.poll()) {
        smartServoStopRx();
    }

    if (gStateTimeout.poll() && gState != IDLE) {
        LL_USART_DisableIT_TXE(servoUart);
        smartServoStopRx();
    }
}

extern "C" void SERVOUART_HANDLER(void) {
    // TX empty
    if (gState == TX && LL_USART_IsActiveFlag_TXE(servoUart)) {
        if (gBuffIndex < gBuffSize) {
            LL_USART_TransmitData8(servoUart, gBuff[gBuffIndex++]);
        } else {
            LL_USART_DisableIT_TXE(servoUart);

            if (!gExpectResponse) {
                smartServoSetResponseReady();
                gState = IDLE;
            } else {
                gState = RX;
                gBuffIndex = 0;

                LL_USART_EnableIT_TC(servoUart);

                gResponseTimeout.restart(30);
            }
        }
    }

    if (gState == RX) {
        // Transmission complete
        if (LL_USART_IsActiveFlag_TC(servoUart)) {
            LL_USART_DisableIT_TC(servoUart);
            LL_USART_ClearFlag_TC(servoUart);

            LL_USART_SetTransferDirection(servoUart, LL_USART_DIRECTION_RX);

            LL_USART_EnableIT_RXNE(servoUart);
        }

        // RX not empty
        if (LL_USART_IsActiveFlag_RXNE(servoUart)) {
            const uint8_t ch = LL_USART_ReceiveData8(servoUart);
            switch (gBuffIndex) {
            case 0:
            case 1:
                if (ch == 0x55) {
                    ++gBuffIndex;
                } else {
                    gBuffIndex = 0;
                }
                break;
            case 2:
                gResponse.id = ch;
                ++gBuffIndex;
                break;
            case 3:
                gResponse.data.size = ch - 1;
                ++gBuffIndex;
                break;
            default:
                gResponse.data.bytes[gBuffIndex - 4] = ch;
                ++gBuffIndex;
                if (gBuffIndex - 4 >= gResponse.data.size) {
                    smartServoStopRx();
                }
                break;
            }
        }
    }
}

void smartServoSendRequest(const CoprocReq_SmartServoReq& req) {
    if (gState != IDLE) {
        DEBUGLN("Invalid CoprocReq_SmartServoReq, state is %d", gState);
        return;
    }

    gStateTimeout.restart(60);

    gState = TX;

    gBuff[0] = 0x55;
    gBuff[1] = 0x55;
    gBuff[2] = req.id;
    gBuff[3] = 2 + req.data.size;

    uint8_t chksum = gBuff[2] + gBuff[3];

    for (size_t i = 0; i < req.data.size; ++i) {
        gBuff[4 + i] = req.data.bytes[i];
        chksum += req.data.bytes[i];
    }

    gBuff[4 + req.data.size] = ~chksum;
    gBuffSize = 4 + req.data.size + 1;
    gBuffIndex = 1;
    gExpectResponse = req.expect_response;

    gResponse.id = req.id;
    gResponse.data.size = 0;

    LL_USART_SetTransferDirection(servoUart, LL_USART_DIRECTION_TX);

    LL_USART_ClearFlag_TC(servoUart);
    LL_USART_TransmitData8(servoUart, gBuff[0]);
    LL_USART_EnableIT_TXE(servoUart);
}
