#include "SmartServoController.hpp"
#include "Bsp.hpp"
#include "ControlLink.hpp"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "utils/Debug.hpp"
#include "utils/TickTimer.hpp"

enum SmartServoState { IDLE, TX, RX };

// assert that response fits into gBuff
static_assert(sizeof(CoprocStat_SmartServoStat_data_t::bytes)
    <= sizeof(CoprocReq_SmartServoReq_data_t::bytes));

static std::array<uint8_t, sizeof(CoprocReq_SmartServoReq_data_t::bytes)> gBuff;

static size_t gBuffSize = 0;
static size_t gBuffIndex = 0;

static SmartServoState gState = IDLE;
static bool gExpectResponse = false;
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
        CoprocStat stat = {
            .which_payload = CoprocStat_smartServoStat_tag,
            .payload = {
                .smartServoStat = {
                    .data = {
                        .size = gBuffSize,
                    },
                },
            },
        };
        memcpy(stat.payload.smartServoStat.data.bytes, gBuff.data(), gBuffSize);
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

            gBuffSize = 0;

            if (!gExpectResponse) {
                smartServoSetResponseReady();
                gState = IDLE;
            } else {
                gState = RX;
                LL_USART_EnableIT_TC(servoUart);

                gResponseTimeout.restart(5);
            }
        }
    }

    if (gState == RX) {
        // Transmission complete
        if (LL_USART_IsActiveFlag_TC(servoUart)) {
            LL_USART_DisableIT_TC(servoUart);
            LL_USART_ClearFlag_TC(servoUart);
            LL_USART_ClearFlag_IDLE(servoUart);

            LL_USART_SetTransferDirection(servoUart, LL_USART_DIRECTION_RX);

            LL_USART_ReceiveData8(servoUart);
            LL_USART_EnableIT_RXNE(servoUart);

            gResponseTimeout.restart(2);
        }

        // RX not empty
        if (LL_USART_IsActiveFlag_RXNE(servoUart)) {
            const uint8_t ch = LL_USART_ReceiveData8(servoUart);

            if (gBuffSize < gBuff.size()) {
                gBuff[gBuffSize++] = ch;
            } else {
                DEBUGLN("SmartServo RX buffer overrun!");
            }
            gResponseTimeout.restart(2);
        }

        // RX overrun, is enabled by the same bit as RXNE so we have to clear it
        if (LL_USART_IsActiveFlag_ORE(servoUart)) {
            LL_USART_ClearFlag_ORE(servoUart);
            DEBUGLN("SmartServo ORE bit set!");
        }
    }
}

void smartServoSendRequest(const CoprocReq_SmartServoReq& req) {
    if (gState != IDLE) {
        DEBUGLN("Invalid CoprocReq_SmartServoReq, state is %d", gState);
        return;
    }

    if (req.data.size == 0) {
        gBuffSize = 0;
        smartServoSetResponseReady();
        return;
    }

    gStateTimeout.restart(10);
    gState = TX;

    memcpy(gBuff.data(), req.data.bytes, req.data.size);
    gBuffSize = req.data.size;
    gBuffIndex = 1;
    gExpectResponse = req.expect_response;

    LL_USART_SetTransferDirection(servoUart, LL_USART_DIRECTION_TX);

    LL_USART_ClearFlag_TC(servoUart);
    LL_USART_TransmitData8(servoUart, gBuff[0]);
    LL_USART_EnableIT_TXE(servoUart);
}
