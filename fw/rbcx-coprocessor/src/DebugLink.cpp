#include <array>
#include <string.h>

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_ll_usart.h"

#include "Bsp.hpp"
#include "Dispatcher.hpp"
#include "Power.hpp"
#include "coproc_codec.h"
#include "coproc_link_parser.h"
#include "rbcx.pb.h"
#include "utils/ByteFifo.hpp"
#include "utils/Debug.hpp"
#include "utils/MessageBufferWrapper.hpp"
#include "utils/QueueWrapper.hpp"
#include "utils/StreamBufferWrapper.hpp"

#include "rbcx.pb.h"

static DMA_HandleTypeDef dmaTxHandle;
static DMA_HandleTypeDef dmaRxHandle;

static std::array<uint8_t, 256> txDmaBuf;
static ByteFifo<128> rxFifo;

static StreamBufferWrapper<txDmaBuf.size() * 4> txStreamBuf;

static constexpr size_t MaxLineLength = 128;
static MessageBufferWrapper<MaxLineLength + 4> rxLineBuffer;

void debugUartInit() {
    txStreamBuf.create();
    rxLineBuffer.create();

    LL_USART_InitTypeDef init;
    LL_USART_StructInit(&init);
    init.BaudRate = 115200;
    init.DataWidth = LL_USART_DATAWIDTH_8B;
    init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    init.Parity = LL_USART_PARITY_NONE;
    init.StopBits = LL_USART_STOPBITS_1;
    init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    if (LL_USART_Init(debugUart, &init) != SUCCESS)
        abort();
    LL_USART_Enable(debugUart);

    // UART RX runs indefinitely in circular mode
    dmaRxHandle.Instance = debugUartRxDmaChannel;
    dmaRxHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dmaRxHandle.Init.Mode = DMA_CIRCULAR;
    dmaRxHandle.Init.MemInc = DMA_MINC_ENABLE;
    dmaRxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaRxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dmaRxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&dmaRxHandle);
    HAL_DMA_Start(&dmaRxHandle, uintptr_t(&(debugUart->DR)),
        uintptr_t(rxFifo.data()), rxFifo.size());
    LL_USART_EnableDMAReq_RX(debugUart);

    HAL_NVIC_SetPriority(debugUartIRQn, debugUartIrqPrio, 0);
    HAL_NVIC_EnableIRQ(debugUartIRQn);
    LL_USART_EnableIT_IDLE(debugUart);

    // UART TX burst is started ad hoc each time
    dmaTxHandle.Instance = debugUartTxDmaChannel;
    dmaTxHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dmaTxHandle.Init.Mode = DMA_NORMAL;
    dmaTxHandle.Init.MemInc = DMA_MINC_ENABLE;
    dmaTxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaTxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dmaTxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&dmaTxHandle) != HAL_OK)
        abort();
    HAL_NVIC_SetPriority(debugUartTxDmaIRQn, debugUartTxDmaIrqPrio, 0);
    HAL_NVIC_EnableIRQ(debugUartTxDmaIRQn);
    LL_USART_EnableDMAReq_TX(debugUart);

    pinInit(debugUartTxPin, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
}

ssize_t debugLinkTx(const uint8_t* data, size_t len) {
    if ((size_t)len > sizeof(txDmaBuf))
        return -1;

    size_t res = 0;

    if (isInInterrupt()) {
        BaseType_t woken = pdFALSE;
        const auto status = taskENTER_CRITICAL_FROM_ISR();
        res = txStreamBuf.write((uint8_t*)data, len, 0, &woken);
        taskEXIT_CRITICAL_FROM_ISR(status);
        portYIELD_FROM_ISR(woken);
    } else {
        while (txStreamBuf.freeSpace() < len)
            vTaskDelay(0);
        taskENTER_CRITICAL();
        res = txStreamBuf.write((uint8_t*)data, len, 0);
        taskEXIT_CRITICAL();
    }

    if (res == 0)
        return -1;

    HAL_NVIC_SetPendingIRQ(debugUartTxDmaIRQn);
    return res;
}

extern "C" int _write(int fd, char* data, int len) {
    return debugLinkTx((uint8_t*)data, len);
}

extern "C" void DEBUGUART_HANDLER(void) {
    int rxHead = rxFifo.size() - __HAL_DMA_GET_COUNTER(&dmaRxHandle);
    rxFifo.setHead(rxHead);
    LL_USART_ClearFlag_IDLE(debugUart);

    const auto fifoAvailable = rxFifo.available();
    char buf[rxFifo.size()];
    rxFifo.peekSpan((uint8_t*)buf, fifoAvailable);

    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    size_t remaining = fifoAvailable;
    char* start = buf;
    while (remaining != 0) {
        char* end = (char*)memchr(start, '\n', remaining);
        if (!end)
            break;

        *end = '\0';
        ++end;

        const size_t len = end - start;

        if (!rxLineBuffer.push_back(
                (uint8_t*)start, len, 0, &pxHigherPriorityTaskWoken)) {
            printf("Not enough space in line buffer, try sending commands "
                   "slower.\n");
        }

        remaining -= len;
        rxFifo.notifyRead(len);
        start = end;
    }
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

extern "C" void DEBUGUART_TX_DMA_HANDLER() {
    HAL_DMA_IRQHandler(&dmaTxHandle);

    if (dmaTxHandle.State == HAL_DMA_STATE_READY) {
        BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
        const auto len = txStreamBuf.read(
            txDmaBuf.data(), txDmaBuf.size(), 0, &pxHigherPriorityTaskWoken);

        if (len > 0) {
            HAL_DMA_Start_IT(&dmaTxHandle, uint32_t(txDmaBuf.data()),
                uint32_t(&debugUart->DR), len);
        }

        portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    }
}

static void debugLinkHandleCommand(const char* cmd) {
    if (strcmp(cmd, "help") == 0) {
        printf("Available commands: \n"
               "  calibrate power VCC_MV BATTERY_MID_MV VREF_33V_MV "
               "TEMPERATURE_C\n");
        return;
    }

    if (strncmp(cmd, "calibrate ", 10) == 0) {
        cmd += 10;

        if (strncmp(cmd, "power ", 6) == 0) {
            cmd += 6;

            CoprocReq req = {
                .which_payload = CoprocReq_calibratePower_tag,
            };

            if (sscanf(cmd, "%lu %lu %lu %lu",
                    &req.payload.calibratePower.vccMv,
                    &req.payload.calibratePower.battMidMv,
                    &req.payload.calibratePower.vRef33Mv,
                    &req.payload.calibratePower.temperatureC)
                != 4) {
                printf("Invalid parameters!\n");
                return;
            }

            dispatcherEnqueueRequest(req);
            return;
        }
    } else if (strncmp(cmd, "pid ", 4) == 0) {
        cmd += 4;

        CoprocReq req = {
            .which_payload = CoprocReq_motorReq_tag,
        };
        req.payload.motorReq.which_motorCmd
            = CoprocReq_MotorReq_setVelocityRegCoefs_tag;
        auto& c = req.payload.motorReq.motorCmd.setVelocityRegCoefs;
        if (sscanf(cmd, "%lu %lu %lu", &c.p, &c.i, &c.d) != 3) {
            printf("Invalid parameters!\n");
            return;
        }

        for (int m : { 0, 1, 2, 3 }) {
            req.payload.motorReq.motorIndex = m;
            dispatcherEnqueueRequest(req);
        }
    }

    printf("Invalid command.\n");
}

void debugLinkPoll() {
    char buf[MaxLineLength];
    if (rxLineBuffer.pop_front((uint8_t*)buf, MaxLineLength, 0)) {
        debugLinkHandleCommand(buf);
    }
}
