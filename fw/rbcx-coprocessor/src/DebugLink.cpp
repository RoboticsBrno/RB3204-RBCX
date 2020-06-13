#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_ll_usart.h"

#include "Bsp.hpp"
#include "coproc_codec.h"
#include "coproc_link_parser.h"
#include "rbcx.pb.h"
#include "utils/ByteFifo.hpp"
#include "utils/Debug.hpp"
#include <array>

static DMA_HandleTypeDef dmaTxHandle;

static std::array<uint8_t, 256> txDmaBuf;

static StaticStreamBuffer_t _txStreamBufStruct;
static uint8_t _txStreamBufStorage[txDmaBuf.size() * 4];
static StreamBufferHandle_t txStreamBuf;

void debugUartInit() {
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

    txStreamBuf = xStreamBufferCreateStatic(sizeof(_txStreamBufStorage), 1,
        _txStreamBufStorage, &_txStreamBufStruct);

    pinInit(debugUartTxPin, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
}

extern "C" int _write(int fd, char* data, int len) {
    if ((size_t)len > sizeof(txDmaBuf))
        return -1;

    size_t res = 0;

    if (isInInterrupt()) {
        const auto status = taskENTER_CRITICAL_FROM_ISR();
        res = xStreamBufferSendFromISR(txStreamBuf, data, len, 0);
        taskEXIT_CRITICAL_FROM_ISR(status);
    } else {
        while ((int)xStreamBufferSpacesAvailable(txStreamBuf) < len)
            vTaskDelay(0);
        taskENTER_CRITICAL();
        res = xStreamBufferSend(txStreamBuf, data, len, 0);
        taskEXIT_CRITICAL();
    }

    if (res == 0)
        return -1;

    HAL_NVIC_SetPendingIRQ(debugUartTxDmaIRQn);
    return res;
}

extern "C" void DEBUGUART_TX_DMA_HANDLER() {
    HAL_DMA_IRQHandler(&dmaTxHandle);

    if (dmaTxHandle.State == HAL_DMA_STATE_READY) {
        BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
        auto len = xStreamBufferReceiveFromISR(txStreamBuf, txDmaBuf.data(),
            txDmaBuf.size(), &pxHigherPriorityTaskWoken);

        if (len > 0) {
            HAL_DMA_Start_IT(&dmaTxHandle, uint32_t(txDmaBuf.data()),
                uint32_t(&debugUart->DR), len);
        }

        portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    }
}
