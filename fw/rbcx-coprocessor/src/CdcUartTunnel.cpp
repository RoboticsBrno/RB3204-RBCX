/// Implements passthrough between the "primary" UART and USB CDC.

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_ll_usart.h"

#include "Bsp.hpp"
#include "ByteFifo.hpp"
#include "CdcUartTunnel.hpp"
#include "UsbCdcLink.h"

DMA_HandleTypeDef primaryUartDmaRxHandle;
DMA_HandleTypeDef primaryUartDmaTxHandle;
ByteFifo<512> primaryUartRxFifo;
std::array<uint8_t, CDC_DATA_SZ> primaryUartTxBuf;

void primaryUartInit() {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    LL_USART_InitTypeDef init;
    LL_USART_StructInit(&init);
    init.BaudRate = 115200;
    init.DataWidth = LL_USART_DATAWIDTH_8B;
    init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    init.Parity = LL_USART_PARITY_NONE;
    init.StopBits = LL_USART_STOPBITS_1;
    init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    LL_USART_Init(USART1, &init);
    LL_USART_Enable(USART1);

    // UART RX runs indefinitely in circular mode
    primaryUartDmaRxHandle.Instance = DMA1_Channel5;
    primaryUartDmaRxHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    primaryUartDmaRxHandle.Init.Mode = DMA_CIRCULAR;
    primaryUartDmaRxHandle.Init.MemInc = DMA_MINC_ENABLE;
    primaryUartDmaRxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    primaryUartDmaRxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    primaryUartDmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    primaryUartDmaRxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&primaryUartDmaRxHandle);
    HAL_DMA_Start(&primaryUartDmaRxHandle, uint32_t(&(USART1->DR)), uint32_t(primaryUartRxFifo.data()), primaryUartRxFifo.size());
    LL_USART_EnableDMAReq_RX(USART1);

    // UART TX burst is started ad hoc each time
    primaryUartDmaTxHandle.Instance = DMA1_Channel4;
    primaryUartDmaTxHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    primaryUartDmaTxHandle.Init.Mode = DMA_NORMAL;
    primaryUartDmaTxHandle.Init.MemInc = DMA_MINC_ENABLE;
    primaryUartDmaTxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    primaryUartDmaTxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    primaryUartDmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    primaryUartDmaTxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&primaryUartDmaTxHandle);
    LL_USART_EnableDMAReq_TX(USART1);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    pinInit(GPIOA, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    pinInit(GPIOA, GPIO_PIN_10, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
}

void primaryUartRxPoll() {
    int rxHead = primaryUartRxFifo.size() - __HAL_DMA_GET_COUNTER(&primaryUartDmaRxHandle);
    primaryUartRxFifo.setHead(rxHead);
}

void primaryUartTx(uint8_t* data, size_t len) {
    HAL_DMA_Start(&primaryUartDmaTxHandle, uint32_t(data), uint32_t(&USART1->DR), len);
}

bool primaryUartTxReady() {
    HAL_DMA_PollForTransfer(&primaryUartDmaTxHandle, HAL_DMA_FULL_TRANSFER, 0);
    return primaryUartDmaTxHandle.State == HAL_DMA_STATE_READY;
}

void tunnelDownstreamHandler() {
    if (primaryUartTxReady()) {
        int transferred = usbd_ep_read(&udev, CDC_RXD_EP, primaryUartTxBuf.data(), primaryUartTxBuf.size());
        if (transferred > 0) {
            primaryUartTx(primaryUartTxBuf.data(), transferred);
        }
    }
}

void tunnelUpstreamHandler() {
    primaryUartRxPoll();
    auto readable = primaryUartRxFifo.readableSpan();
    if (readable.second > 0) {
        int transferred = usbd_ep_write(&udev, CDC_TXD_EP, readable.first, std::min(readable.second, size_t(CDC_DATA_SZ)));
        if (transferred > 0) {
            primaryUartRxFifo.notifyRead(transferred);
        }
    }
}

void tunnelPoll() {
    tunnelUpstreamHandler();
    tunnelDownstreamHandler();
}
