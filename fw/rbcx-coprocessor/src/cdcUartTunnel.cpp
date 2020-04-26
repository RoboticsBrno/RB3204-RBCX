/// Implements passthrough between the "primary" UART and USB CDC.

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_ll_usart.h"

#include "bsp.hpp"
#include "byteFifo.hpp"
#include "cdcUartTunnel.hpp"
#include "usbCdcLink.h"

DMA_HandleTypeDef PrimaryUartDmaRxHandle;
DMA_HandleTypeDef PrimaryUartDmaTxHandle;
ByteFifo<512> PrimaryUartRxFifo;
std::array<uint8_t, CDC_DATA_SZ> PrimaryUartTxBuf;

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
    PrimaryUartDmaRxHandle.Instance = DMA1_Channel5;
    PrimaryUartDmaRxHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    PrimaryUartDmaRxHandle.Init.Mode = DMA_CIRCULAR;
    PrimaryUartDmaRxHandle.Init.MemInc = DMA_MINC_ENABLE;
    PrimaryUartDmaRxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    PrimaryUartDmaRxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    PrimaryUartDmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    PrimaryUartDmaRxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&PrimaryUartDmaRxHandle);
    HAL_DMA_Start(&PrimaryUartDmaRxHandle, uint32_t(&(USART1->DR)), uint32_t(PrimaryUartRxFifo.data()), PrimaryUartRxFifo.size());
    LL_USART_EnableDMAReq_RX(USART1);

    // UART TX burst is started ad hoc each time
    PrimaryUartDmaTxHandle.Instance = DMA1_Channel4;
    PrimaryUartDmaTxHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    PrimaryUartDmaTxHandle.Init.Mode = DMA_NORMAL;
    PrimaryUartDmaTxHandle.Init.MemInc = DMA_MINC_ENABLE;
    PrimaryUartDmaTxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    PrimaryUartDmaTxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    PrimaryUartDmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    PrimaryUartDmaTxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&PrimaryUartDmaTxHandle);
    LL_USART_EnableDMAReq_TX(USART1);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    pinInit(GPIOA, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    pinInit(GPIOA, GPIO_PIN_10, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
}

void primaryUartRxPoll() {
    int rxHead = PrimaryUartRxFifo.size() - __HAL_DMA_GET_COUNTER(&PrimaryUartDmaRxHandle);
    PrimaryUartRxFifo.setHead(rxHead);
}

void primaryUartTx(uint8_t* data, size_t len) {
    HAL_DMA_Start(&PrimaryUartDmaTxHandle, uint32_t(data), uint32_t(&USART1->DR), len);
}

bool primaryUartTxReady() {
    HAL_DMA_PollForTransfer(&PrimaryUartDmaTxHandle, HAL_DMA_FULL_TRANSFER, 0);
    return PrimaryUartDmaTxHandle.State == HAL_DMA_STATE_READY;
}

void tunnelDownstreamHandler() {
    if (primaryUartTxReady()) {
        int transferred = usbd_ep_read(&udev, CDC_RXD_EP, PrimaryUartTxBuf.data(), PrimaryUartTxBuf.size());
        if (transferred > 0) {
            primaryUartTx(PrimaryUartTxBuf.data(), transferred);
        }
    }
}

void tunnelUpstreamHandler() {
    primaryUartRxPoll();
    auto readable = PrimaryUartRxFifo.readableRange();
    if (readable.second > 0) {
        int transferred = usbd_ep_write(&udev, CDC_TXD_EP, readable.first, std::min(readable.second, size_t(CDC_DATA_SZ)));
        if (transferred > 0) {
            PrimaryUartRxFifo.notifyRead(transferred);
        }
    }
}

void tunnelPoll() {
    tunnelUpstreamHandler();
    tunnelDownstreamHandler();
}
