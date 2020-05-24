/// Implements passthrough between the "primary" UART and USB CDC.
/// Primary UART leads to ESP32 main UART.

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_ll_usart.h"

#include "Bsp.hpp"
#include "ByteFifo.hpp"
#include "CdcUartTunnel.hpp"
#include "UsbCdcLink.h"

static DMA_HandleTypeDef dmaRxHandle;
static DMA_HandleTypeDef dmaTxHandle;
static ByteFifo<512> rxFifo;
static std::array<uint8_t, CDC_DATA_SZ> txBuf;

void primaryUartInit() {
    LL_USART_InitTypeDef init;
    LL_USART_StructInit(&init);
    init.BaudRate = 115200;
    init.DataWidth = LL_USART_DATAWIDTH_8B;
    init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    init.Parity = LL_USART_PARITY_NONE;
    init.StopBits = LL_USART_STOPBITS_1;
    init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    LL_USART_Init(primaryUsart, &init);
    LL_USART_Enable(primaryUsart);

    // UART RX runs indefinitely in circular mode
    dmaRxHandle.Instance = primaryRxDmaChannel;
    dmaRxHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dmaRxHandle.Init.Mode = DMA_CIRCULAR;
    dmaRxHandle.Init.MemInc = DMA_MINC_ENABLE;
    dmaRxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaRxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dmaRxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&dmaRxHandle);
    HAL_DMA_Start(&dmaRxHandle, uint32_t(&(primaryUsart->DR)),
        uint32_t(rxFifo.data()), rxFifo.size());
    LL_USART_EnableDMAReq_RX(primaryUsart);

    // UART TX burst is started ad hoc each time
    dmaTxHandle.Instance = primaryTxDmaChannel;
    dmaTxHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dmaTxHandle.Init.Mode = DMA_NORMAL;
    dmaTxHandle.Init.MemInc = DMA_MINC_ENABLE;
    dmaTxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaTxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dmaTxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&dmaTxHandle);
    LL_USART_EnableDMAReq_TX(primaryUsart);

    pinInit(primaryTx, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    pinInit(primaryRx, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
}

static void primaryUartRxPoll() {
    int rxHead = rxFifo.size() - __HAL_DMA_GET_COUNTER(&dmaRxHandle);
    rxFifo.setHead(rxHead);
}

static void primaryUartTx(uint8_t* data, size_t len) {
    HAL_DMA_Start(
        &dmaTxHandle, uint32_t(data), uint32_t(&primaryUsart->DR), len);
}

static bool primaryUartTxReady() {
    HAL_DMA_PollForTransfer(&dmaTxHandle, HAL_DMA_FULL_TRANSFER, 0);
    return dmaTxHandle.State == HAL_DMA_STATE_READY;
}

static void tunnelDownstreamHandler() {
    if (primaryUartTxReady()) {
        int transferred
            = usbd_ep_read(&udev, CDC_RXD_EP, txBuf.data(), txBuf.size());
        if (transferred > 0) {
            primaryUartTx(txBuf.data(), transferred);
        }
    }
}

static void tunnelUpstreamHandler() {
    primaryUartRxPoll();
    auto readable = rxFifo.readableSpan();
    if (readable.second > 0) {
        int transferred = usbd_ep_write(&udev, CDC_TXD_EP, readable.first,
            std::min(readable.second, size_t(CDC_DATA_SZ)));
        if (transferred > 0) {
            rxFifo.notifyRead(transferred);
        }
    }
}

void tunnelPoll() {
    tunnelUpstreamHandler();
    tunnelDownstreamHandler();
}
