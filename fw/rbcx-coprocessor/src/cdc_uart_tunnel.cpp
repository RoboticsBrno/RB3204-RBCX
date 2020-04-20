#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "usb.h"
#include "usb_cdc_link.h"
#include "bsp.hpp"

DMA_HandleTypeDef TunnelDmaRxHandle;
DMA_HandleTypeDef TunnelDmaTxHandle;
ByteFifo<512> TunnelRxFifo;
std::array<uint8_t, CDC_DATA_SZ> TunnelTxBuf;

void tunnel_uart_init()
{
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

    TunnelDmaRxHandle.Instance = DMA1_Channel5;
    TunnelDmaRxHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    TunnelDmaRxHandle.Init.Mode = DMA_CIRCULAR;
    TunnelDmaRxHandle.Init.MemInc = DMA_MINC_ENABLE;
    TunnelDmaRxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    TunnelDmaRxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    TunnelDmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    TunnelDmaRxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&TunnelDmaRxHandle);
    HAL_DMA_Start(&TunnelDmaRxHandle, uint32_t(&(USART1->DR)), uint32_t(TunnelRxFifo.data()), TunnelRxFifo.size());
    LL_USART_EnableDMAReq_RX(USART1);

    TunnelDmaTxHandle.Instance = DMA1_Channel4;
    TunnelDmaTxHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    TunnelDmaTxHandle.Init.Mode = DMA_NORMAL;
    TunnelDmaTxHandle.Init.MemInc = DMA_MINC_ENABLE;
    TunnelDmaTxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    TunnelDmaTxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    TunnelDmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    TunnelDmaTxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&TunnelDmaTxHandle);
    LL_USART_EnableDMAReq_TX(USART1);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    pin_init(GPIOA, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    pin_init(GPIOA, GPIO_PIN_10, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
}

void tunnel_uart_rx_poll()
{
    int rxHead = TunnelRxFifo.size() - __HAL_DMA_GET_COUNTER(&TunnelDmaRxHandle);
    TunnelRxFifo.set_head(rxHead);
}

void tunnel_uart_tx(uint8_t *data, size_t len)
{
    if (len > 0)
    {
        HAL_DMA_Start(&TunnelDmaTxHandle, uint32_t(data), uint32_t(&USART1->DR), len);
    }
}

bool tunnel_uart_tx_done()
{
    HAL_DMA_PollForTransfer(&TunnelDmaTxHandle, HAL_DMA_FULL_TRANSFER, 0);
    bool txBusy = TunnelDmaTxHandle.State == HAL_DMA_STATE_BUSY;

    return !txBusy;
}

extern "C" void cdc_link_rx_handler(usbd_device *dev, uint8_t ep)
{
    if (tunnel_uart_tx_done())
    {
        int received = usbd_ep_read(dev, ep, TunnelTxBuf.data(), TunnelTxBuf.size());
        if (received < 0)
        {
            __BKPT();
        }
        tunnel_uart_tx(TunnelTxBuf.data(), received);
    }
}

extern "C" void cdc_link_tx_handler(usbd_device *dev, uint8_t ep)
{
    tunnel_uart_rx_poll();
    auto readable = TunnelRxFifo.readable_range();
    if (readable.second > 0)
    {
        int transferred = usbd_ep_write(dev, ep, readable.first, std::min(readable.second, size_t(CDC_DATA_SZ)));
        TunnelRxFifo.notify_read(transferred);
    }
}
