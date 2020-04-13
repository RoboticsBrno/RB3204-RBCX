#pragma once

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_ll_usart.h"

DMA_HandleTypeDef TunnelDmaRxHandle;
DMA_HandleTypeDef TunnelDmaTxHandle;

inline void pin_init(GPIO_TypeDef *port, uint16_t pinMask, uint32_t mode, uint32_t pull, uint32_t speed)
{
    GPIO_InitTypeDef init;
    init.Pin = pinMask;
    init.Mode = mode;
    init.Pull = pull;
    init.Speed = speed;
    HAL_GPIO_Init(port, &init);
}

inline void tunnel_usart_init()
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

    TunnelDmaTxHandle.Instance = DMA1_Channel4;
    TunnelDmaTxHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    TunnelDmaTxHandle.Init.Mode = DMA_NORMAL;
    TunnelDmaTxHandle.Init.MemInc = DMA_MINC_ENABLE;
    TunnelDmaTxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    TunnelDmaTxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    TunnelDmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    TunnelDmaTxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&TunnelDmaTxHandle);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    pin_init(GPIOA, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    pin_init(GPIOA, GPIO_PIN_10, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
}

inline void tunnel_usart_tx(uint8_t *data, size_t len)
{
    HAL_DMA_Start(&TunnelDmaTxHandle, uint32_t(data), uint32_t(&USART1->DR), len);
    LL_USART_EnableDMAReq_TX(USART1);
}

inline bool tunnel_usart_poll()
{
    HAL_DMA_PollForTransfer(&TunnelDmaTxHandle, HAL_DMA_FULL_TRANSFER, 0);
    if (TunnelDmaTxHandle.State == HAL_DMA_STATE_BUSY)
    {
        return false;
    }
    else
    {
        return true;
    }
}
