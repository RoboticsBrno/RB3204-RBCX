#pragma once

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_usart.h"
#include "stm32f1xx_hal_gpio.h"

USART_HandleTypeDef TunnelUsartHandle;

inline void pin_init(GPIO_TypeDef *port, uint16_t pinMask, uint32_t mode, uint32_t pull, uint32_t speed)
{
    GPIO_InitTypeDef init;
    init.Pin = pinMask;
    init.Mode = mode;
    init.Pull = pull;
    init.Speed = speed;
    HAL_GPIO_Init(port, &init);
}

void tunnel_usart_init()
{
    TunnelUsartHandle.Instance = USART1;
    TunnelUsartHandle.Init.BaudRate = 115200;
    TunnelUsartHandle.Init.WordLength = USART_WORDLENGTH_8B;
    TunnelUsartHandle.Init.Mode = USART_MODE_TX_RX;
    TunnelUsartHandle.Init.Parity = USART_PARITY_NONE;
    TunnelUsartHandle.Init.StopBits = USART_STOPBITS_1;
    HAL_USART_Init(&TunnelUsartHandle);
}

extern "C" void HAL_USART_MspInit(USART_HandleTypeDef *husart)
{
    if (husart == &TunnelUsartHandle) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        pin_init(GPIOA, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
        pin_init(GPIOA, GPIO_PIN_10, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    }
}
