#pragma once

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include <algorithm>
#include <array>
#include <cassert>

inline void pinInit(GPIO_TypeDef* port, uint16_t pinMask, uint32_t mode, uint32_t pull, uint32_t speed) {
    GPIO_InitTypeDef init;
    init.Pin = pinMask;
    init.Mode = mode;
    init.Pull = pull;
    init.Speed = speed;
    HAL_GPIO_Init(port, &init);
}

inline void pinsInit() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // LED
    pinInit(GPIOC, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    HAL_GPIO_WritePin(GPIOC, 13, GPIO_PIN_SET);

    // USB
    pinInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
}
