#pragma once
/// Board Support Package maps application onto concrete pins/peripherals

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include <algorithm>
#include <array>
#include <cassert>

using PinDef = std::pair<GPIO_TypeDef*, uint16_t>;

inline const PinDef led1Pin = std::make_pair(GPIOC, 13);
inline const PinDef usbDp = std::make_pair(GPIOA, 12);
inline const PinDef usbDn = std::make_pair(GPIOA, 11);

inline const PinDef primaryTx = std::make_pair(GPIOA, 9);
inline const PinDef primaryRx = std::make_pair(GPIOA, 10);
inline const PinDef secondaryTx = std::make_pair(GPIOA, 2);
inline const PinDef secondaryRx = std::make_pair(GPIOA, 3);
inline DMA_Channel_TypeDef * const primaryTxDmaChannel = DMA1_Channel4;
inline DMA_Channel_TypeDef * const primaryRxDmaChannel = DMA1_Channel5;
inline USART_TypeDef * const primaryUsart = USART1;
inline DMA_Channel_TypeDef * const secondaryTxDmaChannel = DMA1_Channel7;
inline DMA_Channel_TypeDef * const secondaryRxDmaChannel = DMA1_Channel6;
inline USART_TypeDef * const secondaryUsart = USART2;

inline void clocksInit() {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        assert(false);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        assert(false);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        assert(false);
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
}

inline void pinInit(GPIO_TypeDef* port, uint16_t pinMask, uint32_t mode, uint32_t pull, uint32_t speed) {
    GPIO_InitTypeDef init;
    init.Pin = pinMask;
    init.Mode = mode;
    init.Pull = pull;
    init.Speed = speed;
    HAL_GPIO_Init(port, &init);
}

inline void pinInit(PinDef pin, uint32_t mode, uint32_t pull, uint32_t speed) {
    pinInit(pin.first, 1 << pin.second, mode, pull, speed);
}

inline void pinsInit() {
    pinInit(led1Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    HAL_GPIO_WritePin(led1Pin.first, led1Pin.second, GPIO_PIN_SET);

    // USB
    pinInit(usbDp, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(usbDn, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
}
