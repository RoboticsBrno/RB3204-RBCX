#pragma once
/// Board Support Package maps application onto concrete pins/peripherals

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include <algorithm>
#include <array>
#include <cassert>

using PinDef = std::pair<GPIO_TypeDef*, uint16_t>;

inline const PinDef led1Pin = std::make_pair(GPIOD, GPIO_PIN_10);
inline const PinDef led2Pin = std::make_pair(GPIOD, GPIO_PIN_11);
inline const PinDef led3Pin = std::make_pair(GPIOD, GPIO_PIN_14);
inline const PinDef led4Pin = std::make_pair(GPIOD, GPIO_PIN_15);

inline const std::array<PinDef, 4> ledPin = { led1Pin, led2Pin, led3Pin, led4Pin };

inline const PinDef usbDp = std::make_pair(GPIOA, GPIO_PIN_12);
inline const PinDef usbDn = std::make_pair(GPIOA, GPIO_PIN_11);

inline const PinDef   debugUartTxPin = std::make_pair(GPIOA, GPIO_PIN_9);
inline const PinDef   debugUartRxPin = std::make_pair(GPIOA, GPIO_PIN_10);
inline const PinDef    userUartTxPin = std::make_pair(GPIOD, GPIO_PIN_5);
inline const PinDef    userUartRxPin = std::make_pair(GPIOD, GPIO_PIN_6);
inline const PinDef   servoUartTxPin = std::make_pair(GPIOD, GPIO_PIN_8); // RX is the same pin - half-duplex communication
inline const PinDef controlUartTxPin = std::make_pair(GPIOC, GPIO_PIN_10);
inline const PinDef controlUartRxPin = std::make_pair(GPIOC, GPIO_PIN_11);
inline const PinDef   tunelUartTxPin = std::make_pair(GPIOC, GPIO_PIN_12);
inline const PinDef   tunelUartRxPin = std::make_pair(GPIOD, GPIO_PIN_2);

inline USART_TypeDef * const   debugUart = USART1;
inline USART_TypeDef * const    userUart = USART2;
inline USART_TypeDef * const   servoUart = USART3;
inline USART_TypeDef * const controlUart = UART4;
inline USART_TypeDef * const   tunelUart = UART5;

inline DMA_Channel_TypeDef * const   tunelUartTxDmaChannel = DMA1_Channel4;
inline DMA_Channel_TypeDef * const   tunelUartRxDmaChannel = DMA1_Channel5;

inline DMA_Channel_TypeDef * const controlUartTxDmaChannel = DMA1_Channel7;
inline DMA_Channel_TypeDef * const controlUartRxDmaChannel = DMA1_Channel6;

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
    assert(HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    assert(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) == HAL_OK);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    assert(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) == HAL_OK);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
}

inline void pinInit(GPIO_TypeDef* port, uint32_t pinMask, uint32_t mode, uint32_t pull, uint32_t speed) {
    GPIO_InitTypeDef init;
    init.Pin = pinMask;
    init.Mode = mode;
    init.Pull = pull;
    init.Speed = speed;
    HAL_GPIO_Init(port, &init);
}

inline void pinInit(PinDef pin, uint32_t mode, uint32_t pull, uint32_t speed) {
    pinInit(pin.first, pin.second, mode, pull, speed);
}

inline bool pinRead(PinDef pin) {
    return HAL_GPIO_ReadPin(pin.first, pin.second) == GPIO_PIN_SET;
}

inline void pinWrite(PinDef pin, bool value) {
    HAL_GPIO_WritePin(pin.first, pin.second, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

inline void pinToggle(PinDef pin) {
    HAL_GPIO_TogglePin(pin.first, pin.second);
}

inline void pinsInit() {

    for (auto led: ledPin)
        pinInit(led, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    // HAL_GPIO_WritePin(led1Pin.first, led1Pin.second, GPIO_PIN_SET);

    // USB
    pinInit(usbDp, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(usbDn, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
}
