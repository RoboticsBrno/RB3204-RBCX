#pragma once
/// Board Support Package maps application onto concrete pins/peripherals

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"

#include <algorithm>
#include <array>
#include <cassert>

using PinDef = std::pair<GPIO_TypeDef*, uint16_t>;

inline const PinDef led1Pin = std::make_pair(GPIOD, GPIO_PIN_10);
inline const PinDef led2Pin = std::make_pair(GPIOD, GPIO_PIN_11);
inline const PinDef led3Pin = std::make_pair(GPIOD, GPIO_PIN_14);
inline const PinDef led4Pin = std::make_pair(GPIOD, GPIO_PIN_15);
inline const PinDef ledPins = std::make_pair(GPIOD, led1Pin.second
                                                  | led2Pin.second
                                                  | led3Pin.second
                                                  | led4Pin.second );
inline const std::array<PinDef, 5> ledPin = { ledPins, led1Pin, led2Pin, led3Pin, led4Pin };

inline const PinDef powerPin = std::make_pair(GPIOD, GPIO_PIN_9);

inline const PinDef buzzerPin = std::make_pair(GPIOD, GPIO_PIN_7);

inline const PinDef buttonOffPin = std::make_pair(GPIOE, GPIO_PIN_15);
inline const PinDef button1Pin   = std::make_pair(GPIOB, GPIO_PIN_6);
inline const PinDef button2Pin   = std::make_pair(GPIOD, GPIO_PIN_3);
inline const PinDef button3Pin   = std::make_pair(GPIOB, GPIO_PIN_2);
inline const PinDef button4Pin   = std::make_pair(GPIOD, GPIO_PIN_4);
inline const PinDef buttonOnPin  = std::make_pair(GPIOC, GPIO_PIN_9);
inline const std::array<PinDef, 6> buttonPin = { buttonOffPin, button1Pin, button2Pin, button3Pin, button4Pin, buttonOnPin };

inline const PinDef usbDnPin = std::make_pair(GPIOA, GPIO_PIN_11);
inline const PinDef usbDpPin = std::make_pair(GPIOA, GPIO_PIN_12);
inline const PinDef usbDpPullUpPin = std::make_pair(USBD_DP_PORT, 1<<USBD_DP_PIN); // defined in platformio.ini, because used by STM USB C library
inline const PinDef usbBusDetectionPin = std::make_pair(GPIOC, GPIO_PIN_13);

inline const PinDef   debugUartTxPin = std::make_pair(GPIOA, GPIO_PIN_9);
inline const PinDef   debugUartRxPin = std::make_pair(GPIOA, GPIO_PIN_10);
inline const PinDef    userUartTxPin = std::make_pair(GPIOD, GPIO_PIN_5);
inline const PinDef    userUartRxPin = std::make_pair(GPIOD, GPIO_PIN_6);
inline const PinDef   servoUartTxPin = std::make_pair(GPIOD, GPIO_PIN_8); // RX is the same pin - half-duplex communication
inline const PinDef controlUartTxPin = std::make_pair(GPIOC, GPIO_PIN_10);
inline const PinDef controlUartRxPin = std::make_pair(GPIOC, GPIO_PIN_11);
// inline const PinDef   tunnelUartTxPin = std::make_pair(GPIOC, GPIO_PIN_12);
// inline const PinDef   tunnelUartRxPin = std::make_pair(GPIOD, GPIO_PIN_2);
inline const PinDef   tunnelUartTxPin = std::make_pair(GPIOA, GPIO_PIN_9);
inline const PinDef   tunnelUartRxPin = std::make_pair(GPIOA, GPIO_PIN_10);

inline USART_TypeDef * const   debugUart = USART1;
inline USART_TypeDef * const    userUart = USART2;
inline USART_TypeDef * const   servoUart = USART3;
inline USART_TypeDef * const controlUart = UART4;
inline USART_TypeDef * const   tunnelUart = USART1;//UART5;

inline DMA_Channel_TypeDef * const   tunnelUartTxDmaChannel = DMA1_Channel4;
inline DMA_Channel_TypeDef * const   tunnelUartRxDmaChannel = DMA1_Channel5;

inline DMA_Channel_TypeDef * const controlUartTxDmaChannel = DMA1_Channel7;
inline DMA_Channel_TypeDef * const controlUartRxDmaChannel = DMA1_Channel6;

inline const PinDef pwm1bPin = std::make_pair(GPIOE, GPIO_PIN_8);
inline const PinDef pwm1aPin = std::make_pair(GPIOE, GPIO_PIN_9);
inline const PinDef pwm2bPin = std::make_pair(GPIOE, GPIO_PIN_10);
inline const PinDef pwm2aPin = std::make_pair(GPIOE, GPIO_PIN_11);
inline const PinDef pwm3bPin = std::make_pair(GPIOE, GPIO_PIN_12);
inline const PinDef pwm3aPin = std::make_pair(GPIOE, GPIO_PIN_13);
inline const PinDef pwm4Pin  = std::make_pair(GPIOE, GPIO_PIN_14);
inline const PinDef in4bPin  = std::make_pair(GPIOD, GPIO_PIN_0);
inline const PinDef in4aPin  = std::make_pair(GPIOD, GPIO_PIN_1);

inline TIM_TypeDef * const pwmTimer = TIM1;
inline static constexpr uint32_t timerIndex2channel[4] = {
    LL_TIM_CHANNEL_CH1,
    LL_TIM_CHANNEL_CH2,
    LL_TIM_CHANNEL_CH3,
    LL_TIM_CHANNEL_CH4
};
inline static constexpr uint32_t timerIndex2negativeChannel[3] = {
    LL_TIM_CHANNEL_CH1N,
    LL_TIM_CHANNEL_CH2N,
    LL_TIM_CHANNEL_CH3N
};
inline static void (* const LL_TIM_OC_SetCompareCH[4])(TIM_TypeDef *TIMx, uint32_t CompareValue) = {
    LL_TIM_OC_SetCompareCH1,
    LL_TIM_OC_SetCompareCH2,
    LL_TIM_OC_SetCompareCH3,
    LL_TIM_OC_SetCompareCH4
};

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
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    assert(HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    assert(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) == HAL_OK);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    assert(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) == HAL_OK);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
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
    pinInit(ledPins, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    pinWrite(powerPin, 1);
    pinInit(powerPin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    pinInit(buzzerPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    for (auto button: buttonPin)
       pinInit(button, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);

    pinInit(pwm1bPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm1aPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm2bPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm2aPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm3bPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm3aPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm4Pin , GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(in4bPin , GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(in4aPin , GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    LL_GPIO_AF_EnableRemap_TIM1();

    // USB
    pinInit(usbDnPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(usbDpPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(usbDpPullUpPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(usbBusDetectionPin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
}

inline bool isPressed(PinDef button) {
    return button == buttonOnPin ? pinRead(button) : !pinRead(button);
}