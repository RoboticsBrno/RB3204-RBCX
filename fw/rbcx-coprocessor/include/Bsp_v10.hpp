#pragma once
/// Board Support Package maps application onto concrete pins/peripherals

#include "FreeRTOSConfig.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_cortex.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_utils.h"

#include <algorithm>
#include <array>
#include <cassert>

#include "Bsp.hpp"

inline const PinDef led1Pin = std::make_pair(GPIOD, GPIO_PIN_10);
inline const PinDef led2Pin = std::make_pair(GPIOD, GPIO_PIN_11);
inline const PinDef led3Pin = std::make_pair(GPIOD, GPIO_PIN_14);
inline const PinDef led4Pin = std::make_pair(GPIOD, GPIO_PIN_15);
inline const PinDef ledPins = std::make_pair(
    GPIOD, led1Pin.second | led2Pin.second | led3Pin.second | led4Pin.second);
inline const std::array<PinDef, 4> ledPin
    = { led1Pin, led2Pin, led3Pin, led4Pin };

inline const PinDef powerPin = std::make_pair(GPIOD, GPIO_PIN_9);

inline const PinDef buttonOffPin = std::make_pair(GPIOE, GPIO_PIN_15);
inline const PinDef button1Pin = std::make_pair(GPIOB, GPIO_PIN_6);
inline const PinDef button2Pin = std::make_pair(GPIOD, GPIO_PIN_3);
inline const PinDef button3Pin = std::make_pair(GPIOB, GPIO_PIN_2);
inline const PinDef button4Pin = std::make_pair(GPIOD, GPIO_PIN_4);
inline const PinDef buttonOnPin = std::make_pair(GPIOC, GPIO_PIN_9);
inline const std::array<PinDef, 6> buttonPin = { buttonOffPin, button1Pin,
    button2Pin, button3Pin, button4Pin, buttonOnPin };

inline const PinDef uts1TrigPin = std::make_pair(GPIOE, GPIO_PIN_0);
inline const PinDef uts1EchoPin = std::make_pair(GPIOE, GPIO_PIN_1);
inline const PinDef uts2TrigPin = std::make_pair(GPIOE, GPIO_PIN_2);
inline const PinDef uts2EchoPin = std::make_pair(GPIOE, GPIO_PIN_3);
inline const PinDef uts3TrigPin = std::make_pair(GPIOE, GPIO_PIN_4);
inline const PinDef uts3EchoPin = std::make_pair(GPIOE, GPIO_PIN_5);
inline const PinDef uts4TrigPin = std::make_pair(GPIOE, GPIO_PIN_6);
inline const PinDef uts4EchoPin = std::make_pair(GPIOE, GPIO_PIN_7);

inline const std::array<PinDef, 4> utsTrigPin
    = { uts1TrigPin, uts2TrigPin, uts3TrigPin, uts4TrigPin };
inline const std::array<PinDef, 4> utsEchoPin
    = { uts1EchoPin, uts2EchoPin, uts3EchoPin, uts4EchoPin };

inline const uint32_t utsIRQPrio = 7;
inline const std::array<IRQn_Type, 4> utsEchoIRQn = {
    EXTI1_IRQn,
    EXTI3_IRQn,
    EXTI9_5_IRQn,
    EXTI9_5_IRQn,
};

inline TIM_TypeDef* const utsTimer = TIM7;

inline const PinDef usbDnPin = std::make_pair(GPIOA, GPIO_PIN_11);
inline const PinDef usbDpPin = std::make_pair(GPIOA, GPIO_PIN_12);
// defined in platformio.ini, because used by STM USB C library:
inline const PinDef usbDpPullUpPin
    = std::make_pair(USBD_DP_PORT, 1 << USBD_DP_PIN);
inline const PinDef usbBusDetectionPin = std::make_pair(GPIOC, GPIO_PIN_13);

// We want DMA on tunnelUart, so on v1.0, we bridge UART1 to ESP
// manually, loose UART5 and use UART2 as debugUart.
// userUart is not available on v1.0
inline const PinDef tunnelUartTxPin = std::make_pair(GPIOA, GPIO_PIN_9);
inline const PinDef tunnelUartRxPin = std::make_pair(GPIOA, GPIO_PIN_10);
inline const PinDef debugUartTxPin = std::make_pair(GPIOD, GPIO_PIN_5);
inline const PinDef debugUartRxPin = std::make_pair(GPIOD, GPIO_PIN_6);
inline const PinDef servoUartTxRxPin = std::make_pair(GPIOD, GPIO_PIN_8);
inline const PinDef controlUartTxPin = std::make_pair(GPIOC, GPIO_PIN_10);
inline const PinDef controlUartRxPin = std::make_pair(GPIOC, GPIO_PIN_11);

inline const PinDef uart5TxPin = std::make_pair(GPIOC, GPIO_PIN_12);
inline const PinDef uart5RxPin = std::make_pair(GPIOD, GPIO_PIN_2);

inline const PinDef i2cSda = std::make_pair(GPIOB, GPIO_PIN_9);
inline const PinDef i2cScl = std::make_pair(GPIOB, GPIO_PIN_8);

inline const IRQn_Type i2cEvIRQn = I2C1_EV_IRQn;
inline const IRQn_Type i2cErIRQn = I2C1_ER_IRQn;
inline const unsigned i2cIRQnPrio = 8;

inline USART_TypeDef* const tunnelUart = USART1;
inline USART_TypeDef* const debugUart = USART2;
inline USART_TypeDef* const servoUart = USART3;
inline USART_TypeDef* const controlUart = UART4;

inline DMA_Channel_TypeDef* const tunnelUartTxDmaChannel = DMA1_Channel4;
inline DMA_Channel_TypeDef* const tunnelUartRxDmaChannel = DMA1_Channel5;

#define DEBUGUART_TX_DMA_HANDLER DMA1_Channel7_IRQHandler
#define DEBUGUART_HANDLER USART2_IRQHandler
inline const IRQn_Type debugUartTxDmaIRQn = DMA1_Channel7_IRQn;
inline const IRQn_Type debugUartIRQn = USART2_IRQn;
inline const unsigned debugUartTxDmaIrqPrio = 9;
inline const unsigned debugUartIrqPrio = 10;
inline DMA_Channel_TypeDef* const debugUartTxDmaChannel = DMA1_Channel7;
inline DMA_Channel_TypeDef* const debugUartRxDmaChannel = DMA1_Channel6;

#define CONTROLUART_TX_DMA_HANDLER DMA2_Channel4_5_IRQHandler
inline const IRQn_Type controlUartTxDmaIRQn = DMA2_Channel4_5_IRQn;
inline const unsigned controlUartTxDmaIRQnPrio = 8;
inline DMA_Channel_TypeDef* const controlUartTxDmaChannel = DMA2_Channel5;
inline DMA_Channel_TypeDef* const controlUartRxDmaChannel = DMA2_Channel3;

#define IN4PORT GPIOD
#define IN4AMASK GPIO_PIN_1
#define IN4BMASK GPIO_PIN_0
inline const PinDef pwm1bPin = std::make_pair(GPIOE, GPIO_PIN_8);
inline const PinDef pwm1aPin = std::make_pair(GPIOE, GPIO_PIN_9);
inline const PinDef pwm2bPin = std::make_pair(GPIOE, GPIO_PIN_10);
inline const PinDef pwm2aPin = std::make_pair(GPIOE, GPIO_PIN_11);
inline const PinDef pwm3bPin = std::make_pair(GPIOE, GPIO_PIN_12);
inline const PinDef pwm3aPin = std::make_pair(GPIOE, GPIO_PIN_13);
inline const PinDef pwm4Pin = std::make_pair(GPIOE, GPIO_PIN_14);
inline const PinDef in4bPin = std::make_pair(IN4PORT, IN4BMASK);
inline const PinDef in4aPin = std::make_pair(IN4PORT, IN4AMASK);

inline TIM_TypeDef* const pwmTimer = TIM1;

inline const PinDef encoder1aPin = std::make_pair(GPIOA, GPIO_PIN_15);
inline const PinDef encoder1bPin = std::make_pair(GPIOB, GPIO_PIN_3);
inline const PinDef encoder2aPin = std::make_pair(GPIOB, GPIO_PIN_4);
inline const PinDef encoder2bPin = std::make_pair(GPIOB, GPIO_PIN_5);
inline const PinDef encoder3aPin = std::make_pair(GPIOD, GPIO_PIN_12);
inline const PinDef encoder3bPin = std::make_pair(GPIOD, GPIO_PIN_13);
inline const PinDef encoder4aPin = std::make_pair(GPIOC, GPIO_PIN_6);
inline const PinDef encoder4bPin = std::make_pair(GPIOC, GPIO_PIN_7);

inline TIM_TypeDef* const encoderTimer[4] = { TIM2, TIM3, TIM4, TIM8 };

inline const PinDef servo1Pin = std::make_pair(GPIOA, GPIO_PIN_0);
inline const PinDef servo2Pin = std::make_pair(GPIOA, GPIO_PIN_1);
inline const PinDef servo3Pin = std::make_pair(GPIOA, GPIO_PIN_2);
inline const PinDef servo4Pin = std::make_pair(GPIOA, GPIO_PIN_3);
inline const PinDef servoPins = std::make_pair(GPIOA,
    servo1Pin.second | servo2Pin.second | servo3Pin.second | servo4Pin.second);

inline TIM_TypeDef* const servoTimer = TIM5;

inline const PinDef espEnPin = std::make_pair(GPIOC, GPIO_PIN_8);
inline const PinDef esp0Pin = std::make_pair(GPIOC, GPIO_PIN_10);
inline const PinDef esp2Pin = std::make_pair(GPIOC, GPIO_PIN_11);
inline const PinDef esp12Pin = std::make_pair(GPIOB, GPIO_PIN_15);
inline const PinDef esp15Pin = std::make_pair(GPIOB, GPIO_PIN_14);

inline const PinDef buzzerPin = std::make_pair(GPIOD, GPIO_PIN_7);

inline const ADC_channel_t batteryVoltageAdcChannel = LL_ADC_CHANNEL_6;
inline const ADC_channel_t batteryMiddleVoltageAdcChannel = LL_ADC_CHANNEL_7;

inline const ADC_rank_t batteryVoltageAdcRank = LL_ADC_INJ_RANK_1;
inline const ADC_rank_t internalReferenceVoltageAdcRank = LL_ADC_INJ_RANK_2;
inline const ADC_rank_t batteryMiddleVoltageAdcRank = LL_ADC_INJ_RANK_3;
inline const ADC_rank_t temperatureAdcRank = LL_ADC_INJ_RANK_4;

inline const PinDef batteryVoltagePin = std::make_pair(GPIOA, GPIO_PIN_6);
inline const PinDef batteryMiddleVoltagePin = std::make_pair(GPIOA, GPIO_PIN_7);

inline ADC_TypeDef* const auxiliaryAdc = ADC1;
inline const IRQn_Type auxiliaryAndMotorAdcIRQn = ADC1_2_IRQn;
inline const unsigned auxiliaryAndMotorAdcIrqPrio = 9;
#define AUXILIARY_AND_MOTOR_ADC_IRQ_HANDLER ADC1_2_IRQHandler

inline void clocksInit() {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        abort();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        abort();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    PeriphClkInit.AdcClockSelection = RCC_CFGR_ADCPRE_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        abort();
    }

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
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
}

// Set-up ESP32 strapping pins for the normal mode functions. Esp32Manager
// handles the strapping process and ESP32 reset, and calls this function
// after the reset is done so that normal function can be restored.
inline void reinitEspStrappingPins() {
    // ControlUart uses esp0Pin and esp2Pin
    pinInit(
        controlUartTxPin, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    pinInit(controlUartRxPin, GPIO_MODE_AF_INPUT, GPIO_PULLUP,
        GPIO_SPEED_FREQ_HIGH);

    // Other pins are unused during normal function, set them to default state
    pinInit(esp12Pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(esp15Pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
}

inline void pinsInit() {
    pinInit(ledPins, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    pinWrite(powerPin, 1);
    pinInit(powerPin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    for (auto button : buttonPin)
        pinInit(button, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);

    pinInit(pwm1bPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm1aPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm2bPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm2aPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm3bPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm3aPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(pwm4Pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(in4bPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(in4aPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    pinInit(encoder1aPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder1bPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder2aPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder2bPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder3aPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder3bPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder4aPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder4bPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);

    // I2C
    pinInit(i2cScl, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(i2cSda, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);

    // USB
    pinInit(usbDnPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(usbDpPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(
        usbDpPullUpPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(
        usbBusDetectionPin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    //LL_GPIO_AF_EnableRemap_TIM1();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM1_REMAP, AFIO_MAPR_TIM1_REMAP_FULLREMAP);
    //LL_GPIO_AF_RemapPartial1_TIM2();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM2_REMAP, AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1);
    //LL_GPIO_AF_RemapPartial_TIM3();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM3_REMAP, AFIO_MAPR_TIM3_REMAP_PARTIALREMAP);
    //LL_GPIO_AF_EnableRemap_TIM4();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM4_REMAP, AFIO_MAPR_TIM4_REMAP);
    //LL_GPIO_AF_Remap_SWJ_NOJTAG();
    LL_GPIO_AF_Remap(AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
    //I2C1-2
    LL_GPIO_AF_Remap(AFIO_MAPR_I2C1_REMAP, AFIO_MAPR_I2C1_REMAP);

    pinInit(servoPins, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM);

    for (const auto& p : utsTrigPin) {
        pinWrite(p, 0);
        pinInit(p, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM);
    }
    for (const auto& p : utsEchoPin) {
        pinInit(p, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN,
            GPIO_SPEED_FREQ_MEDIUM);
    }

    //LL_GPIO_AF_EnableRemap_USART2();
    LL_GPIO_AF_Remap(AFIO_MAPR_USART2_REMAP, AFIO_MAPR_USART2_REMAP);

    // Set UART5 pins to high impedance so we can override them from pinheads
    pinInit(uart5TxPin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(uart5RxPin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 7, 0); // Ultrasound, ESP_EN
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 7, 0); // Ultrasound
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI3_IRQn, 7, 0); // Ultrasound
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(i2cEvIRQn, i2cIRQnPrio, 0);
    HAL_NVIC_SetPriority(i2cErIRQn, i2cIRQnPrio, 0);
    HAL_NVIC_EnableIRQ(i2cEvIRQn);
    HAL_NVIC_EnableIRQ(i2cErIRQn);

    pinInit(buzzerPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    reinitEspStrappingPins();
}
