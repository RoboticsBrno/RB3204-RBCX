#pragma once
/// Board Support Package maps application onto concrete pins/peripherals

#include "FreeRTOSConfig.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_ll_utils.h"
#include <algorithm>
#include <array>
#include <cassert>

using PinDef = std::pair<GPIO_TypeDef*, uint16_t>;

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
inline const PinDef utsTrigPins = std::make_pair(GPIOE,
    uts1TrigPin.second | uts2TrigPin.second | uts3TrigPin.second
        | uts4TrigPin.second);
inline const PinDef utsEchoPins = std::make_pair(GPIOE,
    uts1EchoPin.second | uts2EchoPin.second | uts3EchoPin.second
        | uts4EchoPin.second);

inline const std::array<PinDef, 4> utsTrigPin
    = { uts1TrigPin, uts2TrigPin, uts3TrigPin, uts4TrigPin };
inline const std::array<PinDef, 4> utsEchoPin
    = { uts1EchoPin, uts2EchoPin, uts3EchoPin, uts4EchoPin };

#define UTSTIMER_HANDLER TIM7_IRQHandler
#define UTS1_ECHO_HANDLER EXTI1_IRQHandler
#define UTS2_ECHO_HANDLER EXTI3_IRQHandler
#define UTS3_ECHO_HANDLER EXTI9_5_IRQHandler
#define UTS4_ECHO_HANDLER EXTI9_5_IRQHandler
inline const uint32_t utsIRQPrio = 7;
inline const IRQn_Type utsTimerIRQn = TIM7_IRQn;
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

inline USART_TypeDef* const tunnelUart = USART1;
inline USART_TypeDef* const debugUart = USART2;
inline USART_TypeDef* const servoUart = USART3;
inline USART_TypeDef* const controlUart = UART4;

inline DMA_Channel_TypeDef* const tunnelUartTxDmaChannel = DMA1_Channel4;
inline DMA_Channel_TypeDef* const tunnelUartRxDmaChannel = DMA1_Channel5;

#define DEBUGUART_TX_DMA_HANDLER DMA1_Channel7_IRQHandler
inline const IRQn_Type debugUartTxDmaIRQn = DMA1_Channel7_IRQn;
inline const unsigned debugUartTxDmaIrqPrio = 4;
inline DMA_Channel_TypeDef* const debugUartTxDmaChannel = DMA1_Channel7;
inline DMA_Channel_TypeDef* const debugUartRxDmaChannel = DMA1_Channel6;

#define CONTROLUART_TX_DMA_HANDLER DMA2_Channel4_5_IRQHandler
inline const IRQn_Type controlUartTxDmaIRQn = DMA2_Channel4_5_IRQn;
inline const unsigned controlUartTxDmaIRQnPrio = 8;
inline DMA_Channel_TypeDef* const controlUartTxDmaChannel = DMA2_Channel5;
inline DMA_Channel_TypeDef* const controlUartRxDmaChannel = DMA2_Channel3;

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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
}

inline void pinInit(GPIO_TypeDef* port, uint32_t pinMask, uint32_t mode,
    uint32_t pull, uint32_t speed) {

    // HAL_GPIO_Init leaves some flags set if called multiple times
    // on the same pin
    HAL_GPIO_DeInit(port, pinMask);

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
    HAL_GPIO_WritePin(
        pin.first, pin.second, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

inline void pinToggle(PinDef pin) { HAL_GPIO_TogglePin(pin.first, pin.second); }

// This is because AFIO_MAPR register bits SWJ_CFG are write-only, so classical approach
//   read-modify-write does not work.
//   DO NOT USE LL_GPIO_AF_*Remap* FUNCTIONS!
#define AFIO_MAPR_RESERVED 0xF8E00000
inline void LL_GPIO_AF_Remap(uint32_t mask, uint32_t value) {
    static uint32_t mapr = 0;
    mask |= AFIO_MAPR_RESERVED;
    mapr = (mapr & ~mask) | (value & ~AFIO_MAPR_RESERVED);
    AFIO->MAPR = mapr;
}

inline void pinsInit() {
    pinInit(ledPins, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    pinWrite(powerPin, 1);
    pinInit(powerPin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    for (auto button : buttonPin)
        pinInit(button, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);

    // USB
    pinInit(usbDnPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(usbDpPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(
        usbDpPullUpPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(
        usbBusDetectionPin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    //LL_GPIO_AF_RemapPartial1_TIM2();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM2_REMAP, AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1);
    //LL_GPIO_AF_RemapPartial_TIM3();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM3_REMAP, AFIO_MAPR_TIM3_REMAP_PARTIALREMAP);
    //LL_GPIO_AF_EnableRemap_TIM4();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM4_REMAP, AFIO_MAPR_TIM4_REMAP);
    //LL_GPIO_AF_Remap_SWJ_NOJTAG();
    LL_GPIO_AF_Remap(AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

    pinInit(servoPins, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM);

    pinWrite(utsTrigPins, 0);
    pinInit(
        utsTrigPins, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(utsEchoPins, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN,
        GPIO_SPEED_FREQ_MEDIUM);

    //LL_GPIO_AF_EnableRemap_USART2();
    LL_GPIO_AF_Remap(AFIO_MAPR_USART2_REMAP, AFIO_MAPR_USART2_REMAP);

    // Set UART5 pins to high impedance so we can override them from pinheads
    pinInit(uart5TxPin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(uart5RxPin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    // Configure IRQ on espEnPin pin
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

inline bool isPressed(PinDef button) {
    return button == buttonOnPin ? pinRead(button) : !pinRead(button);
}

inline uint32_t getButtons() {
    uint32_t buttons = 0;
    for (size_t i = 0; i < buttonPin.size(); ++i) {
        buttons |= isPressed(buttonPin[i]) << i;
    }
    return buttons;
}

inline void setLeds(uint32_t ledsOn) {
    for (size_t i = 0; i < ledPin.size(); ++i) {
        pinWrite(ledPin[i], ledsOn & (1 << i));
    }
}
