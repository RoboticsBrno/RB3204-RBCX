#pragma once
/// Board Support Package maps application onto concrete pins/peripherals

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_hal_cortex.h"

#include <algorithm>
#include <array>
#include <cassert>

using PinDef = std::pair<GPIO_TypeDef*, uint16_t>;

inline const PinDef powerPin = std::make_pair(GPIOD, GPIO_PIN_9);

inline const PinDef espEnPin = std::make_pair(GPIOC, GPIO_PIN_8);

inline const PinDef buzzerPin = std::make_pair(GPIOD, GPIO_PIN_7);

inline const PinDef led1Pin = std::make_pair(GPIOD, GPIO_PIN_10);
inline const PinDef led2Pin = std::make_pair(GPIOD, GPIO_PIN_11);
inline const PinDef led3Pin = std::make_pair(GPIOD, GPIO_PIN_14);
inline const PinDef led4Pin = std::make_pair(GPIOD, GPIO_PIN_15);
inline const PinDef ledPins = std::make_pair(GPIOD, led1Pin.second
                                                  | led2Pin.second
                                                  | led3Pin.second
                                                  | led4Pin.second );
inline const std::array<PinDef, 5> ledPin = {
    ledPins,
    led1Pin,
    led2Pin,
    led3Pin,
    led4Pin
};

inline const PinDef buttonOffPin = std::make_pair(GPIOE, GPIO_PIN_15);
inline const PinDef button1Pin   = std::make_pair(GPIOB, GPIO_PIN_6);
inline const PinDef button2Pin   = std::make_pair(GPIOD, GPIO_PIN_3);
inline const PinDef button3Pin   = std::make_pair(GPIOB, GPIO_PIN_2);
inline const PinDef button4Pin   = std::make_pair(GPIOD, GPIO_PIN_4);
inline const PinDef buttonOnPin  = std::make_pair(GPIOC, GPIO_PIN_9);
inline const std::array<PinDef, 6> buttonPin = {
    buttonOffPin,
    button1Pin,
    button2Pin,
    button3Pin,
    button4Pin,
    buttonOnPin
};

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

#define MOTORS_COUNT 4

#define IN4PORT  GPIOD
#define IN4AMASK GPIO_PIN_1
#define IN4BMASK GPIO_PIN_0
inline const PinDef pwm1bPin = std::make_pair(GPIOE, GPIO_PIN_8);
inline const PinDef pwm1aPin = std::make_pair(GPIOE, GPIO_PIN_9);
inline const PinDef pwm2bPin = std::make_pair(GPIOE, GPIO_PIN_10);
inline const PinDef pwm2aPin = std::make_pair(GPIOE, GPIO_PIN_11);
inline const PinDef pwm3bPin = std::make_pair(GPIOE, GPIO_PIN_12);
inline const PinDef pwm3aPin = std::make_pair(GPIOE, GPIO_PIN_13);
inline const PinDef pwm4Pin  = std::make_pair(GPIOE, GPIO_PIN_14);
inline const PinDef in4bPin  = std::make_pair(IN4PORT, IN4BMASK);
inline const PinDef in4aPin  = std::make_pair(IN4PORT, IN4AMASK);

inline TIM_TypeDef * const pwmTimer = TIM1;

inline const PinDef encoder1aPin = std::make_pair(GPIOA, GPIO_PIN_15);
inline const PinDef encoder1bPin = std::make_pair(GPIOB, GPIO_PIN_3);
inline const PinDef encoder2aPin = std::make_pair(GPIOB, GPIO_PIN_4);
inline const PinDef encoder2bPin = std::make_pair(GPIOB, GPIO_PIN_5);
inline const PinDef encoder3aPin = std::make_pair(GPIOD, GPIO_PIN_12);
inline const PinDef encoder3bPin = std::make_pair(GPIOD, GPIO_PIN_13);
inline const PinDef encoder4aPin = std::make_pair(GPIOC, GPIO_PIN_6);
inline const PinDef encoder4bPin = std::make_pair(GPIOC, GPIO_PIN_7);

typedef std::pair<TIM_TypeDef * const, IRQn_Type> encoderTimerTableEntry_t;
inline const std::array<encoderTimerTableEntry_t, MOTORS_COUNT> encoderTimer = {
    std::make_pair( TIM2, TIM2_IRQn ),
    std::make_pair( TIM3, TIM3_IRQn ),
    std::make_pair( TIM4, TIM4_IRQn ),
    std::make_pair( TIM8, TIM8_UP_IRQn )
};
#define ENCODER1_IRQ_HANDLER  TIM2_IRQHandler
#define ENCODER2_IRQ_HANDLER  TIM3_IRQHandler
#define ENCODER3_IRQ_HANDLER  TIM4_IRQHandler
#define ENCODER4_IRQ_HANDLER1 TIM8_UP_IRQHandler
#define ENCODER4_IRQ_HANDLER2 TIM8_TRIG_COM_IRQHandler

#define SERVOS_COUNT 4

inline const PinDef servo1Pin = std::make_pair(GPIOA, GPIO_PIN_0);
inline const PinDef servo2Pin = std::make_pair(GPIOA, GPIO_PIN_1);
inline const PinDef servo3Pin = std::make_pair(GPIOA, GPIO_PIN_2);
inline const PinDef servo4Pin = std::make_pair(GPIOA, GPIO_PIN_3);
inline const PinDef servoPins = std::make_pair(GPIOA, servo1Pin.second
                                                    | servo2Pin.second
                                                    | servo3Pin.second
                                                    | servo4Pin.second );

inline TIM_TypeDef * const servoTimer = TIM5;

typedef uint32_t DMA_channel_t;

typedef uint32_t ADC_channel_t;
typedef uint32_t ADC_rank_t;

inline const ADC_channel_t batteryVoltageAdcChannel       = LL_ADC_CHANNEL_6;
inline const ADC_channel_t batteryMiddleVoltageAdcChannel = LL_ADC_CHANNEL_7;
inline const ADC_channel_t aio1AdcChannel                 = LL_ADC_CHANNEL_4;
inline const ADC_channel_t aio2AdcChannel                 = LL_ADC_CHANNEL_5;

inline const ADC_rank_t batteryVoltageAdcRank             = LL_ADC_INJ_RANK_1;
inline const ADC_rank_t internalReferenceVoltageAdcRank   = LL_ADC_INJ_RANK_2;
inline const ADC_rank_t batteryMiddleVoltageAdcRank       = LL_ADC_INJ_RANK_3;
inline const ADC_rank_t temperatureAdcRank                = LL_ADC_INJ_RANK_4;
inline const ADC_rank_t aio1AdcRank                       = LL_ADC_REG_RANK_1;
inline const ADC_rank_t aio2AdcRank                       = LL_ADC_REG_RANK_2;


inline const PinDef batteryVoltagePin       = std::make_pair(GPIOA, GPIO_PIN_6);
inline const PinDef batteryMiddleVoltagePin = std::make_pair(GPIOA, GPIO_PIN_7);
inline const PinDef aio1Pin                 = std::make_pair(GPIOA, GPIO_PIN_4);
inline const PinDef aio2Pin                 = std::make_pair(GPIOA, GPIO_PIN_5);

inline ADC_TypeDef * const auxiliaryAdc = ADC1;
inline const IRQn_Type     auxiliaryAndMotorAdcIRQn = ADC1_2_IRQn;
#define AUXILIARY_AND_MOTOR_ADC_IRQ_HANDLER ADC1_2_IRQHandler
inline DMA_TypeDef * const auxiliaryAdcDma = DMA1;
inline const DMA_channel_t auxiliaryAdcDmaChannel = LL_DMA_CHANNEL_1;
inline const IRQn_Type     auxiliaryAdcDmaIRQn = DMA1_Channel1_IRQn;
#define AUXILIARY_ADC_DMA_IRQ_HANDLER DMA1_Channel1_IRQHandler

inline const std::array<ADC_channel_t, 4> motorCurrentAdcChannel = {
    LL_ADC_CHANNEL_14,
    LL_ADC_CHANNEL_15,
    LL_ADC_CHANNEL_8,
    LL_ADC_CHANNEL_9
};

inline const PinDef motor1CurrentPin = std::make_pair(GPIOC, GPIO_PIN_4);
inline const PinDef motor2CurrentPin = std::make_pair(GPIOC, GPIO_PIN_5);
inline const PinDef motor3CurrentPin = std::make_pair(GPIOB, GPIO_PIN_0);
inline const PinDef motor4CurrentPin = std::make_pair(GPIOB, GPIO_PIN_1);

inline ADC_TypeDef * const motorCurrentAdc = ADC2;
// IRQ shared with auxiliary ADC

inline const std::array<ADC_channel_t, SERVOS_COUNT> servoCurrentAdcChannel = {
    LL_ADC_CHANNEL_10,
    LL_ADC_CHANNEL_11,
    LL_ADC_CHANNEL_12,
    LL_ADC_CHANNEL_13
};

inline const PinDef servo1CurrentPin = std::make_pair(GPIOC, GPIO_PIN_0);
inline const PinDef servo2CurrentPin = std::make_pair(GPIOC, GPIO_PIN_1);
inline const PinDef servo3CurrentPin = std::make_pair(GPIOC, GPIO_PIN_2);
inline const PinDef servo4CurrentPin = std::make_pair(GPIOC, GPIO_PIN_3);

inline ADC_TypeDef * const servoCurrentAdc = ADC3;
inline const IRQn_Type     servoCurrentAdcIRQn = ADC3_IRQn;
#define SERVO_CURRENT_ADC_IRQ_HANDLER ADC3_IRQHandler

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
    PeriphClkInit.AdcClockSelection = RCC_CFGR_ADCPRE_DIV6;
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
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
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

// This is because AFIO_MAPR register bits SWJ_CFG are write-only, so classical approach
//   read-modifi-write does not work.
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

    pinWrite(espEnPin, 1);
    pinInit(espEnPin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

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
    //LL_GPIO_AF_EnableRemap_TIM1();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM1_REMAP, AFIO_MAPR_TIM1_REMAP_FULLREMAP);

    pinInit(encoder1aPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder1bPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder2aPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder2bPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder3aPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder3bPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder4aPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    pinInit(encoder4bPin, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_MEDIUM);
    //LL_GPIO_AF_RemapPartial1_TIM2();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM2_REMAP, AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1);
    //LL_GPIO_AF_RemapPartial_TIM3();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM3_REMAP, AFIO_MAPR_TIM3_REMAP_PARTIALREMAP);
    //LL_GPIO_AF_EnableRemap_TIM4();
    LL_GPIO_AF_Remap(AFIO_MAPR_TIM4_REMAP, AFIO_MAPR_TIM4_REMAP);
    //LL_GPIO_AF_Remap_SWJ_NOJTAG();
    LL_GPIO_AF_Remap(AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

    pinInit(servoPins, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM);

    pinInit(motor1CurrentPin, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(motor2CurrentPin, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(motor3CurrentPin, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(motor4CurrentPin, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    pinInit(batteryVoltagePin      , GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(batteryMiddleVoltagePin, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(aio1Pin                , GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(aio2Pin                , GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    pinInit(servo1CurrentPin, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(servo2CurrentPin, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(servo3CurrentPin, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(servo4CurrentPin, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    //LL_GPIO_AF_EnableRemap_USART2();
    LL_GPIO_AF_Remap(AFIO_MAPR_USART2_REMAP, AFIO_MAPR_USART2_REMAP);
    

    // USB
    pinInit(usbDnPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(usbDpPin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    pinInit(usbDpPullUpPin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    pinInit(usbBusDetectionPin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
}

inline bool isPressed(PinDef button) {
    return button == buttonOnPin ? pinRead(button) : !pinRead(button);
}