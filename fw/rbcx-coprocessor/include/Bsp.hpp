#pragma once

#include "FreeRTOSConfig.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_cortex.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_utils.h"

#include <algorithm>

using PinDef = std::pair<GPIO_TypeDef*, uint16_t>;

typedef uint32_t DMA_channel_t;
typedef uint32_t ADC_channel_t;
typedef uint32_t ADC_rank_t;

#define USB_LP_IRQ_HANDLER USB_LP_CAN1_RX0_IRQHandler
inline const IRQn_Type usbLpIRQn = USB_LP_CAN1_RX0_IRQn;
inline constexpr unsigned usbLpIRQnPrio = 8;

inline const int mainTaskPrio = 4;
inline const int motorTaskPrio = 2;
inline const int ultrasoundTaskPrio = 2;
inline const int i2cPrio = 2;
inline const int softResetTaskPrio = configMAX_PRIORITIES - 1;

inline void pinInit(GPIO_TypeDef* port, uint32_t pinMask, uint32_t mode,
    uint32_t pull, uint32_t speed, bool deInitFirst = false) {

    // HAL_GPIO_Init leaves some flags set if called multiple times
    // on the same pin
    if (deInitFirst)
        HAL_GPIO_DeInit(port, pinMask);

    GPIO_InitTypeDef init;
    init.Pin = pinMask;
    init.Mode = mode;
    init.Pull = pull;
    init.Speed = speed;
    HAL_GPIO_Init(port, &init);
}

inline void pinInit(PinDef pin, uint32_t mode, uint32_t pull, uint32_t speed,
    bool deInitFirst = false) {
    pinInit(pin.first, pin.second, mode, pull, speed, deInitFirst);
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

inline void LL_ADC_SetChannelSamplingTimeFix(
    ADC_TypeDef* ADCx, uint32_t Channel, uint32_t SamplingTime) {
    volatile uint32_t* const reg
        = (Channel & ADC_SMPR2_REGOFFSET) ? &ADCx->SMPR2 : &ADCx->SMPR1;
    const uint8_t offset = (Channel & ADC_CHANNEL_SMPx_BITOFFSET_MASK)
        >> ADC_CHANNEL_SMPx_BITOFFSET_POS;
    *reg = (*reg & ~(ADC_SMPR2_SMP0 << offset))
        | (SamplingTime & ADC_SMPR2_SMP0) << offset;
}

#if RBCX_HW_VER == 0x0100
#include "Bsp_v10.hpp"
#elif RBCX_HW_VER == 0x0101
#include "Bsp_v11.hpp"
#else
#error "Invalid RBCX_HW_VER"
#endif

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

void softResetInit();
void softReset(); // Resets all peripherials to initial state, eg. when ESP32 restarts.
