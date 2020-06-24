#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_rtc.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_hal_adc.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_hal_cortex.h"

#include <array>
#include "Bsp.hpp"
#include "CdcUartTunnel.hpp"
#include "UsbCdcLink.h"
#include "ControlUart.hpp"

#include "build_time.hpp"

#include <cstring>
#include <ctime>
#include <cstdio>
#include <cmath>

// from fw-putc-replace branch
extern "C" int _write(int file, const char* data, size_t len) {
    // We should check that file is equal to stdin
    tunnelUartTx(reinterpret_cast<const uint8_t*>(data), len);
    HAL_Delay(1 + len / 11);
    return len;
}

static constexpr uint16_t maxPwm = 2000;

void setPwmValue(TIM_TypeDef * timer, uint8_t channel, uint16_t value) {
    reinterpret_cast<__IO uint16_t*>(&timer->CCR1)[channel<<1] = value; // output channel comparsion value
}

void setMotorPower(uint8_t motorID, float power, bool brake = false) {
    assert(motorID < MOTORS_COUNT);
    if (power > 1 || power < -1) {
        //printf("Warning: motor[%u] power out of range <-1; 1> (%9.4f).\n", uint32_t(motorID), power);
        printf("Warning: motor[%d] power out of range <-1; 1> (%dE-3).\n", int(motorID), int(power*1000));
        power = copysignf(1, power);
    }
    uint16_t pwm = fabsf(power) * maxPwm;
    setPwmValue(pwmTimer, motorID, pwm);
    if (pwm == 0 || brake) {
        switch (motorID) {
        case 3:
            IN4PORT->BRR = IN4AMASK | IN4BMASK; // set LOW on IN4A and IN4B
            pwmTimer->CCER |= TIM_CCER_CC4P; // invert channel 4
            break;
        default:
            // set PWM on both channels and invert positive channel
            pwmTimer->CCER = (pwmTimer->CCER & ~(TIM_CCER_CC1NP  << (motorID<<2)))
                | (TIM_CCER_CC1E  << (motorID<<2))
                | (TIM_CCER_CC1NE << (motorID<<2))
                | (TIM_CCER_CC1P  << (motorID<<2));
            break;
        }
    } else {
        switch (motorID) {
        case 3:
            IN4PORT->BSRR = power < 0 ? IN4AMASK | (IN4BMASK<<16)   // pinWrite(in4aPin, power < 0);
                                      : IN4BMASK | (IN4AMASK<<16);  // pinWrite(in4bPin, power > 0);
            pwmTimer->CCER &= ~TIM_CCER_CC4P; // make channel 4 non-inverted
            break;
        default:
            if (power > 0) {
                // set HIGH on positive channel and inverted PWM on negative channel
                pwmTimer->CCER = 
                    (pwmTimer->CCER & ~((TIM_CCER_CC1P  << (motorID<<2))
                                      | (TIM_CCER_CC1NE << (motorID<<2))))
                    | (TIM_CCER_CC1NP << (motorID<<2))
                    | (TIM_CCER_CC1E  << (motorID<<2));
            } else {
                // set HIGH on negative channel and inverted PWM on positive channel
                pwmTimer->CCER = 
                    (pwmTimer->CCER & ~((TIM_CCER_CC1E  << (motorID<<2))
                                      | (TIM_CCER_CC1NP << (motorID<<2))))
                    | (TIM_CCER_CC1NE << (motorID<<2))
                    | (TIM_CCER_CC1P  << (motorID<<2));
            }
            break;
        }
    }
}

#define SYSTICK_LENGTH 24
uint32_t _sysTickToUpConting(uint32_t v) {
    return (SysTick->LOAD & SysTick_LOAD_RELOAD_Msk) - (SysTick->VAL & SysTick_VAL_CURRENT_Msk);
}
uint64_t _sysTickCombineHLcounters(uint32_t h, uint32_t l) {
    return (uint64_t(h) << SYSTICK_LENGTH) | l;
}
static uint32_t _sysTickHigh = 0;
uint64_t getSysTickTime() {
    uint32_t th1 = _sysTickHigh;
    uint32_t tl1 = SysTick->VAL;
    uint32_t th2 = _sysTickHigh;
    uint32_t tl2 = SysTick->VAL;
    tl1 = _sysTickToUpConting(tl1);
    tl2 = _sysTickToUpConting(tl2);
    if (tl2 < tl1)
        return _sysTickCombineHLcounters(th1, tl1);
    return _sysTickCombineHLcounters(th2, tl2);
}
void systick_delay(uint64_t t) {
    t += getSysTickTime();
    while (t < getSysTickTime()) {}
}
//if (SysTick->CTRL & SysTick_CTRL_CLKSOURCE_Msk) // [http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0552a/Bhccjgga.html]
//    _adc_prescaler /= 8;

#define LL_ADC_INJ_RANK(x) (LL_ADC_INJ_RANK_1 + x - 1)
#define ADC_POWER_ON_STABILIZATION_TIME 2
void adcWait(uint32_t clockCycles) {
    if (clockCycles == 0)
        return;
    uint32_t adcPrescaler = 1;
    const uint32_t APB2prescaler = LL_RCC_GetAPB2Prescaler();
    if (APB2prescaler != 0)
        adcPrescaler <<= ((APB2prescaler - RCC_CFGR_PPRE2_DIV2) >> RCC_CFGR_PPRE2_Pos) + 1;
    adcPrescaler *= 2 * ((LL_RCC_GetADCClockSource(RCC_CFGR_ADCPRE) >> RCC_CFGR_ADCPRE_Pos) + 1);
    clockCycles *= adcPrescaler;
    while(--clockCycles != 0) {}
}
void LL_ADC_SetChannelSamplingTimeFix(ADC_TypeDef *ADCx, uint32_t Channel, uint32_t SamplingTime) {
    volatile uint32_t* const reg = (Channel & ADC_SMPR2_REGOFFSET) ? &ADCx->SMPR2 : &ADCx->SMPR1;
    const uint8_t offset = (Channel & ADC_CHANNEL_SMPx_BITOFFSET_MASK) >> ADC_CHANNEL_SMPx_BITOFFSET_POS;
    *reg = (*reg & ~(ADC_SMPR2_SMP0 << offset))
         | (SamplingTime & ADC_SMPR2_SMP0) << offset;
}

volatile int32_t encoder[MOTORS_COUNT] = { 0 };
volatile uint16_t rawAio[2] = { 0 };

int main() {
    clocksInit();
    HAL_Init();
    pinsInit();
    tunnelUartInit();
    controlUartInit();
    cdcLinkInit();

    //HAL_Delay(1000);
    printf("RBCX v1.0 HW test\n\t" __DATE__ " " __TIME__ " (%lu)\n", UNIX_TIMESTAMP);
    
    // RTC
    HAL_PWR_EnableBkUpAccess();
    uint32_t isRtcValid = LL_RTC_BKP_GetRegister(BKP, LL_RTC_BKP_DR1);
    printf("isRtcValid %lu\n", isRtcValid);
    if (isRtcValid == 0) {
        while (!LL_RTC_IsActiveFlag_RTOF(RTC));
        LL_RTC_DisableWriteProtection(RTC);
        LL_RCC_LSE_Enable();
        uint32_t timeout = HAL_GetTick();
        while (!LL_RCC_LSE_IsReady()) {
            if ((HAL_GetTick() - timeout) > 1000) {
                printf("Could not initialize 32.768 kHz Quartz.\n");
                break;
            }
        }
        LL_RTC_TIME_Set(RTC, UNIX_TIMESTAMP);
        LL_RTC_SetAsynchPrescaler(RTC, 32767);
        LL_RTC_EnableWriteProtection(RTC);
        timeout = HAL_GetTick();
        while (!LL_RTC_IsActiveFlag_RTOF(RTC)) {
            if ((HAL_GetTick() - timeout) > 100) { // decreased timeout, because some problem on Renatas board and no time to solve it
                printf("RTC synchronization timeout.\n");
                break;
            }
        }
        LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
        LL_RCC_EnableRTC();
        isRtcValid = 1;
        LL_RTC_BKP_SetRegister(BKP, LL_RTC_BKP_DR1, isRtcValid);
        printf("RTC updated by current compilation time %lu.\n", UNIX_TIMESTAMP);
    }
    LL_RTC_WaitForSynchro(RTC);
    printf("RTC time is %lu\n", LL_RTC_TIME_Get(RTC));

    // ADCs
    ADC_TypeDef * const adcs[3] = {
        auxiliaryAdc,
        motorCurrentAdc,
        servoCurrentAdc };
    for (auto adc: adcs)
        LL_ADC_Enable(adc);
    adcWait(ADC_POWER_ON_STABILIZATION_TIME);
    for (auto adc: adcs)
        LL_ADC_StartCalibration(adc);
    for (auto adc: adcs)
        while (LL_ADC_IsCalibrationOnGoing(adc)) {}
    // auxiliaryADC
    LL_DMA_InitTypeDef adc1DmaInit = {
        .PeriphOrM2MSrcAddress = reinterpret_cast<uintptr_t>(&auxiliaryAdc->DR),
        .MemoryOrM2MDstAddress = reinterpret_cast<uintptr_t>(rawAio),
        .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
        .Mode = LL_DMA_MODE_CIRCULAR,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD,
        .NbData = 2,
        .Priority = LL_DMA_PRIORITY_MEDIUM
    };
    LL_DMA_Init(auxiliaryAdcDma, auxiliaryAdcDmaChannel, &adc1DmaInit);
    LL_DMA_EnableChannel(auxiliaryAdcDma, auxiliaryAdcDmaChannel);
    LL_DMA_EnableIT_TC(auxiliaryAdcDma, auxiliaryAdcDmaChannel);
    LL_DMA_EnableIT_TE(auxiliaryAdcDma, auxiliaryAdcDmaChannel);
    LL_ADC_INJ_SetTrigAuto          (auxiliaryAdc, LL_ADC_INJ_TRIG_FROM_GRP_REGULAR);
    LL_ADC_SetSequencersScanMode    (auxiliaryAdc, LL_ADC_SEQ_SCAN_ENABLE);
    LL_ADC_SetChannelSamplingTimeFix(auxiliaryAdc, aio1AdcChannel                 , LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_SetChannelSamplingTimeFix(auxiliaryAdc, aio2AdcChannel                 , LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_SetChannelSamplingTimeFix(auxiliaryAdc, batteryVoltageAdcChannel       , LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_SetChannelSamplingTimeFix(auxiliaryAdc, batteryMiddleVoltageAdcChannel , LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_SetChannelSamplingTimeFix(auxiliaryAdc, LL_ADC_CHANNEL_VREFINT         , LL_ADC_SAMPLINGTIME_1CYCLE_5);
    LL_ADC_SetChannelSamplingTimeFix(auxiliaryAdc, LL_ADC_CHANNEL_TEMPSENSOR      , LL_ADC_SAMPLINGTIME_239CYCLES_5);
    LL_ADC_REG_SetSequencerRanks    (auxiliaryAdc, aio1AdcRank                    , aio1AdcChannel);
    LL_ADC_REG_SetSequencerRanks    (auxiliaryAdc, aio2AdcRank                    , aio2AdcChannel);
    auxiliaryAdc->JSQR = ADC_JSQR_JL
                       | (batteryVoltageAdcChannel       & ADC_JSQR_JSQ1) << (ADC_JSQR_JSQ2_Pos * ((batteryVoltageAdcRank           - 1) & 3))
                       | (LL_ADC_CHANNEL_VREFINT         & ADC_JSQR_JSQ1) << (ADC_JSQR_JSQ2_Pos * ((internalReferenceVoltageAdcRank - 1) & 3))
                       | (batteryMiddleVoltageAdcChannel & ADC_JSQR_JSQ1) << (ADC_JSQR_JSQ2_Pos * ((batteryMiddleVoltageAdcRank     - 1) & 3))
                       | (LL_ADC_CHANNEL_TEMPSENSOR      & ADC_JSQR_JSQ1) << (ADC_JSQR_JSQ2_Pos * ((temperatureAdcRank              - 1) & 3));
    // Next four functions does not work, fixed with previous statement
    //LL_ADC_INJ_SetSequencerRanks (auxiliaryAdc, batteryVoltageAdcRank          , batteryVoltageAdcChannel);
    //LL_ADC_INJ_SetSequencerRanks (auxiliaryAdc, internalReferenceVoltageAdcRank, LL_ADC_CHANNEL_VREFINT);
    //LL_ADC_INJ_SetSequencerRanks (auxiliaryAdc, batteryMiddleVoltageAdcRank    , batteryMiddleVoltageAdcChannel);
    //LL_ADC_INJ_SetSequencerRanks (auxiliaryAdc, temperatureAdcRank             , LL_ADC_CHANNEL_TEMPSENSOR);
    //LL_ADC_INJ_SetSequencerLength(auxiliaryAdc, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS); // included in previous hotfix (its mandatory part)
    LL_ADC_REG_SetSequencerLength(auxiliaryAdc, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);
    auxiliaryAdc->CR2 = ADC_CR2_TSVREFE             // enable temperature sensor and intermal voltage reference
                      | ADC_CR2_DMA                 // enable DMA
                      | LL_ADC_REG_TRIG_SOFTWARE    // software trigger for regular group
                      | LL_ADC_INJ_TRIG_SOFTWARE    // software trigger for injected group
                      | ADC_CR2_CONT                // enable continuous mearurement mode
                      | ADC_CR2_SWSTART             // start the first conversion
                      | ADC_CR2_ADON;               // keep ADC enabled and start the first conversion
    // motorCurrentADC + servoCurrentADC
    LL_ADC_SetSequencersScanMode(motorCurrentAdc, LL_ADC_SEQ_SCAN_ENABLE);
    LL_ADC_SetSequencersScanMode(servoCurrentAdc, LL_ADC_SEQ_SCAN_ENABLE);
    LL_ADC_INJ_SetSequencerDiscont(motorCurrentAdc, LL_ADC_INJ_SEQ_DISCONT_1RANK);
    motorCurrentAdc->JSQR = ADC_JSQR_JL; // first part of LL_ADC_INJ_SetSequencerRanks hotfix
    servoCurrentAdc->JSQR = ADC_JSQR_JL; // first part of LL_ADC_INJ_SetSequencerRanks hotfix
    for (uint8_t i = 0; i != MOTORS_COUNT; ++i) {
        LL_ADC_SetChannelSamplingTimeFix(motorCurrentAdc, motorCurrentAdcChannel[i], LL_ADC_SAMPLINGTIME_1CYCLE_5);
        LL_ADC_SetChannelSamplingTimeFix(servoCurrentAdc, servoCurrentAdcChannel[i], LL_ADC_SAMPLINGTIME_1CYCLE_5);
        motorCurrentAdc->JSQR |= (motorCurrentAdcChannel[i] & ADC_JSQR_JSQ1) << (ADC_JSQR_JSQ2_Pos * i);
        servoCurrentAdc->JSQR |= (servoCurrentAdcChannel[i] & ADC_JSQR_JSQ1) << (ADC_JSQR_JSQ2_Pos * i);
        // Next two functions does not work, fixed with previous statements
        //LL_ADC_INJ_SetSequencerRanks (motorCurrentAdc, LL_ADC_INJ_RANK(i+1), motorCurrentAdcChannel[i]);
        //LL_ADC_INJ_SetSequencerRanks (servoCurrentAdc, LL_ADC_INJ_RANK(i+1), servoCurrentAdcChannel[i]);
    }
    //LL_ADC_INJ_SetSequencerLength(motorCurrentAdc, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS); // included in previous hotfix (its mandatory part)
    //LL_ADC_INJ_SetSequencerLength(servoCurrentAdc, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS); // included in previous hotfix (its mandatory part)
    LL_ADC_EnableIT_JEOS(motorCurrentAdc);
    LL_ADC_EnableIT_JEOS(servoCurrentAdc);
    motorCurrentAdc->CR2 = ADC_CR2_JEXTTRIG                 // enable external trigger for injected group
                         | LL_ADC_INJ_TRIG_EXT_TIM1_TRGO    // set TIM1_TRGO as external trigger for injected group
                         | ADC_CR2_ADON;                    // keep ADC enabled and start the first conversion
    servoCurrentAdc->CR2 = ADC_CR2_JEXTTRIG                 // enable external trigger for injected group
                         | LL_ADC_INJ_TRIG_EXT_TIM5_TRGO    // set TIM1_TRGO as external trigger for injected group
                         | ADC_CR2_ADON;                    // keep ADC enabled and start the first conversion
    // ADCs IRQ
    HAL_NVIC_SetPriority(auxiliaryAndMotorAdcIRQn, 1, 0);
    HAL_NVIC_SetPriority(auxiliaryAdcDmaIRQn, 1, 0);
    HAL_NVIC_SetPriority(servoCurrentAdcIRQn, 2, 0);
    HAL_NVIC_EnableIRQ(auxiliaryAndMotorAdcIRQn);
    HAL_NVIC_EnableIRQ(auxiliaryAdcDmaIRQn);
    HAL_NVIC_EnableIRQ(servoCurrentAdcIRQn);

/*    constexpr uint8_t regs = 15;
    const char* regName[regs] = {
        "SR   ",
        "CR1  ",
        "CR2  ",
        "SMPR1",
        "SMPR2",
        "JOFR1",
        "JOFR2",
        "JOFR3",
        "JOFR4",
        "HTR  ",
        "LTR  ",
        "SQR1 ",
        "SQR2 ",
        "SQR3 ",
        "JSQR "
    };
    for (auto adc: adcs) {
        printf("ADC %08lX\n", reinterpret_cast<uintptr_t>(adc));
        for (uint8_t i = 0; i != regs; ++i) {
            uint32_t v = reinterpret_cast<uint32_t*>(adc)[i];
            printf("\t%s: %08lX\n", regName[i], v);
        }
    }*/

    // PWM
    LL_TIM_InitTypeDef TIM_PWM_Init = {
        .Prescaler = 0,
        .CounterMode = LL_TIM_COUNTERMODE_CENTER_DOWN, // this sets interrupts flag when counter reachs TOP
        .Autoreload = maxPwm,
        .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 1                         // update generated only at undefflow, not both underflow and overflow
    };
    LL_TIM_OC_InitTypeDef TIM_OC_Init = {
        .OCMode = LL_TIM_OCMODE_PWM2,
        .OCState = LL_TIM_OCSTATE_ENABLE,
        .OCNState = LL_TIM_OCSTATE_ENABLE,
        .CompareValue = maxPwm/2,
        .OCPolarity = LL_TIM_OCPOLARITY_HIGH,
        .OCNPolarity = LL_TIM_OCPOLARITY_HIGH,
        .OCIdleState = LL_TIM_OCIDLESTATE_HIGH,
        .OCNIdleState = LL_TIM_OCIDLESTATE_HIGH
    };
    LL_TIM_Init(pwmTimer, &TIM_PWM_Init);
    for (uint16_t channel = LL_TIM_CHANNEL_CH1; channel != 0; channel <<= 4)
    {
        LL_TIM_OC_Init(pwmTimer, channel, &TIM_OC_Init);
        LL_TIM_OC_EnablePreload(pwmTimer, channel);
    }
    LL_TIM_SetOffStates(pwmTimer, LL_TIM_OSSI_DISABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_GenerateEvent_UPDATE(pwmTimer);
    LL_TIM_SetTriggerOutput(pwmTimer, LL_TIM_TRGO_UPDATE);
    LL_TIM_EnableBRK(pwmTimer);
    LL_TIM_EnableAllOutputs(pwmTimer);
    LL_TIM_EnableCounter(pwmTimer);

    // Encoders
    LL_TIM_ENCODER_InitTypeDef TIM_Encoder_Init = {
        .EncoderMode = LL_TIM_ENCODERMODE_X4_TI12,
        .IC1Polarity = LL_TIM_IC_POLARITY_RISING,
        .IC1ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
        .IC1Prescaler = LL_TIM_ICPSC_DIV1,
        .IC1Filter = LL_TIM_IC_FILTER_FDIV1,
        .IC2Polarity = LL_TIM_IC_POLARITY_RISING,
        .IC2ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
        .IC2Prescaler = LL_TIM_ICPSC_DIV1,
        .IC2Filter = LL_TIM_IC_FILTER_FDIV1
    };
    for (encoderTimerTableEntry_t timer_irqn: encoderTimer) {
        auto timer = timer_irqn.first;
        auto irqn  = timer_irqn.second;
        LL_TIM_ENCODER_Init(timer, &TIM_Encoder_Init);
        LL_TIM_SetAutoReload(timer, 0xFFFF);
        LL_TIM_GenerateEvent_UPDATE(timer);
        LL_TIM_ClearFlag_UPDATE(timer);
        LL_TIM_EnableCounter(timer);
        if (irqn == TIM2_IRQn) {
            LL_TIM_SetTriggerInput(timer, LL_TIM_TS_TI1F_ED);
            LL_TIM_EnableIT_TRIG(timer);
            LL_TIM_EnableIT_UPDATE(timer);
            HAL_NVIC_SetPriority(irqn, 1, 0);
            HAL_NVIC_EnableIRQ(irqn);
        }
    }
    //HAL_NVIC_SetPriority(TIM8_TRG_COM_IRQn, 1, 0);
    //HAL_NVIC_EnableIRQ(TIM8_TRG_COM_IRQn);

    // Servos
    LL_TIM_StructInit(&TIM_PWM_Init);
    TIM_PWM_Init.Prescaler = 24/2-1; // should be computed based on PCLK1 frequency
    TIM_PWM_Init.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_PWM_Init.Autoreload = 60000; // should be computed based on PCLK1 frequency
    TIM_PWM_Init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_PWM_Init.RepetitionCounter = 0;
    LL_TIM_OC_StructInit(&TIM_OC_Init);
    TIM_OC_Init.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_Init.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_Init.CompareValue = 9000; // should be computed based on PCLK1 frequency
    TIM_OC_Init.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_Init.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_Init(servoTimer, &TIM_PWM_Init);
    for (uint16_t channel = LL_TIM_CHANNEL_CH1; channel != 0; channel <<= 4)
    {
        LL_TIM_OC_Init(servoTimer, channel, &TIM_OC_Init);
        LL_TIM_OC_EnablePreload(servoTimer, channel);
    }
    LL_TIM_SetOffStates(servoTimer, LL_TIM_OSSI_DISABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_GenerateEvent_UPDATE(servoTimer);
    LL_TIM_SetTriggerOutput(servoTimer, LL_TIM_TRGO_UPDATE);
    LL_TIM_EnableAllOutputs(servoTimer);
    LL_TIM_EnableCounter(servoTimer);

    // PWM preset
    for(uint8_t i = 0; i != MOTORS_COUNT; ++i)
        setMotorPower(i, 0.5);

    // main loop
    uint8_t leds = 0x01;
    const uint32_t ledPeriod = 500;
    uint32_t nextLedTime = ledPeriod;
    bool ledTest = false;
    bool usbConnected = false;
    float power = 0;
    float powerIncrement = 0.1;
    const uint32_t powerPeriod = 1000;
    uint32_t nextPowerTime = 0;
    const uint32_t encoderPeriod = 1000;
    uint32_t nextEncoderTime = 0;
    bool sendNewLine = false;
    uint32_t off_timeout = 0;
    while (true) {
        const uint32_t now = HAL_GetTick();
        if (ledTest && now >= nextLedTime) {
            switch (leds) {
            case 0x01: leds = 0x03; break;
            case 0x30: leds = 0x01; break;
            default: leds <<= 1; break;
            }
            for (uint8_t i = 0; i != ledPin.size()-1; ++i)
                pinWrite(ledPin[i+1], leds & (1<<i));
            nextLedTime += ledPeriod;
        }
        if (isPressed(buttonOffPin)) {
            if (off_timeout == 0) {
                off_timeout = now + 3200;
            } else if (now >= off_timeout) {
                pinWrite(powerPin, 0);
                pinWrite(ledPins, 0);
                ledTest = false;
                if (isPressed(button4Pin) && isPressed(button2Pin)) {
                    LL_RCC_ForceBackupDomainReset();
                    printf("Force BACKUP reset.\n");
                }
                printf("Shutting down...\n");
                for(;;);
            }
        } else {
            off_timeout = 0;
            if (isPressed(buttonOnPin)) {
                pinWrite(powerPin, 1);
                ledTest = true;
            }
        }
        for (uint8_t i = 1; 0 && i != 5; ++i) {
            if (isPressed(buttonPin[i])) {
                ledTest = false;
                pinWrite(ledPin[i], 1);
            } else if (!ledTest) {
                pinWrite(ledPin[i], 0);
            }
        }
        if (LL_TIM_IsActiveFlag_BRK(pwmTimer)) {
            pinWrite(led2Pin, 1);
            LL_TIM_ClearFlag_BRK(pwmTimer);
        } else {
            pinWrite(led2Pin, 0);
        }
        pinWrite(buzzerPin, isPressed(button4Pin) && isPressed(button3Pin) && !isPressed(button1Pin));
        pinWrite(espEnPin, !(isPressed(button1Pin) && isPressed(button2Pin)));
        if (LL_RTC_IsActiveFlag_SEC(RTC)) {
            LL_RTC_ClearFlag_SEC(RTC);
            if (isPressed(button4Pin) && isPressed(button2Pin)) {
                HAL_Delay(1);
                if (sendNewLine)
                    printf("\t");
                printf("Time %lu", LL_RTC_TIME_Get(RTC));
                sendNewLine = true;
            }
        }
        if (0 && now >= nextPowerTime) {
            nextPowerTime += powerPeriod;
            if (sendNewLine)
                printf("\t");
            //printf("Power %9.4f", power);
            printf("Power %5d", int(power * maxPwm));
            sendNewLine = true;
            for(uint8_t i = 0; i != 4; ++i)
                setMotorPower(i, power, isPressed(button1Pin));
            power += powerIncrement;
            if (fabsf(power) > 1) {
                power = copysignf(1, power);
                powerIncrement = -powerIncrement;
            }
        }
        if (1 && now >= nextEncoderTime) {
            nextEncoderTime += encoderPeriod;
            if (sendNewLine)
                printf("\t");
            printf("Encoders");
            for(uint8_t i = 0; i != 4; ++i) {
                printf("%6lu", encoderTimer[i].first->CNT);
            }
            sendNewLine = true;
        }
        if (sendNewLine) {
            sendNewLine = false;
            printf("\n");
        }
        bool v = pinRead(usbBusDetectionPin);
        if (v != usbConnected) {
            usbConnected = v;
            if (usbConnected) {
                printf("USB connected\n");
            } else {
                printf("USB disconnected\n");
            }
        }
        //if ()
        cdcLinkPoll();
        tunnelPoll();
        std::array<uint8_t, 255> loopback;
        auto len = controlUartRxFrame(loopback.data(), loopback.size());
        if (len && controlUartTxReady()) {
            controlUartTxFrame(loopback.data(), len);
        }
    }
}

extern "C" void SysTick_Handler() {
    ++_sysTickHigh;
    HAL_IncTick();
}

extern "C" void AUXILIARY_AND_MOTOR_ADC_IRQ_HANDLER() {
    if (LL_ADC_IsEnabledIT_JEOS(auxiliaryAdc) && LL_ADC_IsActiveFlag_JEOS(auxiliaryAdc)) {
        //pinWrite(led1Pin, 1);
        uint16_t rawBatteryVoltage           = LL_ADC_INJ_ReadConversionData12(auxiliaryAdc, batteryVoltageAdcRank);
        uint16_t rawBatteryMiddleVoltage     = LL_ADC_INJ_ReadConversionData12(auxiliaryAdc, batteryMiddleVoltageAdcRank);
        uint16_t rawInternalReferenceVoltage = LL_ADC_INJ_ReadConversionData12(auxiliaryAdc, internalReferenceVoltageAdcRank);
        uint16_t rawTemperature              = LL_ADC_INJ_ReadConversionData12(auxiliaryAdc, temperatureAdcRank);
        // This is place for battery voltage control, if desired in interrupt
        LL_ADC_ClearFlag_JEOS(auxiliaryAdc);
        //pinWrite(led1Pin, 0);
    }
    if (LL_ADC_IsEnabledIT_JEOS(motorCurrentAdc) && LL_ADC_IsActiveFlag_JEOS(motorCurrentAdc)) {
        //pinWrite(led2Pin, 1);
        for (uint8_t i = 0; i != MOTORS_COUNT; ++i) {
            uint16_t rawCurrent = LL_ADC_INJ_ReadConversionData12(motorCurrentAdc, LL_ADC_INJ_RANK(i+1));
            // This is place for motor current regulator, just enable this interrupt
            // Called every fourth PWM cycle
        }
        LL_ADC_ClearFlag_JEOS(motorCurrentAdc);
        //pinWrite(led2Pin, 0);
    }
    HAL_NVIC_ClearPendingIRQ(auxiliaryAndMotorAdcIRQn);
}

extern "C" void SERVO_CURRENT_ADC_IRQ_HANDLER() {
    if (LL_ADC_IsEnabledIT_JEOS(servoCurrentAdc) && LL_ADC_IsActiveFlag_JEOS(servoCurrentAdc)) {
        //pinWrite(led3Pin, 1);
        for (uint8_t i = 0; i != SERVOS_COUNT; ++i) {
            uint16_t rawCurrent = LL_ADC_INJ_ReadConversionData12(servoCurrentAdc, LL_ADC_INJ_RANK(i+1));
            // This is place for servo current limitation, just enable this interrupt
        }
        LL_ADC_ClearFlag_JEOS(servoCurrentAdc);
        //pinWrite(led3Pin, 0);
    }
    HAL_NVIC_ClearPendingIRQ(servoCurrentAdcIRQn);
}

extern "C" void AUXILIARY_ADC_DMA_IRQ_HANDLER() {
    if (LL_DMA_IsActiveFlag_GI1(auxiliaryAdcDma) || LL_DMA_IsActiveFlag_TC1(auxiliaryAdcDma)) {
        //pinWrite(led4Pin, 1);
        LL_DMA_ClearFlag_TC1(auxiliaryAdcDma);
        LL_DMA_ClearFlag_TE1(auxiliaryAdcDma);
        LL_DMA_ClearFlag_GI1(auxiliaryAdcDma);
        //pinWrite(led4Pin, 0);
    }
    HAL_NVIC_ClearPendingIRQ(auxiliaryAdcDmaIRQn);
}

void encoder_irq_handler(encoderTimerTableEntry_t encoder) {
    auto timer = encoder.first;
    auto irqn  = encoder.second;
    if (LL_TIM_IsEnabledIT_TRIG(timer) && LL_TIM_IsActiveFlag_TRIG(timer)) {
        pinToggle(led4Pin);
        LL_TIM_ClearFlag_TRIG(timer);
    }
    if (LL_TIM_IsEnabledIT_UPDATE(timer) && LL_TIM_IsActiveFlag_UPDATE(timer)) {
        pinToggle(led3Pin);
        LL_TIM_ClearFlag_UPDATE(timer);
    }
    HAL_NVIC_ClearPendingIRQ(irqn);
}

extern "C" void ENCODER1_IRQ_HANDLER() {
    encoder_irq_handler(encoderTimer[0]);
}
extern "C" void ENCODER2_IRQ_HANDLER() {
    encoder_irq_handler(encoderTimer[1]);
}
extern "C" void ENCODER3_IRQ_HANDLER() {
    encoder_irq_handler(encoderTimer[2]);
}
extern "C" void ENCODER4_IRQ_HANDLER1() {
    encoder_irq_handler(encoderTimer[3]);
}
extern "C" void ENCODER4_IRQ_HANDLER2() {
    encoder_irq_handler(encoderTimer[3]);
    HAL_NVIC_ClearPendingIRQ(TIM8_TRG_COM_IRQn);
}
