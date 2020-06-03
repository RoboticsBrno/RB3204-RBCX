#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_rtc.h"
#include "stm32f1xx_ll_tim.h"

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
    assert(motorID < 4);
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

int main() {
    clocksInit();
    HAL_Init();
    pinsInit();
    tunnelUartInit();
    controlUartInit();    
    cdcLinkInit();

    HAL_Delay(1000);
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
                printf("Could not initialize 32.768 kHz Quartz.");
                break;
            }
        }
        LL_RTC_TIME_Set(RTC, UNIX_TIMESTAMP);
        LL_RTC_SetAsynchPrescaler(RTC, 32767);
        LL_RTC_EnableWriteProtection(RTC);
        timeout = HAL_GetTick();
        while (!LL_RTC_IsActiveFlag_RTOF(RTC)) {
            if ((HAL_GetTick() - timeout) > 1000) {
                printf("RTC synchronization timeout.");
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

    // PWM
    LL_TIM_InitTypeDef TIM_PWM_Init = {
        .Prescaler = 0,
        .CounterMode = LL_TIM_COUNTERMODE_CENTER_DOWN, // this sets interrupts flag when counter reachs TOP
        .Autoreload = maxPwm,
        .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0
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
    for (auto timer: encoderTimer) {
        LL_TIM_ENCODER_Init(timer, &TIM_Encoder_Init);
        LL_TIM_EnableCounter(timer);
    }

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
    LL_TIM_EnableAllOutputs(servoTimer);
    LL_TIM_EnableCounter(servoTimer);

    // main loop
    uint8_t leds = 0x01;
    const uint32_t ledPeriod = 500;
    uint32_t nextLedTime = ledPeriod;
    bool ledTest = true;
    bool usbConnected = false;
    float power = 0;
    float powerIncrement = 0.1;
    const uint32_t powerPeriod = 1000;
    uint32_t nextPowerTime = 0;
    const uint32_t encoderPeriod = 1000;
    uint32_t nextEncoderTime = 0;
    bool sendNewLine = false;
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
            pinWrite(powerPin, 0);
            pinWrite(ledPins, 0);
            ledTest = false;
            if (isPressed(button4Pin) && isPressed(button2Pin)) {
                LL_RCC_ForceBackupDomainReset();
                printf("Force BACKUP reset.\n");
            }
            printf("Shutting down...\n");
            for(;;);
        } else if (isPressed(buttonOnPin)) {
            pinWrite(powerPin, 1);
            ledTest = true;
        }
        for (uint8_t i = 1; i != 5; ++i) {
            if (isPressed(buttonPin[i])) {
                ledTest = false;
                pinWrite(ledPin[i], 1);
            } else if (!ledTest) {
                pinWrite(ledPin[i], 0);
            }
        }
        pinWrite(buzzerPin, isPressed(button4Pin) && isPressed(button3Pin) && !isPressed(button1Pin));
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
        if (now >= nextPowerTime) {
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
        if (now >= nextEncoderTime) {
            nextEncoderTime += encoderPeriod;
            if (sendNewLine)
                printf("\t");
            printf("Encoders");
            for(uint8_t i = 0; i != 4; ++i) {
                printf("%6u", encoderTimer[i]->CNT);
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
        /*if (isPressed(button1Pin) && isPressed(button3Pin) && isPressed(button4Pin)) {
            printf("AFIO_MAPR = 0x%08X\n", AFIO->MAPR);
            LL_GPIO_AF_Remap(AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_RESET);
            printf("JTAG enabled\n");
            while(isPressed(button1Pin) || isPressed(button3Pin) || isPressed(button4Pin)) {}
        }*/
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
    HAL_IncTick();
}
