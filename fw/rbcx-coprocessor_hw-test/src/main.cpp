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

void setMotorPower(uint8_t motorID, float power, bool brake = false) {
    assert(motorID < 4);
    if (power > 1 || power < -1) {
        //printf("Warning: motor[%u] power out of range <-1; 1> (%9.4f).\n", uint32_t(motorID), power);

        printf("Warning: motor[%u] power out of range <-1; 1> (%dE-3).\n", uint32_t(motorID), int(power*1000));
        power = copysignf(1, power);
    }
    uint16_t pwm = fabsf(power) * maxPwm;
    LL_TIM_OC_SetCompareCH[motorID](pwmTimer, pwm);
    if (brake || pwm == 0) {
        switch (motorID) {
        case 3:
            pinWrite(in4aPin, 0);
            pinWrite(in4bPin, 0);
            LL_TIM_OC_SetPolarity(pwmTimer, timerIndex2channel[motorID], LL_TIM_OCPOLARITY_LOW);
            break;
        default:
            LL_TIM_OC_SetPolarity(pwmTimer, timerIndex2channel[motorID], LL_TIM_OCPOLARITY_LOW);
            LL_TIM_OC_SetPolarity(pwmTimer, timerIndex2negativeChannel[motorID], LL_TIM_OCPOLARITY_HIGH);
            LL_TIM_CC_EnableChannel(pwmTimer, timerIndex2channel[motorID]
                                            | timerIndex2negativeChannel[motorID]);
            break;
        }
    } else {
        switch (motorID) {
        case 3:
            pinWrite(in4aPin, power < 0);
            pinWrite(in4bPin, power > 0);
            LL_TIM_OC_SetPolarity(pwmTimer, timerIndex2channel[motorID], LL_TIM_OCPOLARITY_HIGH);
            break;
        default:
            if (power > 0) {
                LL_TIM_OC_SetPolarity(pwmTimer, timerIndex2channel[motorID], LL_TIM_OCPOLARITY_HIGH);
                LL_TIM_OC_SetPolarity(pwmTimer, timerIndex2negativeChannel[motorID], LL_TIM_OCPOLARITY_LOW);
                LL_TIM_CC_EnableChannel(pwmTimer, timerIndex2channel[motorID]);
                LL_TIM_CC_DisableChannel(pwmTimer, timerIndex2negativeChannel[motorID]);
            } else {
                LL_TIM_OC_SetPolarity(pwmTimer, timerIndex2channel[motorID], LL_TIM_OCPOLARITY_LOW);
                LL_TIM_OC_SetPolarity(pwmTimer, timerIndex2negativeChannel[motorID], LL_TIM_OCPOLARITY_HIGH);
                LL_TIM_CC_EnableChannel(pwmTimer, timerIndex2negativeChannel[motorID]);
                LL_TIM_CC_DisableChannel(pwmTimer, timerIndex2channel[motorID]);
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
    LL_TIM_InitTypeDef TIM_Init = {
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
    LL_TIM_Init(pwmTimer, &TIM_Init);
    for (uint32_t channel: timerIndex2channel)
    {
        LL_TIM_OC_Init(pwmTimer, channel, &TIM_OC_Init);
        LL_TIM_OC_EnablePreload(pwmTimer, channel);
    }
    LL_TIM_SetOffStates(pwmTimer, LL_TIM_OSSI_DISABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_GenerateEvent_UPDATE(pwmTimer);
    LL_TIM_EnableAllOutputs(pwmTimer);
    LL_TIM_EnableCounter(pwmTimer);

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
    while (true) {
        if (ledTest && HAL_GetTick() >= nextLedTime) {
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
        pinWrite(buzzerPin, isPressed(button4Pin) && isPressed(button3Pin));
        bool v = pinRead(usbBusDetectionPin);
        if (v != usbConnected) {
            usbConnected = v;
            if (usbConnected) {
                printf("USB connected\n");
            } else {
                printf("USB disconnected\n");
            }
        }
        if (LL_RTC_IsActiveFlag_SEC(RTC)) {
            LL_RTC_ClearFlag_SEC(RTC);
            if (isPressed(button4Pin) && isPressed(button2Pin)) {
                HAL_Delay(1);
                printf("Time %lu\n", LL_RTC_TIME_Get(RTC));
            }
        }
        if (HAL_GetTick() >= nextPowerTime) {
            nextPowerTime += powerPeriod;
            //printf("Power %9.4f\n", power);
            printf("Power %5d\n", int(power * maxPwm));
            for(uint8_t i = 0; i != 4; ++i)
                setMotorPower(i, power, isPressed(button1Pin));
            power += powerIncrement;
            if (fabsf(power) > 1) {
                power = copysignf(1, power);
                powerIncrement = -powerIncrement;
            }
        }
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
