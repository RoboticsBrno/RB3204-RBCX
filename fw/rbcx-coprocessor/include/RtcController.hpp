#pragma once

#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_rtc.h"

inline void rtcInit() {
    // LSE and RTC started at board init

    LL_PWR_EnableBkUpAccess();

    // We ensure RTC is running (noop in case it is)
    LL_RTC_InitTypeDef init = {
        .AsynchPrescaler = 0x7FFF, // 32kHz crystal -> 1s tick
    };
    LL_RTC_Init(RTC, &init);
}

inline bool rtcSetTime(uint32_t seconds) {
    bool ok = true;
    ok &= LL_RTC_EnterInitMode(RTC) == SUCCESS;
    LL_RTC_TIME_Set(RTC, seconds);
    ok &= LL_RTC_ExitInitMode(RTC) == SUCCESS;
    return ok;
}

inline uint32_t rtcGetTime() { return LL_RTC_TIME_Get(RTC); }