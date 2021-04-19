#include "RtcController.hpp"

#include "ControlLink.hpp"
#include "utils/Debug.hpp"

#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_rtc.h"

static void ensureInitialized() {
    // Ensure RTC is running (noop in case it is)
    LL_RTC_InitTypeDef init = {
        .AsynchPrescaler = 0x7FFF, // 32kHz crystal -> 1s tick
        .OutPutSource = LL_RTC_CALIB_OUTPUT_NONE,
    };

    LL_RTC_Init(RTC, &init);
}

void rtcInit() {
    LL_PWR_EnableBkUpAccess();

    // Make sure the 32kHz crystal is started.
    // We don't wait for LSERDY as it can take several seconds.
    LL_RCC_LSE_Enable();
}

bool rtcInitReady() { return LL_RCC_LSE_IsReady(); }

static void sendStatus() {
    CoprocStat status;
    status.which_payload = CoprocStat_rtcStat_tag,
    status.payload.rtcStat = CoprocStat_RtcStat {
        .time = rtcGetTime(),
        .alarm = rtcGetAlarm(),
        .notReady = !rtcInitReady(),
    };
    controlLinkTx(status);
}

void rtcDispatch(const CoprocReq_RtcReq& req) {
    switch (req.which_rtcCmd) {
    case CoprocReq_RtcReq_get_tag:
        sendStatus();
        break;
    case CoprocReq_RtcReq_setTime_tag:
        if (rtcInitReady()) {
            rtcSetTime(req.rtcCmd.setTime);
        }
        sendStatus();
        break;
    case CoprocReq_RtcReq_setAlarm_tag:
        if (rtcInitReady()) {
            rtcSetAlarm(req.rtcCmd.setAlarm);
        }
        sendStatus();
        break;
    }
}

uint32_t rtcGetTime() { return LL_RTC_TIME_Get(RTC); }

void rtcSetTime(uint32_t seconds) {
    ensureInitialized();
    LL_RTC_TIME_SetCounter(RTC, seconds);
}

uint32_t rtcGetAlarm() {
    uint32_t seconds = 0;
    seconds |= LL_RTC_BKP_GetRegister(BKP, LL_RTC_BKP_DR1);
    seconds |= LL_RTC_BKP_GetRegister(BKP, LL_RTC_BKP_DR2) << 16;
    return seconds;
}

void rtcSetAlarm(uint32_t seconds) {
    ensureInitialized();
    LL_RTC_ALARM_SetCounter(RTC, seconds);

    // We mirror the value into DR1 and DR2 16-bit regs
    // because ALARM is write-only and LL_RTC_ALARM_Get
    // returns garbage after reset.
    LL_RTC_BKP_SetRegister(BKP, LL_RTC_BKP_DR1, seconds);
    LL_RTC_BKP_SetRegister(BKP, LL_RTC_BKP_DR2, seconds >> 16);
}
