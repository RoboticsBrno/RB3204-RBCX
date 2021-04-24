#pragma once

#include "rbcx.pb.h"

#include <stdint.h>

void rtcInit();
bool rtcInitReady();
void rtcDispatch(const CoprocReq_RtcReq& req);

uint32_t rtcGetTime();
void rtcSetTime(uint32_t seconds);
uint32_t rtcGetAlarm();
void rtcSetAlarm(uint32_t seconds);
