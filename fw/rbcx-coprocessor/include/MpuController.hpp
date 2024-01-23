#pragma once

#include "rbcx.pb.h"

void mpuDispatch(const CoprocReq_MpuReq& request);
void mpuTick();

void mpuCreate();
void mpuInitialize();
void mpuReset();
void mpuOnIntTriggered();
