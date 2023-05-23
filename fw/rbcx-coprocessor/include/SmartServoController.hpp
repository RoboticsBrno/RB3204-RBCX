#pragma once

#include "rbcx.pb.h"

void smartServoInit();
void smartServoSendRequest(const CoprocReq_SmartServoReq& req);
void smartServoPoll();
