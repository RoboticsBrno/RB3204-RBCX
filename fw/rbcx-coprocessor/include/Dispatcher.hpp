#pragma once

#include "rbcx.pb.h"

void dispatcherInit();
bool dispatcherEnqueueStatus(const CoprocStat& status);
void dispatcherPoll();
