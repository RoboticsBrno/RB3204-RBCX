#pragma once

#include "rbcx.pb.h"

bool dispatcherEnqueueStatus(const CoprocStat& status);
void dispatcherPoll();
