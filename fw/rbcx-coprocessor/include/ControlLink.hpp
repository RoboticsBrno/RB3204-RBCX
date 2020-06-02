#pragma once

#include <stddef.h>
#include <stdint.h>

#include "rbcx.pb.h"

void controlUartInit();
bool controlLinkTxReady();
void controlLinkTx(const CoprocStat& outgoing);
bool controlLinkRx(CoprocReq& incoming);
