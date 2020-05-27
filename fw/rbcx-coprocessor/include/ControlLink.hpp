#pragma once

#include <stdint.h>
#include <stddef.h>

#include "rbcx.pb.h"


void controlUartInit();
bool controlLinkTxReady();
void controlLinkTx(const StmMessage &outgoing);
bool controlLinkRx(EspMessage &incoming);
