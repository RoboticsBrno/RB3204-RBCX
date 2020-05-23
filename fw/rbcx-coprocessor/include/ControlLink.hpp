#pragma once

#include <stdint.h>
#include <stddef.h>

#include "rbcx.pb.h"


void secondaryUartInit();
bool controlLinkTxReady();
void controlLinkTx(const StmMessage &outgoing);
bool controlLinkRx(EspMessage &incoming);
