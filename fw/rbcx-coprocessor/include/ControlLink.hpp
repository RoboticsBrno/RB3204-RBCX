#pragma once

#include <stddef.h>
#include <stdint.h>

#include "rbcx.pb.h"

void controlUartInit();
bool controlLinkTxReady();
void controlLinkTx(const StmMessage& outgoing);
bool controlLinkRx(EspMessage& incoming);
