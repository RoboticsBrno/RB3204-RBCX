#pragma once

#include <stdint.h>
#include <stddef.h>

void secondaryUartInit();
bool controlLinkTxReady();
void controlLinkTxFrame(uint8_t *data, size_t len);
size_t controlLinkRxFrame(uint8_t *data, size_t len);
