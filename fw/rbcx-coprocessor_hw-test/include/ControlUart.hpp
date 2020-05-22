#pragma once

#include <stdint.h>
#include <stddef.h>

void controlUartInit();
bool controlUartTxReady();
void controlUartTxFrame(uint8_t *data, size_t len);
size_t controlUartRxFrame(uint8_t *data, size_t len);
