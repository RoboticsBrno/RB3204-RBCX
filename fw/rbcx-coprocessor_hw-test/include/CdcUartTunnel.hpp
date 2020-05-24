#pragma once

void tunnelUartInit();
void tunnelPoll();
void tunnelUartTx(const uint8_t* data, size_t len);
