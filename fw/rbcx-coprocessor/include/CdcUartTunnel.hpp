#pragma once

#include "usb_cdc.h"

void tunnelUartInit();
void tunnelPoll();
bool tunnelOnSetLineCoding(
    const usb_cdc_line_coding& old, const usb_cdc_line_coding& current);
