#pragma once

#include "usb.h"

void tunnel_uart_init();

#ifdef __cplusplus
extern "C" {
#endif

void cdc_link_init();
void cdc_link_poll();

void cdc_link_rx_handler(usbd_device *dev, uint8_t ep);
void cdc_link_tx_handler(usbd_device *dev, uint8_t ep);

#ifdef __cplusplus
}

#endif
