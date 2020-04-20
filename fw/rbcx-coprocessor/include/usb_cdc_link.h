#pragma once

#include "usb.h"

#define CDC_EP0_SIZE    0x08
#define CDC_RXD_EP      0x01
#define CDC_TXD_EP      0x81
#define CDC_DATA_SZ     0x40
#define CDC_NTF_EP      0x82
#define CDC_NTF_SZ      0x08

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
