#pragma once

#include "usb.h"

#define CDC_EP0_SIZE    0x08
#define CDC_RXD_EP      0x01
#define CDC_TXD_EP      0x81
#define CDC_DATA_SZ     0x40
#define CDC_NTF_EP      0x82
#define CDC_NTF_SZ      0x08

extern usbd_device udev;

#ifdef __cplusplus
extern "C" {
#endif

void cdc_link_init();
void cdc_link_poll();

#ifdef __cplusplus
}
#endif
