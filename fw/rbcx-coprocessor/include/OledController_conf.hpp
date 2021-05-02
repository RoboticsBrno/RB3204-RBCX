/**
 * Private configuration file for the OLED library.
 * This example is configured for STM32F0, I2C and including all fonts.
 */
#include "I2Cdev.hpp"

#ifndef __OLED_CONF_H__
#define __OLED_CONF_H__



// Mirror the screen if needed
#define OLED_MIRROR_VERT
#define OLED_MIRROR_HORIZ

// Set inverse color if needed
// # define OLED_INVERSE_COLOR

// Include only needed fonts
#define OLED_INCLUDE_FONT_6x8
#define OLED_INCLUDE_FONT_7x10
#define OLED_INCLUDE_FONT_11x18
#define OLED_INCLUDE_FONT_16x26

// Some OLEDs don't display anything in first two columns.
// In this case change the following macro to 130.
// The default value is 128.
// #define OLED_WIDTH           130

// The height can be changed as well if necessary.
// It can be 32, 64 or 128. The default value is 64.
// #define OLED_HEIGHT          64

#endif /* __OLED_CONF_H__ */
