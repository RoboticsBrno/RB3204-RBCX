#include <stdint.h>

#ifndef __OLED_FONTS_H__
#define __OLED_FONTS_H__

#include "OledController_conf.hpp"

typedef struct {
	const uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef;

#ifdef OLED_INCLUDE_FONT_6x8
extern FontDef Font_6x8;
#endif
#ifdef OLED_INCLUDE_FONT_7x10
extern FontDef Font_7x10;
#endif
#ifdef OLED_INCLUDE_FONT_11x18
extern FontDef Font_11x18;
#endif
#ifdef OLED_INCLUDE_FONT_16x26
extern FontDef Font_16x26;
#endif
#endif // __OLED_FONTS_H__
