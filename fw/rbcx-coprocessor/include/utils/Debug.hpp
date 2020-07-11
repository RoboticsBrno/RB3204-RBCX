#pragma once

#include <array>

#include "FreeRTOS.h"
#include "task.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#ifndef NDEBUG

#define DEBUG(fmt, ...)                                                        \
    printf("[%10lu][" __FILE__ ":" TOSTRING(__LINE__) "]: " fmt,               \
        xTaskGetTickCount(), ##__VA_ARGS__)

inline void DEBUG_HEX(const uint8_t* data, size_t len) {
    static const char* hex = "0123456789ABCDEF";
    char buf[] = { '0', '0', ' ' };

    for (size_t i = 0; i < len; ++i) {
        buf[0] = hex[data[i] >> 4];
        buf[1] = hex[data[i] & 0xF];
        fwrite(buf, sizeof(buf), 1, stdout);
    }
    putchar('\n');
}

#else

#define DEBUG(fmt, ...)                                                        \
    do {                                                                       \
    } while (0);

inline void DEBUG_HEX(const uint8_t* data, size_t len) {}
#endif

inline bool isInInterrupt() {
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

void rebootToDfu();
