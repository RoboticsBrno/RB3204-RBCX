#pragma once

#include <array>

#include "FreeRTOS.h"
#include "task.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define DEBUG(fmt, ...)                                                        \
    printf("[%10lu][" __FILE__ ":" TOSTRING(__LINE__) "]: " fmt,               \
        xTaskGetTickCount(), ##__VA_ARGS__)

inline void DEBUG_HEX(const uint8_t* data, size_t len) {
    static const char* hex = "0123456789ABCDEF";
    std::array<char, 4> buf { '0', '0', ' ' };

    for (size_t i = 0; i < len; ++i) {
        buf[0] = hex[data[i] & 0xF];
        buf[1] = hex[data[i] >> 4];
        fwrite(buf.data(), buf.size(), 1, stdout);
    }
    putchar('\n');
}

inline bool isInInterrupt() {
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}
