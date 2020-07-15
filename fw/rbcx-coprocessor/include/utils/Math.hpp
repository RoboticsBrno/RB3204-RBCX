#pragma once

#include <stdint.h>

/// https://github.com/chmike/fpsqrt/blob/master/fpsqrt.c
/// Compute int32_t to int32_t square root
/// RETURNS the integer square root of v
/// REQUIRES v is positive
int32_t sqrti(int32_t v) {
    uint32_t t, q, b, r;
    r = v; // r = v - x²
    b = 0x40000000; // a²
    q = 0; // 2ax
    while (b > 0) {
        t = q + b; // t = 2ax + a²
        q >>= 1; // if a' = a/2, then q' = q/2
        if (r >= t) // if (v - x²) >= 2ax + a²
        {
            r -= t; // r' = (v - x²) - (2ax + a²)
            q += b; // if x' = (x + a) then ax' = ax + a², thus q' = q' + b
        }
        b >>= 2; // if a' = a/2, then b' = b / 4
    }
    return q;
}
