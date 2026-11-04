#pragma once

#include <stdint.h>

/* SD reaches the adapter on GP3 (port pin 4) or GP4 (port pin 1) depending on the cable. The GBA link
 * PIO programs handle both pins as one 2-bit group, GP3 = bit 0 and GP4 = bit 1. The pin that does not
 * carry SD reads high, from the partner's VCC or from the pull-up, so a bit is 0 when either pin reads 0. */

#define SD_PIN_GP3 0x1u
#define SD_PIN_GP4 0x2u

/* Each bit doubled for `out pins, 2`. The OSR shifts right, so the word goes out LSB first. */
static inline uint32_t sdPinsSpread(uint16_t value)
{
    uint32_t x = value;
    x = (x | (x << 8)) & 0x00FF00FFu;
    x = (x | (x << 4)) & 0x0F0F0F0Fu;
    x = (x | (x << 2)) & 0x33333333u;
    x = (x | (x << 1)) & 0x55555555u;
    return x | (x << 1);
}

/* 16 samples of both pins, shifted in left so the first bit sits in bits 31:30. Returns the word LSB
 * first, as it was sent. */
static inline uint16_t sdPinsDecode(uint32_t raw)
{
    uint32_t x = raw & (raw >> 1) & 0x55555555u;   /* bit 2k = GP3 AND GP4 of sample 15-k */
    x = (x | (x >> 1)) & 0x33333333u;
    x = (x | (x >> 2)) & 0x0F0F0F0Fu;
    x = (x | (x >> 4)) & 0x00FF00FFu;
    x = (x | (x >> 8)) & 0x0000FFFFu;
    x = ((x & 0x5555u) << 1) | ((x & 0xAAAAu) >> 1);
    x = ((x & 0x3333u) << 2) | ((x & 0xCCCCu) >> 2);
    x = ((x & 0x0F0Fu) << 4) | ((x & 0xF0F0u) >> 4);
    x = ((x & 0x00FFu) << 8) | ((x & 0xFF00u) >> 8);
    return (uint16_t)x;
}

/* The pin a received word came in on: zeros on exactly one pin while the other stayed high. 0 when no
 * pin saw a zero (an all-ones word, or no reply) or both did (partner powering off). */
static inline uint8_t sdPinsFramePin(uint32_t raw)
{
    const uint32_t low = ~raw;
    const int gp3 = (low & 0x55555555u) != 0;
    const int gp4 = (low & 0xAAAAAAAAu) != 0;
    if (gp3 == gp4) return 0;
    return gp3 ? SD_PIN_GP3 : SD_PIN_GP4;
}
