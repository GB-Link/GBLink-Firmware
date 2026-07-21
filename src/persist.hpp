#pragma once

#include <cstdint>

// Persistent device settings, stored in the dedicated 4 KB flash "storage"
// partition (see boards/rpi_pico.overlay). One versioned struct holds the
// WebUSB landing-page flag and the per-mode LED colors.

// Per-mode LED colour slots (fixed order — shared with the launcher UI).
enum LedSlot : uint8_t {
    LED_SLOT_IDLE = 0,        // connected, no active mode (green)
    LED_SLOT_GBA = 1,         // GBA trade/link (yellow)
    LED_SLOT_GBC = 2,         // GB/GBC link (blue)
    LED_SLOT_PRINTER = 3,     // GB printer (purple)
    LED_SLOT_ADVANCE_WARS = 4,// Advance Wars (white)
    LED_SLOT_EREADER = 5,
    LED_SLOT_COUNT = 6,
};

// WebUSB landing-page advertising. Default (unwritten flash) is enabled.
bool landingPageEnabled();
void setLandingPageEnabled(bool enabled);

// Per-mode LED colours (logical RGB, 0-255; the WS2812 driver does GRB packing).
void getLedColor(uint8_t slot, uint8_t out[3]);
void setLedColor(uint8_t slot, uint8_t r, uint8_t g, uint8_t b);
void resetLedColors(); // restore all slots to the built-in defaults

// Apply a slot's stored colour to the LED now (used at mode entry).
void applyLedForSlot(uint8_t slot);
