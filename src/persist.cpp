#include "persist.hpp"
#include "hardware.hpp"

#include <cstring>
#include <zephyr/storage/flash_map.h>

namespace
{
    #define STORAGE_PARTITION_ID FIXED_PARTITION_ID(storage_partition)

    constexpr uint8_t SETTINGS_MAGIC = 0x5A;
    constexpr uint8_t SETTINGS_VERSION = 6; // bumped for the new RFU LED slot

    struct PersistSettings
    {
        uint8_t magic;
        uint8_t version;
        uint8_t landing;                       // 0 = disabled, else enabled
        uint8_t led[LED_SLOT_COUNT][3];        // per-slot logical RGB
    };

    // Real-colour defaults at a uniform 25% brightness (64/255) so every mode's
    // brightness slider starts at the same place. WS2812 magnitude = brightness.
    const uint8_t kDefaultColors[LED_SLOT_COUNT][3] = {
        { 0,   64,  0   }, // idle/connected — green
        { 64,  64,  0   }, // GBA/Celio — yellow
        { 0,   0,   64  }, // GB/GBC — blue
        { 64,  0,   64  }, // printer — purple
        { 64,  64,  64  }, // Advance Wars — white
        { 0,   64,  64  }, // RFU wireless — cyan
    };

    void fillDefaults(PersistSettings& s)
    {
        s.magic = SETTINGS_MAGIC;
        s.version = SETTINGS_VERSION;
        s.landing = 1;
        memcpy(s.led, kDefaultColors, sizeof(s.led));
    }

    // Load the settings struct, falling back to defaults when flash is unwritten
    // or from an older layout (magic/version mismatch).
    PersistSettings loadSettings()
    {
        PersistSettings s{};
        const struct flash_area *fa;
        if (flash_area_open(STORAGE_PARTITION_ID, &fa) != 0) {
            fillDefaults(s);
            return s;
        }
        flash_area_read(fa, 0, &s, sizeof(s));
        flash_area_close(fa);

        if (s.magic != SETTINGS_MAGIC || s.version != SETTINGS_VERSION) {
            fillDefaults(s);
        }
        return s;
    }

    void saveSettings(const PersistSettings& s)
    {
        const struct flash_area *fa;
        if (flash_area_open(STORAGE_PARTITION_ID, &fa) != 0) {
            return;
        }
        // Flash must be erased before writing; RP2040 write-block-size is 1.
        flash_area_erase(fa, 0, fa->fa_size);
        flash_area_write(fa, 0, &s, sizeof(s));
        flash_area_close(fa);
    }
}

bool landingPageEnabled()
{
    return loadSettings().landing != 0;
}

void setLandingPageEnabled(bool enabled)
{
    PersistSettings s = loadSettings();
    s.landing = enabled ? 1 : 0;
    saveSettings(s);
}

void getLedColor(uint8_t slot, uint8_t out[3])
{
    if (slot >= LED_SLOT_COUNT) { out[0] = out[1] = out[2] = 0; return; }
    PersistSettings s = loadSettings();
    out[0] = s.led[slot][0];
    out[1] = s.led[slot][1];
    out[2] = s.led[slot][2];
}

void setLedColor(uint8_t slot, uint8_t r, uint8_t g, uint8_t b)
{
    if (slot >= LED_SLOT_COUNT) return;
    PersistSettings s = loadSettings();
    s.led[slot][0] = r;
    s.led[slot][1] = g;
    s.led[slot][2] = b;
    saveSettings(s);
}

void resetLedColors()
{
    PersistSettings s = loadSettings();
    memcpy(s.led, kDefaultColors, sizeof(s.led));
    saveSettings(s);
}

void applyLedForSlot(uint8_t slot)
{
    uint8_t c[3];
    getLedColor(slot, c);
    Hardware::getInstance().setLED(c[0], c[1], c[2], true);
}
