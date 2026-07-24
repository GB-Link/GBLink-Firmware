#pragma once

#include <cstdint>

// Single source for the firmware version. It is reported over the
// protocol (GetFirmwareInfo) and folded into the USB bcdDevice (see usbLayer.cpp).
//
// Why bcdDevice tracks the version: Windows caches a device's MS OS / WinUSB
// driver binding keyed by VID/PID/bcdDevice. If bcdDevice never changes, a host
// that cached an older descriptor keeps the stale binding after a firmware
// update and will not re-read the corrected descriptors. Advancing bcdDevice on
// every release makes Windows treat it as a new device and re-enumerate cleanly.
namespace fw
{
    inline constexpr uint8_t versionMajor = 2;
    inline constexpr uint8_t versionMinor = 2;
    inline constexpr uint8_t versionPatch = 2;

    // Packed BCD nibbles 0x0<major><minor><patch>, e.g. 2.2.1 -> 0x0221.
    // Keep each field <= 9 to stay valid BCD.
    inline constexpr uint16_t bcdDevice =
        (static_cast<uint16_t>(versionMajor) << 8) |
        (static_cast<uint16_t>(versionMinor) << 4) |
        static_cast<uint16_t>(versionPatch);
}
