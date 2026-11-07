#pragma once

#include <cstdint>
#include <span>

// Tracks which transport (WebUSB, WebSerial/CDC-ACM, or the bridge UART to a
// companion MCU) the device should currently send status and data back on, and
// provides a dispatch layer so call sites don't need to know which transport
// is active.
namespace Transport
{
    enum class Id : uint8_t { Usb, Serial, BridgeUart };

    using ReceiveHandler = void(*)(std::span<const uint8_t>, void*);

    // Each transport layer calls this from its receive path when a host frame
    // arrives, so the device replies on the same transport the host used.
    void setActive(Id id);
    Id active();

    // Register the same handler on every transport so the device accepts host
    // commands/data regardless of which one is open.
    void registerCommandHandler(ReceiveHandler handler, void* userData);
    void registerDataHandler(ReceiveHandler handler, void* userData);

    // Route a send to whichever transport the host last spoke on.
    bool sendStatus(std::span<const uint8_t, 2> data);
    bool sendData(std::span<const uint8_t> data);
}
