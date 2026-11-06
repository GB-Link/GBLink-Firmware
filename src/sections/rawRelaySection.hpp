
#include <cstdint>
#include <cstring>
#include <array>

extern "C"
{
    #include "../layers/linkLayer.h"
}

#include "../layers/usbLayer.hpp"
#include "../linkStatus.hpp"

#pragma once

class RawRelaySection
{
public:
    RawRelaySection();
    ~RawRelaySection();

    void process();

    void cancel()
    {
        m_cancel = true;
    }

private:

    void flushRxBuffer();

    static struct LegacyTransmit transmitCallback(void* userData);
    static void receiveCallback(uint16_t rx, void* userData);
    static void transiveDoneCallback(uint16_t rx, uint16_t tx, void* userData);

    bool m_cancel = false;

    std::array<uint16_t, 32> m_rxPacket = {};
    size_t m_rxPacketIndex = 0;

    // Default timing: ~8.3ms between exchanges (suitable for Advance Wars)
    // At PIO clkdiv 67.816 (~540ns/instruction), 15370 iterations ≈ 8.3ms
    static constexpr uint32_t defaultTimingUs = 15370;
};

void rawRelay_receiveHandler(std::span<const uint8_t> data, void*);
