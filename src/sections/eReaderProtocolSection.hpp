#pragma once

#include <cstdint>
#include <span>

#include "eReaderProtocol.hpp"

#include <zephyr/kernel.h>

extern "C"
{
    #include "../layers/linkLayer.h"
}

class EReaderProtocolSection
{
public:
    EReaderProtocolSection(erproto::Profile profile);
    ~EReaderProtocolSection();

    // Detects the cable and arms the profile's PIO link (sleeps ~200ms for the
    // Pokemon partner-presence window); call from the module thread before
    // process().
    void armLink();

    void process();

    void cancel() { m_cancel = true; }

private:
    void pumpOutbound();

    bool m_cancel = false;
    uint8_t m_chunk[64] = {};
    size_t m_chunkLen = 0;
};

void erProto_receiveHandler(std::span<const uint8_t> data, void*);
