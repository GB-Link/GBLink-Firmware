
#include "rawRelaySection.hpp"
#include "../layers/transport.hpp"

// TX queue: USB → GBA (filled by USB handler, drained by PIO ISR)
// Element size = 2 bytes (one uint16_t), depth = 256
K_MSGQ_DEFINE(g_rawRelayTxQueue, 2, 256, 2);

// RX queue: GBA → USB (filled by PIO ISR, drained by process() loop)
K_MSGQ_DEFINE(g_rawRelayRxQueue, 2, 256, 2);

void rawRelay_receiveHandler(std::span<const uint8_t> data, void*)
{
    // 64-byte USB packet = 32 x uint16_t values
    const uint16_t* words = reinterpret_cast<const uint16_t*>(data.data());
    for (size_t i = 0; i < 32 && i * 2 < data.size(); i++)
    {
        if (words[i] != 0x0000)
        {
            k_msgq_put(&g_rawRelayTxQueue, &words[i], K_NO_WAIT);
        }
    }
}

RawRelaySection::RawRelaySection()
{
    k_msgq_purge(&g_rawRelayTxQueue);
    k_msgq_purge(&g_rawRelayRxQueue);

    link_setLegacyTransmitCallback(&transmitCallback, this);
    link_setLegacyReceiveCallback(&receiveCallback, this);
    link_setLegacyTransiveDoneCallback(&transiveDoneCallback, this);
}

RawRelaySection::~RawRelaySection()
{
    link_setLegacyTransmitCallback(nullptr, nullptr);
    link_setLegacyReceiveCallback(nullptr, nullptr);
    link_setLegacyTransiveDoneCallback(nullptr, nullptr);
}

void RawRelaySection::process()
{
    while (!m_cancel)
    {
        uint16_t value;

        // Drain RX queue into USB accumulation buffer
        while (k_msgq_get(&g_rawRelayRxQueue, &value, K_NO_WAIT) == 0)
        {
            m_rxPacket[m_rxPacketIndex++] = value;

            if (m_rxPacketIndex == 32)
            {
                flushRxBuffer();
            }
        }

        // Flush partial buffer periodically to avoid stale data
        if (m_rxPacketIndex > 0)
        {
            flushRxBuffer();
        }

        // Sleep briefly to yield CPU; ~10ms gives good balance
        // between latency and CPU usage
        k_sleep(K_MSEC(10));
    }

    // Flush any remaining data
    if (m_rxPacketIndex > 0)
    {
        flushRxBuffer();
    }
}

void RawRelaySection::flushRxBuffer()
{
    Transport::sendData(
        std::span(reinterpret_cast<const uint8_t*>(m_rxPacket.data()), 64)
    );

    std::memset(m_rxPacket.data(), 0x00, 64);
    m_rxPacketIndex = 0;
}

struct LegacyTransmit RawRelaySection::transmitCallback(void* userData)
{
    RawRelaySection* self = static_cast<RawRelaySection*>(userData);

    uint16_t value = 0x7FFF; // CMD_NONE: safe idle value for GBA Multi mode
    k_msgq_get(&g_rawRelayTxQueue, &value, K_NO_WAIT);

    return { value, self->defaultTimingUs };
}

void RawRelaySection::receiveCallback(uint16_t rx, void* userData)
{
    (void)userData;
    k_msgq_put(&g_rawRelayRxQueue, &rx, K_NO_WAIT);
}

void RawRelaySection::transiveDoneCallback(uint16_t rx, uint16_t tx, void* userData)
{
    (void)rx;
    (void)tx;
    (void)userData;
}
