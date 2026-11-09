
#include "awProtocolSection.hpp"

#include <cstring>
#include <new>

#include <zephyr/sys/ring_buffer.h>

#include "../layers/transport.hpp"
#include "../linkStatus.hpp"


#include "../layers/link/gbaLinkLayer.hpp"


// Outbound MAW1 byte stream. Single producer (PIO done-ISR), single consumer
// (process() thread) — the safe concurrent use of a Zephyr ring_buf.
RING_BUF_DECLARE(g_awOutRing, 2048);

static awproto::AwProxy g_proxy;
static awproto::Maw1StreamParser g_parser;

// Gates the transport data handler, which is registered before the link
// handshake completes and may also run a little past section teardown. With
// the proxy in static storage a stale frame is harmless; this just keeps
// pre-session noise out.
static volatile bool g_sessionActive = false;

static struct NextTransmit transmitCallback(void*)
{
    // The GBA is player 1 (seat 0) as bus master, else player 2 (seat 1). The slave program ignores the seat.
    const uint8_t gbaSeat = g_proxy.gbaIsMaster ? 0 : 1;
    return { gbaSeat, TxValues(g_proxy.stagedTx), g_proxy.stagedTiming };
}

static void receiveCallback(uint16_t rx, void*)
{
    g_proxy.onRound(rx, k_uptime_get_32());
}

static void emitTrampoline(void*, uint16_t cmd, uint8_t state,
                           const uint16_t* words, uint16_t cnt)
{
    // Only ever called from the PIO done-ISR (single producer), so a single
    // static scratch buffer is safe.
    static uint8_t frame[awproto::maxFrameBytes];
    const size_t len = awproto::maw1Serialize(frame, cmd, state, words, cnt);

    // All-or-nothing: a partially written frame would corrupt the stream and
    // the chunker's frame-boundary padding guarantee.
    if (ring_buf_space_get(&g_awOutRing) >= len)
    {
        ring_buf_put(&g_awOutRing, frame, len);
    }
    else
    {
        g_proxy.counters.txDropRingFull++;
    }
}

static void frameTrampoline(void*, uint16_t cmd, uint8_t state,
                            const uint16_t* words, uint16_t cnt)
{
    // The replay queue is popped from the PIO done-ISR; lock out interrupts
    // for the push so the two never interleave (pushes are small memcpys,
    // worst case ~0.5 KB, a few µs — well inside the ≥8 ms round period).
    const unsigned int key = irq_lock();
    g_proxy.applyNetFrame(cmd, state, words, cnt, k_uptime_get_32());
    irq_unlock(key);
}

void awProto_receiveHandler(std::span<const uint8_t> data, void*)
{
    if (g_sessionActive)
    {
        g_parser.push(data.data(), data.size());
    }
}

AwProtocolSection::AwProtocolSection(awproto::GameVariant variant, enum MultiMode multiMode)
{
    new (&g_proxy) awproto::AwProxy();
    new (&g_parser) awproto::Maw1StreamParser();

    g_proxy.cfg = awproto::AwConfig::forVariant(variant);
    // SLAVE link mode means the attached GBA is the bus master (player 1).
    g_proxy.gbaIsMaster = (multiMode == SLAVE);
    g_proxy.emit = &emitTrampoline;

    g_parser.setCallback(&frameTrampoline, nullptr);

    ring_buf_reset(&g_awOutRing);

    link_setTransmitCallback(&transmitCallback, this);
    link_setReceiveCallback(&receiveCallback, this);

    g_sessionActive = true;
}

AwProtocolSection::~AwProtocolSection()
{
    g_sessionActive = false;

    // The module disables the link (multiMode_selectMode(DISABLED)) before this
    // destructor runs, so no PIO ISR can race the deregistration below.
    link_setTransmitCallback(nullptr, nullptr);
    link_setReceiveCallback(nullptr, nullptr);
}

void AwProtocolSection::process()
{
    while (!m_cancel)
    {
        pumpOutbound();

        // Link supervision: status reporting only, the proxy keeps answering
        // the GBA regardless. Skipped until the remote has spoken once, so a
        // partner who hasn't started their game yet doesn't read as a drop.
        const uint32_t lastHeard = g_proxy.lastHeardMs;
        if (lastHeard != 0)
        {
            const bool quiet =
                (k_uptime_get_32() - lastHeard) > awproto::linkTimeoutMs;
            if (quiet && !m_reportedReconnecting)
            {
                sendLinkStatus(LinkStatus::LinkReconnecting);
                m_reportedReconnecting = true;
            }
            else if (!quiet && m_reportedReconnecting)
            {
                sendLinkStatus(LinkStatus::LinkConnected);
                m_reportedReconnecting = false;
            }
        }

        k_sleep(K_MSEC(2));
    }
}

void AwProtocolSection::pumpOutbound()
{
    for (;;)
    {
        if (m_chunkLen < sizeof(m_chunk))
        {
            m_chunkLen += ring_buf_get(&g_awOutRing, &m_chunk[m_chunkLen],
                                       sizeof(m_chunk) - m_chunkLen);
        }

        if (m_chunkLen == sizeof(m_chunk))
        {
            if (!Transport::sendData(std::span(m_chunk, sizeof(m_chunk))))
                return;  // transport busy/down — keep the chunk, retry next pass
            m_chunkLen = 0;
            continue;
        }

        if (m_chunkLen > 0)
        {
            // Ring drained mid-chunk. The ring only ever holds whole frames,
            // so this boundary is a frame boundary: pad and flush now rather
            // than waiting for more data (INTERSYNC latency matters).
            std::memset(&m_chunk[m_chunkLen], 0, sizeof(m_chunk) - m_chunkLen);
            if (Transport::sendData(std::span(m_chunk, sizeof(m_chunk))))
                m_chunkLen = 0;
        }
        return;
    }
}
