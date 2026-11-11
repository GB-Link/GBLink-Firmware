
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
// One per other player, in seat order: their frames arrive mixed, packet by packet.
static awproto::Maw1StreamParser g_parsers[awproto::maxPeers];
static uint8_t g_seat = 0;
static uint8_t g_playerCount = 2;

// Gates the transport data handler, which is registered before the link
// handshake completes and may also run a little past section teardown. With
// the proxy in static storage a stale frame is harmless; this just keeps
// pre-session noise out.
static volatile bool g_sessionActive = false;

static struct NextTransmit transmitCallback(void*)
{
    TxValues txValues;
    for (uint8_t i = 0; i < g_proxy.peerCount; i++) txValues.push(g_proxy.stagedTx[i]);
    return { g_seat, txValues, g_proxy.stagedTiming };
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
}

static void frameTrampoline(void* peer, uint16_t cmd, uint8_t state,
                            const uint16_t* words, uint16_t cnt)
{
    // The replay queues are popped from the PIO done-ISR; lock out interrupts
    // for the push so the two never interleave (pushes are small memcpys,
    // worst case ~0.5 KB, a few µs — well inside the ≥8 ms round period).
    const unsigned int key = irq_lock();
    g_proxy.applyNetFrame(static_cast<uint8_t>(reinterpret_cast<uintptr_t>(peer)), cmd, state, words, cnt, k_uptime_get_32());
    irq_unlock(key);
}

void awProto_receiveHandler(std::span<const uint8_t> data, void*)
{
    if (!g_sessionActive || data.empty()) return;

    // A packet starts with the seat it comes from; the peers are the other seats in order.
    const uint8_t seat = data[0];
    if (seat == g_seat || seat >= g_playerCount) return;
    const uint8_t peer = seat < g_seat ? seat : seat - 1;
    g_parsers[peer].push(data.data() + 1, data.size() - 1);
}

AwProtocolSection::AwProtocolSection(awproto::GameVariant variant, uint8_t seat, uint8_t playerCount)
{
    new (&g_proxy) awproto::AwProxy();

    g_seat = seat;
    g_playerCount = playerCount;
    m_chunk[0] = seat;

    g_proxy.cfg = awproto::AwConfig::forVariant(variant);
    // Seat 0's GBA is the bus master (player 1): the adapter is in SLAVE link mode.
    g_proxy.gbaIsMaster = (seat == 0);
    g_proxy.peerCount = playerCount - 1;
    g_proxy.emit = &emitTrampoline;

    for (uint8_t peer = 0; peer < awproto::maxPeers; peer++)
    {
        new (&g_parsers[peer]) awproto::Maw1StreamParser();
        g_parsers[peer].setCallback(&frameTrampoline, reinterpret_cast<void*>(static_cast<uintptr_t>(peer)));
    }

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
        // the GBA regardless. A player counts once it has spoken, so one who
        // hasn't started their game yet doesn't read as a drop.
        bool heard = false;
        bool quiet = false;
        for (uint8_t peer = 0; peer < g_proxy.peerCount; peer++)
        {
            const uint32_t lastHeard = g_proxy.peers[peer].lastHeardMs;
            if (lastHeard == 0) continue;
            heard = true;
            if ((k_uptime_get_32() - lastHeard) > awproto::linkTimeoutMs) quiet = true;
        }
        if (heard)
        {
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
            m_chunkLen = chunkHeader;
            continue;
        }

        if (m_chunkLen > chunkHeader)
        {
            // Ring drained mid-chunk. The ring only ever holds whole frames,
            // so this boundary is a frame boundary: pad and flush now rather
            // than waiting for more data (INTERSYNC latency matters).
            std::memset(&m_chunk[m_chunkLen], 0, sizeof(m_chunk) - m_chunkLen);
            if (Transport::sendData(std::span(m_chunk, sizeof(m_chunk))))
                m_chunkLen = chunkHeader;
        }
        return;
    }
}
