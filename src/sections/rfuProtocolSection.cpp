
#include "rfuProtocolSection.hpp"

#include <cstring>
#include <new>

#include <zephyr/sys/ring_buffer.h>

#include "../layers/transport.hpp"
#include "../linkStatus.hpp"
#include "../hardware.hpp"

// Outbound RFU1 byte stream. Producers: PIO done-ISR (command-triggered
// sends), transport context (connect ACKs), section thread (broadcast
// ticks) — every put is wrapped in irq_lock so they never interleave.
// Consumer: process() thread.
RING_BUF_DECLARE(g_rfuOutRing, 2048);

static rfuproto::RfuCore g_core;
static rfuproto::Rfu1StreamParser g_parser;

// Wakes the section thread as soon as network data arrives, so wait-events
// resolve with transport latency instead of poll latency.
K_SEM_DEFINE(g_rfuNetRxSem, 0, 1);

static volatile bool g_sessionActive = false;

// Last time a valid RFU1 frame arrived from the remote (0 = never) — drives
// the LinkReconnecting supervision.
static volatile uint32_t g_lastNetRxMs = 0;

// Reverse-phase visibility: the master exchange reads back the GBA's SO line
// each event word = whatever the GBA-as-slave shifted out. For a DATA_READY
// event the GBA stages its ACK 0x996600A8 after accepting word 1, so the
// word-2 readback == 0x996600A8 means the GBA ACCEPTED the event; its
// unchanged 0x80000000 reversal pre-load (or a shift of it) means the word
// was mis-clocked. g_revRx1 = last word, g_revRx0 = prior.
static volatile uint32_t g_revRx0 = 0;
static volatile uint32_t g_revRx1 = 0;

// Where the last unsuccessful delivery failed (telemetry byte 56 of the 0x2E
// frame): low nibble = stage, high nibble = word index. 0 = the last delivery
// succeeded. Byte 57 = the GBA SO level sampled at that failure.
enum DelivFail : uint8_t
{
    dfNone = 0,
    dfPrecheckSoLow = 1,   // retired: armed SO rests LOW (HW test #25/#28)
    dfExchangeTimeout = 2, // PIO exchange did not complete (SM never pushed)
    dfDanceSoHigh = 3,     // GBA never raised SO after a word (handshake)
    dfFinalSoLow = 4,      // GBA never dropped SO after the last word
};
static volatile uint8_t g_dbgDelivFail = dfNone;
static volatile uint8_t g_dbgDelivSo = 0;
// Soft-retry budget per wait: systematic word-1 failures retry every poll
// (the GBA stays armed), but must eventually give the bus back.
static uint32_t g_retriesThisWait = 0;
// SD-pulse resets this session (must be visible: a spurious one silently
// wipes live link state — HW test #40).
static volatile uint8_t g_sdResets = 0;
static constexpr uint32_t kMaxRetriesPerWait = 400;

// Stall forensics: when the GBA fails to clock a wait-class ACK-read for
// >250ms (the observed SEND_DATAW death, tests #33-35), latch a one-shot
// snapshot of the wires + slave SM state. Reported as frame tag 0x2F.
static volatile bool g_stallValid = false;
static volatile uint8_t g_stallPc = 0, g_stallTx = 0, g_stallRx = 0, g_stallLines = 0;
static volatile uint32_t g_stallLastRx = 0;
static uint32_t g_waitEnteredMs = 0;
static bool g_waitTracked = false;

// Non-beacon frames that arrived while the GBA was mid-detection or mid-
// command. applyNetPacket runs under irq_lock, which masks the PIO done-ISR;
// during detection that desyncs the timer-paced ID dance, and during a
// command exchange (2 MHz, ~40µs inter-transfer gaps) a delayed ISR misses
// the GBA's next transfer — the slave SM sits at `pull` through it and the
// exchange desyncs mid-command (observed: the LL-ACK arriving ~10ms behind
// the child's own send stalled the SEND_DATAW ack-read in tests #33/#34).
// Only comWaitCmd (idle between commands) and the reversed states (GBA is
// slave, not clocking) may apply inbound frames directly.
static inline bool safeToApplyInbound(uint8_t comstate)
{
    return comstate == rfuproto::comWaitCmd ||
           comstate == rfuproto::comWaitEvent ||
           comstate == rfuproto::comWaitResp;
}
struct ParkedFrame
{
    uint32_t ptype;
    uint32_t hdata;
    uint16_t payloadLen;
    uint8_t payload[92];
};
static constexpr uint8_t kParkedMax = 8;
static ParkedFrame g_parked[kParkedMax];
static volatile uint8_t g_parkedCount = 0;

static uint16_t randTrampoline(void*)
{
    // Device IDs only need to vary between sessions, not be unpredictable —
    // gpsp seeds from cpu_ticks for the same reason. xorshift mixed with the
    // cycle counter avoids a Kconfig entropy-driver dependency.
    static uint32_t s = 0;
    if (s == 0) s = k_cycle_get_32() | 1;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return static_cast<uint16_t>(s ^ k_cycle_get_32());
}

static void emitTrampoline(void*, const uint8_t* frame, size_t len)
{
    // All-or-nothing: a partial frame would corrupt the stream and the
    // chunker's frame-boundary padding guarantee.
    const unsigned int key = irq_lock();
    if (ring_buf_space_get(&g_rfuOutRing) >= len)
    {
        ring_buf_put(&g_rfuOutRing, frame, len);
    }
    irq_unlock(key);
}

static void pioDoneCallback(uint32_t rx, void*)
{
    // GBA-master role only (the adapter-master delivery reads its words
    // synchronously and never raises the PIO interrupt).
    g_core.onSlaveTransfer(rx, k_uptime_get_32());
    rfuLink_pushTx(g_core.stagedTx);

    // WAIT-class ACK just clocked out (comstate is now comWaitEvent): the GBA
    // has dropped to slave. Wake the section thread so pollWaitEvent answers
    // promptly (librfu arms no timer before word 1, so this is a latency trim,
    // not a race — but it keeps delivery snappy).
    if (g_core.comstate == rfuproto::comWaitEvent)
        k_sem_give(&g_rfuNetRxSem);
}

static void frameTrampoline(void*, uint32_t ptype, uint32_t hdata,
                            const uint8_t* payload, uint16_t payloadLen)
{
    // Apply inbound frames only while the GBA is not actively clocking us
    // (see safeToApplyInbound). Beacons repeat and are safe to drop when they
    // can't apply; anything else is PARKED and drained by the section thread
    // a poll-tick later, so nothing is swallowed.
    if (!safeToApplyInbound(g_core.comstate))
    {
        if (ptype == rfuproto::NET_BROADCAST) return;
        if (g_parkedCount < kParkedMax && payloadLen <= sizeof(g_parked[0].payload))
        {
            ParkedFrame& p = g_parked[g_parkedCount];
            p.ptype = ptype;
            p.hdata = hdata;
            p.payloadLen = payloadLen;
            std::memcpy(p.payload, payload, payloadLen);
            g_parkedCount = g_parkedCount + 1;
        }
        return;
    }

    const uint32_t now = k_uptime_get_32();
    g_lastNetRxMs = now ? now : 1;

    const unsigned int key = irq_lock();
    g_core.applyNetPacket(ptype, hdata, payload, payloadLen, now);
    irq_unlock(key);
    k_sem_give(&g_rfuNetRxSem);
}

void rfuProto_receiveHandler(std::span<const uint8_t> data, void*)
{
    if (g_sessionActive)
    {
        g_parser.push(data.data(), data.size());
    }
}

RfuProtocolSection::RfuProtocolSection(uint8_t role)
{
    new (&g_core) rfuproto::RfuCore();
    new (&g_parser) rfuproto::Rfu1StreamParser();

    g_core.emit = &emitTrampoline;
    g_core.rand16 = &randTrampoline;
    g_core.reset();
    g_core.roleLock = role;  // after reset(): reset() must not clobber the role

    g_parser.setCallback(&frameTrampoline, nullptr);

    ring_buf_reset(&g_rfuOutRing);
    k_sem_reset(&g_rfuNetRxSem);
    g_lastNetRxMs = 0;

    rfuLink_setDoneCallback(&pioDoneCallback, nullptr);

    g_revRx0 = g_revRx1 = 0;
    g_dbgDelivFail = dfNone;
    g_dbgDelivSo = 0;
    g_retriesThisWait = 0;
    g_sdResets = 0;
    g_parkedCount = 0;
    g_stallValid = false;
    g_waitTracked = false;
    g_sessionActive = true;
}

RfuProtocolSection::~RfuProtocolSection()
{
    g_sessionActive = false;

    // The module disables the RFU link before this destructor runs, so no
    // PIO ISR can race the deregistration.
    rfuLink_setDoneCallback(nullptr, nullptr);
}

void RfuProtocolSection::feedNetworkBytes(std::span<const uint8_t> data)
{
    g_parser.push(data.data(), data.size());
}

// Poll the GBA's SO line for a level. The librfu slave ISR's handshake steps
// are level-based spins with no deadline of their own (each word only has to
// land inside its 100.2ms inter-word timer), so a generous-but-bounded poll
// is safe on both sides.
static bool waitGbaSo(bool level, uint32_t timeoutUs)
{
    for (uint32_t t = 0; t < timeoutUs; t += 2)
    {
        if (rfuLink_gbaLineHigh() == level) return true;
        k_busy_wait(2);
    }
    return rfuLink_gbaLineHigh() == level;
}

// Clock one wait-event frame into the GBA-as-slave, word by word, running
// librfu's slave-ISR ready dance between words (sio32intr_clock_slave):
// after each 32-clock word the GBA spins for our SI LOW, raises its SO,
// processes the word, spins for our SI HIGH, then drops SO — re-arming for
// the next word, or (after the trailer) handing the bus back and issuing its
// next command as master within tens of microseconds. Word 1 needs no dance:
// the GBA armed itself right after the WAIT-class ACK (librfu_intr.c:119-124)
// and purely counts clocks.
void RfuProtocolSection::runDelivery()
{
    uint32_t words[8];
    uint8_t n;
    {
        // Snapshot under lock: a soft-resetting GBA can clock a 0x494E through
        // the (still active) slave SM between pollWaitEvent and here, knocking
        // the core back to the ID exchange.
        const unsigned int key = irq_lock();
        if (g_core.comstate != rfuproto::comWaitResp ||
            g_core.eventWordCount() > sizeof(words) / sizeof(words[0]))
        {
            irq_unlock(key);
            g_retriesThisWait = 0;  // wait ended via the soft-reset escape
            return;
        }
        n = g_core.eventWordCount();
        for (uint8_t i = 0; i < n; i++) words[i] = g_core.eventWord(i);
        irq_unlock(key);
    }

    // IMPORTANT (HW test #25, re-confirmed #28): the armed GBA-as-slave does
    // NOT present its 0x80000000 pre-load on SO while idle — after the WAIT
    // ack it arms with SIOCNT SD=0 and SO rests LOW until it is clocked. So
    // there is no observable "armed" level to gate on. Instead: hold the
    // slave role briefly so the command-phase SI mirror finishes serving the
    // GBA's post-ACK handshake_wait(1)/(0) and its arm sequence (a few µs of
    // IRQ time; 150µs is generous), then take the bus.
    k_busy_wait(150);

    {
        // Atomic re-check + takeover: a soft reset (0x494E through the still
        // active slave SM) during the wait above knocks the core back to the
        // ID exchange — the GBA is bus master again and SC must not be taken
        // from it. setRole seeds SI LOW = the inverted-ACK lead-in step 1.
        const unsigned int key = irq_lock();
        if (g_core.comstate != rfuproto::comWaitResp)
        {
            irq_unlock(key);
            g_retriesThisWait = 0;  // wait ended via the soft-reset escape
            return;
        }
        rfuLink_setRole(RFU_ROLE_ADAPTER_MASTER);
        irq_unlock(key);
    }

    // Word-1 inverted-ACK lead-in (afska wireless_adapter.md "Inverted ACKs"):
    // the GBA may answer our SI-low with an SO-high pulse; on this hardware it
    // usually does NOT before word 1 (SO stays low until clocked — test #25),
    // so the poll is bounded and timing out is the normal path.
    if (waitGbaSo(true, 120))
    {
        rfuLink_masterDriveSi(true);     // its handshake_wait(1)
        waitGbaSo(false, 4000);          // it drops SO when ready
        rfuLink_masterDriveSi(false);    // ready to clock (doc step 5)
    }
    k_busy_wait(45);                     // >=40µs settle (anti-desync)

    bool ok = true;
    bool swapped = false;
    bool retriable = false;
    for (uint8_t i = 0; ok && i < n; i++)
    {
        uint32_t rx = 0;
        ok = rfuLink_masterExchange(words[i], &rx, 2000);
        if (!ok)
        {
            g_dbgDelivFail = static_cast<uint8_t>(dfExchangeTimeout | (i << 4));
            g_dbgDelivSo = rfuLink_gbaLineHigh() ? 1 : 0;
            // Word 1 never completed = nothing meaningful reached the GBA;
            // it is still armed — safe to retry after re-arming as slave.
            retriable = (i == 0);
            break;
        }

        g_revRx0 = g_revRx1;
        g_revRx1 = rx;
        g_core.counters.masterTransfers++;

        if (i > 0 && rx == 0x996601EE)
        {
            // The GBA rejected the event command and staged an error response
            // in its place. With well-formed events this never fires; count it
            // and abort — the GBA recovers through its own timer + checkID.
            g_core.counters.errResponses++;
            ok = false;
            break;
        }

        rfuLink_masterDriveSi(false);
        ok = waitGbaSo(true, 4000);          // SO high: word received, processing
        if (!ok)
        {
            g_dbgDelivFail = static_cast<uint8_t>(dfDanceSoHigh | (i << 4));
            g_dbgDelivSo = rfuLink_gbaLineHigh() ? 1 : 0;
            break;
        }
        rfuLink_masterDriveSi(true);         // release its handshake_wait(1)

        if (i + 1 < n)
        {
            // Between words the GBA re-arms with two back-to-back SIOCNT
            // writes under IME=0 (librfu_intr.c:311-328): SO dips low for
            // well under a microsecond and then presents the staged word's
            // MSB (always 1) — there is NO observable "re-armed" level to
            // wait on. Its path from our SI-high to armed is bounded: the
            // TM0-proximity spin (worst ~125us at a 1024 prescale) plus a
            // handful of stores — a fixed 250us more than covers it and
            // subsumes the >=40us inter-transfer settle.
            k_busy_wait(250);
        }
        else
        {
            // Final word: the GBA drops SO and KEEPS it low (state-8 exit,
            // librfu_intr.c:286-309), then re-drives SC as bus master and
            // issues its next command within tens of microseconds. Detect the
            // drop and swap back ATOMICALLY — an ISR between detection and
            // the swap would let that first command clock into a disabled
            // slave SM, bit-slipping every word after (the edge counter never
            // realigns to word boundaries).
            for (uint32_t t = 0; t < 4000 && !swapped; t += 2)
            {
                const unsigned int key = irq_lock();
                if (!rfuLink_gbaLineHigh())
                {
                    g_core.finishDelivery();
                    rfuLink_setRole(RFU_ROLE_GBA_MASTER);
                    rfuLink_pushTx(g_core.stagedTx);
                    swapped = true;
                }
                irq_unlock(key);
                if (!swapped) k_busy_wait(2);
            }
            ok = swapped;
            if (!swapped)
            {
                g_dbgDelivFail = static_cast<uint8_t>(dfFinalSoLow | (i << 4));
                g_dbgDelivSo = rfuLink_gbaLineHigh() ? 1 : 0;
            }
        }
    }

    if (swapped)
    {
        g_dbgDelivFail = dfNone;
        g_retriesThisWait = 0;
    }
    else
    {
        const unsigned int key = irq_lock();
        if (retriable && ++g_retriesThisWait <= kMaxRetriesPerWait)
        {
            g_core.retryDelivery();
        }
        else
        {
            g_core.abortDelivery();
            g_retriesThisWait = 0;
        }
        rfuLink_setRole(RFU_ROLE_GBA_MASTER);
        rfuLink_pushTx(g_core.stagedTx);
        irq_unlock(key);
    }
}

void RfuProtocolSection::process()
{
    while (!m_cancel)
    {
        const uint32_t now = k_uptime_get_32();

        pumpOutbound();

        // The game pulsed SD (AgbRFU_SoftReset): full adapter reset, exactly
        // like gpsp's rfu_reset. This catches resets in EVERY state —
        // including mid-payload, where the 0x494E command-stream escape
        // cannot fire — and precedes the checkID re-run by microseconds.
        if (rfuLink_sdResetSeen())
        {
            const unsigned int key = irq_lock();
            g_core.onSdReset();
            irq_unlock(key);
            g_retriesThisWait = 0;
            g_sdResets = g_sdResets + 1;
        }

        // Parked frames apply once the GBA is between commands (or reversed)
        // — same irq_lock discipline as frameTrampoline.
        if (g_parkedCount > 0 && safeToApplyInbound(g_core.comstate))
        {
            const uint8_t cnt = g_parkedCount;
            g_lastNetRxMs = now ? now : 1;
            for (uint8_t i = 0; i < cnt; i++)
            {
                const unsigned int key = irq_lock();
                g_core.applyNetPacket(g_parked[i].ptype, g_parked[i].hdata,
                                      g_parked[i].payload, g_parked[i].payloadLen, now);
                irq_unlock(key);
            }
            // A frame can park while the loop above runs — compact instead of
            // zeroing so it isn't silently dropped.
            const unsigned int key = irq_lock();
            for (uint8_t i = cnt; i < g_parkedCount; i++)
                g_parked[i - cnt] = g_parked[i];
            g_parkedCount = g_parkedCount - cnt;
            irq_unlock(key);
        }

        // Stall detector: comWaitEvent with the ACK-read still outstanding
        // means the GBA is mid-handshake between its last command transfer
        // and the ACK-read. >250ms there = the observed SEND_DATAW death;
        // snapshot the wires + slave SM once so telemetry shows exactly what
        // both sides were doing (GBA SO level, our SI level, SM PC, FIFOs).
        if (g_core.comstate == rfuproto::comWaitEvent && !g_core.dbgWaitAck())
        {
            if (!g_waitTracked)
            {
                g_waitTracked = true;
                g_waitEnteredMs = now;
            }
            else if (!g_stallValid && (now - g_waitEnteredMs) > 250)
            {
                uint8_t pc, tx, rx, lines;
                rfuLink_debugSnapshot(&pc, &tx, &rx, &lines);
                g_stallPc = pc;
                g_stallTx = tx;
                g_stallRx = rx;
                g_stallLines = lines;
                g_stallLastRx = g_core.dbgLastRx;
                g_stallValid = true;
            }
        }
        else
        {
            g_waitTracked = false;
        }

        // Wait-event evaluation and periodic upkeep mutate state shared
        // with the PIO ISR — lock interrupts for the brief calls.
        const unsigned int key = irq_lock();
        const bool startDelivery = g_core.pollWaitEvent(now);
        g_core.tick(now);
        irq_unlock(key);

        if (startDelivery)
        {
            // Synchronous and bounded (~1ms typical, ~15ms worst on a dead
            // GBA); the command-phase SI=!SO handshake resumes the instant
            // the role swaps back.
            runDelivery();
        }

        superviseLink(now);
        updateDiagnosticLed();
        reportDiagnostics(now);

        k_sem_take(&g_rfuNetRxSem, K_MSEC(1));
    }
}

// Bring-up indicator: localizes a detection failure to the physical layer at
// a glance (see RfuCore's dbg* fields). Remove once the link is proven.
void RfuProtocolSection::updateDiagnosticLed()
{
    enum Diag { dRed, dMagenta, dYellow, dGreen };
    Diag d;
    if (g_core.comstate >= rfuproto::comWaitCmd) d = dGreen;       // detected
    else if (g_core.dbgNintendo)                 d = dYellow;      // bit order ok
    else if (g_core.dbgAnyRx)                     d = dMagenta;    // rx, wrong order
    else                                          d = dRed;        // no clock/data

    if (d == m_lastDiag) return;
    m_lastDiag = d;

    auto& hw = Hardware::getInstance();
    switch (d)
    {
        case dRed:     hw.setLED(40, 0, 0, true);   break;
        case dMagenta: hw.setLED(40, 0, 40, true);  break;
        case dYellow:  hw.setLED(40, 40, 0, true);  break;
        case dGreen:   hw.setLED(0, 40, 0, true);   break;
    }
}

// Bring-up telemetry: surface the raw 32-bit words the GBA actually clocked in,
// so the host tool can decode exactly how a stuck (magenta/yellow) detection is
// mis-aligned (bit-reversed vs half-swapped vs edge-slipped vs garbage). Sent as
// a sub-64-byte data frame (tag 0x0E) → arrives on the client's raw-data channel.
// Throttled; remove with the diagnostic LED once detection is proven.
void RfuProtocolSection::reportDiagnostics(uint32_t nowMs)
{
    if ((nowMs - m_lastDiagReportMs) < 500) return;
    m_lastDiagReportMs = nowMs;

    const uint32_t first = g_core.dbgFirstRx;
    const uint32_t last  = g_core.dbgLastRx;
    const uint32_t xfers = g_core.counters.slaveTransfers;
    const uint8_t diag[16] = {
        0x0E,
        static_cast<uint8_t>(g_core.dbgAnyRx ? 1 : 0),
        static_cast<uint8_t>(g_core.dbgNintendo ? 1 : 0),
        static_cast<uint8_t>(g_core.comstate),
        static_cast<uint8_t>(first),       static_cast<uint8_t>(first >> 8),
        static_cast<uint8_t>(first >> 16), static_cast<uint8_t>(first >> 24),
        static_cast<uint8_t>(last),        static_cast<uint8_t>(last >> 8),
        static_cast<uint8_t>(last >> 16),  static_cast<uint8_t>(last >> 24),
        static_cast<uint8_t>(xfers),       static_cast<uint8_t>(xfers >> 8),
        static_cast<uint8_t>(xfers >> 16), static_cast<uint8_t>(xfers >> 24),
    };
    Transport::sendData(std::span<const uint8_t>(diag, sizeof(diag)));

    // Expanded post-detection frame (tag 0x2E — NOT 0x0F, which would collide
    // with GetFirmwareInfo responses on the same raw-data channel): counters +
    // role + the last-8 command-opcode ring, to localize a WAIT/clock-reversal
    // freeze. All u32 little-endian (matches the 0x0E decoder). KEY
    // discriminators: masterTransfers (reverse words clocked), deliveryAborts,
    // loginRestarts, and the rev[] readback (word-2 == 0x996600A8 => the GBA
    // accepted).
    // MUST stay under 64 bytes: the web client routes exactly-64-byte data
    // transfers to the link-relay path, not the raw/telemetry channel.
    uint8_t f[62] = {};
    f[0] = 0x2E;
    f[1] = static_cast<uint8_t>(g_core.comstate);
    f[2] = static_cast<uint8_t>(g_core.state);
    f[3] = static_cast<uint8_t>(rfuLink_getRole());
    f[4] = static_cast<uint8_t>((g_core.dbgAnyRx     ? 0x01 : 0) |
                                (g_core.dbgNintendo  ? 0x02 : 0) |
                                (g_core.dbgWaitAck() ? 0x04 : 0));
    f[5] = g_core.dbgLastCmd();
    f[6] = g_core.dbgLastPlen();
    const uint8_t cnt  = g_core.dbgCmdRingCount;
    const uint8_t head = g_core.dbgCmdRingHead;
    f[7] = cnt;
    for (uint8_t i = 0; i < 8; i++)  // ordered oldest -> newest
        f[8 + i] = (i < cnt) ? g_core.dbgCmdRing[static_cast<uint8_t>(head - cnt + i) & 7] : 0;
    const uint32_t c32[8] = {
        g_core.counters.slaveTransfers, g_core.counters.masterTransfers,
        g_core.counters.waitEventTransfers, g_core.counters.commands,
        g_core.counters.deliveryAborts, g_core.counters.loginRestarts,
        g_core.counters.errResponses, g_core.dbgLastRx,
    };
    for (int i = 0; i < 8; i++)
    {
        const int o = 16 + i * 4;
        f[o]     = static_cast<uint8_t>(c32[i]);
        f[o + 1] = static_cast<uint8_t>(c32[i] >> 8);
        f[o + 2] = static_cast<uint8_t>(c32[i] >> 16);
        f[o + 3] = static_cast<uint8_t>(c32[i] >> 24);
    }
    // Reverse-phase SO readback: bytes 48-51 = prior word, 52-55 = last word.
    const uint32_t rev[2] = { g_revRx0, g_revRx1 };
    for (int i = 0; i < 2; i++)
    {
        const int o = 48 + i * 4;
        f[o]     = static_cast<uint8_t>(rev[i]);
        f[o + 1] = static_cast<uint8_t>(rev[i] >> 8);
        f[o + 2] = static_cast<uint8_t>(rev[i] >> 16);
        f[o + 3] = static_cast<uint8_t>(rev[i] >> 24);
    }
    // Delivery failure diagnostics: byte 56 = stage|word<<4 of the last
    // failed delivery (0 = last delivery succeeded), byte 57 = GBA SO level
    // at that failure, bytes 58-61 = soft retries (LE).
    f[56] = g_dbgDelivFail;
    f[57] = g_dbgDelivSo;
    const uint32_t rt = g_core.counters.deliveryRetries;
    f[58] = static_cast<uint8_t>(rt);
    f[59] = static_cast<uint8_t>(rt >> 8);
    f[60] = static_cast<uint8_t>(rt >> 16);
    f[61] = static_cast<uint8_t>(rt >> 24);
    Transport::sendData(std::span<const uint8_t>(f, sizeof(f)));

    // Event-class counters (tag 0x1D): how each wait resolved (data /
    // host-rtx / timeo / disc) + the last event header word. A parent's UNI
    // pump seeing anything but data/rtx fails its DRAC-ACK gate. Bytes 15-17
    // carry slot-wipe provenance (which path cleared an occupied client
    // slot, 4-bit saturating) and the live slot-occupancy bitmap.
    const uint32_t lev = g_core.dbgLastEvent;
    const uint32_t lpw = g_core.dbgLastLinkPwr;
    const auto sat4 = [](uint8_t v) -> uint8_t { return v > 15 ? 15 : v; };
    const uint8_t ev[18] = {
        0x1D,
        g_core.dbgEvData, g_core.dbgEvRtx, g_core.dbgEvTimeo, g_core.dbgEvDisc,
        static_cast<uint8_t>(lev),       static_cast<uint8_t>(lev >> 8),
        static_cast<uint8_t>(lev >> 16), static_cast<uint8_t>(lev >> 24),
        g_core.dbgLinkPwrZero,
        static_cast<uint8_t>(lpw),       static_cast<uint8_t>(lpw >> 8),
        static_cast<uint8_t>(lpw >> 16), static_cast<uint8_t>(lpw >> 24),
        g_sdResets,
        static_cast<uint8_t>((sat4(g_core.dbgWipeEvict) << 4) | sat4(g_core.dbgWipeNetDisc)),
        static_cast<uint8_t>((sat4(g_core.dbgWipeHostStart) << 4) | sat4(g_core.dbgWipeCmdDisc)),
        static_cast<uint8_t>((sat4(g_core.dbgWipeReset) << 4) | g_core.dbgSlotOccupancy()),
    };
    Transport::sendData(std::span<const uint8_t>(ev, sizeof(ev)));

    // Wait-stall snapshot (tag 0x2F, 10 bytes): wires + slave SM at the
    // moment a wait-class ACK-read went >250ms overdue. lines bit0 = GBA SO,
    // bit1 = our SI, bit2 = SC, bit3 = adapter-master role.
    if (g_stallValid)
    {
        const uint32_t lrx = g_stallLastRx;
        const uint8_t sp[10] = {
            0x2F, 1, g_stallLines, g_stallPc, g_stallTx, g_stallRx,
            static_cast<uint8_t>(lrx),       static_cast<uint8_t>(lrx >> 8),
            static_cast<uint8_t>(lrx >> 16), static_cast<uint8_t>(lrx >> 24),
        };
        Transport::sendData(std::span<const uint8_t>(sp, sizeof(sp)));
    }
}

void RfuProtocolSection::superviseLink(uint32_t nowMs)
{
    // Status reporting only — the proxy keeps answering the GBA regardless.
    // Skipped until the remote has spoken once.
    const uint32_t lastHeard = g_lastNetRxMs;
    if (lastHeard == 0) return;

    // Signed: g_lastNetRxMs is stamped on the transport thread and can be a
    // hair ahead of this loop pass's nowMs — the unsigned delta underflowed
    // and flapped LinkReconnecting/LinkConnected during frame-heavy windows.
    const bool quiet = static_cast<int32_t>(nowMs - lastHeard) > 4000;
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

void RfuProtocolSection::pumpOutbound()
{
    for (;;)
    {
        if (m_chunkLen < sizeof(m_chunk))
        {
            m_chunkLen += ring_buf_get(&g_rfuOutRing, &m_chunk[m_chunkLen],
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
            // Ring drained mid-chunk; the ring only holds whole frames, so
            // pad to the chunk boundary and flush now (latency over
            // efficiency — broadcasts and ACKs are small).
            std::memset(&m_chunk[m_chunkLen], 0, sizeof(m_chunk) - m_chunkLen);
            if (Transport::sendData(std::span(m_chunk, sizeof(m_chunk))))
                m_chunkLen = 0;
        }
        return;
    }
}
