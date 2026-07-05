#include <cstdint>
#include <cstring>
#include <cstddef>

#pragma once

// GBA Wireless Adapter (RFU, AGB-015) protocol core.
//
// Port of gpsp's working emulation (gpsp/rfu.c). The adapter impersonates
// the wireless adapter to the local GBA — answering every SIO transfer from
// its own state — while broadcasts, connections and game payloads move
// asynchronously over the Celio relay as "RFU1" frames (byte-compatible with
// gpsp's netpackets).
//
// Porting model: gpsp's rfu_transfer(sent) returns the response in the same
// transfer (an emulator luxury). Real hardware preloads its TX word before a
// transfer starts, so this port splits the state machine:
//   consume(rx)  — receive-side transitions (ID-exchange detection, command
//                  header/payload accumulation, command processing)
//   produce()    — emits the NEXT response word and performs the
//                  delivery-side transitions (ACK → data words → idle)
// Command-phase responses are input-independent in gpsp, so this one-transfer
// shift reproduces gpsp's stream exactly. The detection phase (SIO32 ID
// exchange) genuinely cannot be a same-cycle echo on hardware, so it is
// handled specially as a canned sequence — see ID_DANCE.
//
// This header is Zephyr-free so the FSM and framing compile on the host for
// tests. Concurrency contract (enforced by rfuProtocolSection):
//   - onSlaveTransfer() runs in the PIO done-ISR (GBA-master role only).
//   - applyNetPacket(), pollWaitEvent(), tick() and the delivery bookkeeping
//     (eventWord*/finishDelivery/abortDelivery) run under irq_lock() so they
//     never interleave with the ISR.
//   - stagedTx is written by the FSM, read by the PIO ISR push path.

namespace rfuproto
{

constexpr uint32_t BUSY_WORD = 0x80000000;  // adapter idle/ready marker

// Command IDs (gpsp rfu.c:59-96; reverse-engineered, see
// https://github.com/afska/gba-link-connection and blog.kuiper.dev)
constexpr uint8_t CMD_INIT1       = 0x10;
constexpr uint8_t CMD_LINKPWR     = 0x11;
constexpr uint8_t CMD_SYSVER      = 0x12;
constexpr uint8_t CMD_SYSSTAT     = 0x13;
constexpr uint8_t CMD_SLOTSTAT    = 0x14;
constexpr uint8_t CMD_CFGSTAT     = 0x15;
constexpr uint8_t CMD_BCST_DATA   = 0x16;
constexpr uint8_t CMD_SYSCFG      = 0x17;
constexpr uint8_t CMD_HOST_START  = 0x19;
constexpr uint8_t CMD_HOST_ACCEPT = 0x1A;
constexpr uint8_t CMD_HOST_STOP   = 0x1B;
constexpr uint8_t CMD_BCRD_START  = 0x1C;
constexpr uint8_t CMD_BCRD_FETCH  = 0x1D;
constexpr uint8_t CMD_BCRD_STOP   = 0x1E;
constexpr uint8_t CMD_CONNECT     = 0x1F;
constexpr uint8_t CMD_ISCONNECTED = 0x20;
constexpr uint8_t CMD_CONCOMPL    = 0x21;
constexpr uint8_t CMD_SEND_DATA   = 0x24;
constexpr uint8_t CMD_SEND_DATAW  = 0x25;
constexpr uint8_t CMD_RECV_DATA   = 0x26;
constexpr uint8_t CMD_WAIT        = 0x27;
constexpr uint8_t CMD_DISCONNECT  = 0x30;
constexpr uint8_t CMD_WAIT2       = 0x35;  // wait-class alias (librfu ID_UNK35_REQ)
constexpr uint8_t CMD_INIT2       = 0x3D;
constexpr uint8_t CMD_RTX_WAIT    = 0x37;

// Adapter-initiated response commands (delivered with the adapter as master)
constexpr uint8_t CMD_RESP_TIMEO  = 0x27;
constexpr uint8_t CMD_RESP_DATA   = 0x28;
constexpr uint8_t CMD_RESP_DISC   = 0x29;

constexpr uint32_t CONN_INPROGRESS = 0x01000000;
constexpr uint32_t CONN_FAILED     = 0x02000000;
constexpr uint32_t CONN_COMP_FAIL  = 0x01000000;

enum ComState : uint8_t
{
    comIdWait = 0,  // waiting for the GBA to start the SIO32 ID exchange
    comIdDance,     // playing the canned NINTENDO ID-exchange sequence
    comWaitCmd,
    comWaitDat,
    comRespCmd,
    comRespDat,
    comRespErr,
    comRespErr2,
    comWaitEvent,   // waiting for data/timeout; adapter will turn bus master
    comWaitResp     // streaming an adapter-master response frame
};

// SIO32 ID exchange (FRLG AgbRFU_checkID / librfu_sio32id.c). The GBA, as SIO
// master, walks the "NINTENDO" connection words and expects the adapter
// (slave) to answer with a complement dance that converges to RFU_ID. gpsp
// answers it same-cycle with (sent<<16)|~prev — impossible on real hardware,
// which preloads its TX word one transfer ahead. So, like the real adapter
// (and gpsp's canned gbp_seq for the GB Player), we play a FIXED sequence:
// the GBA's NINTENDO word echoed in the high half, the complement of the
// previous word in the low half. Derived and proven against a faithful
// Sio32IDIntr model (tests/host/rfuIdDerive.cpp); converges to 0x8001 at the
// 9th dance word. Played starting the transfer after the GBA's first 0x494E,
// which makes the prefix self-aligning regardless of the GBA's first word.
constexpr uint32_t ID_RFU = 0x00008001;
constexpr uint32_t ID_DANCE[9] = {
    0x494EB6B1, 0x494EB6B1, 0x544EB6B1, 0x544EABB1, 0x4E45ABB1,
    0x4E45B1BA, 0x4F44B1BA, 0x4F44B0BB, 0x8001B0BB,
};
// The GBA's converged word: (recv_id<<16)|RFU_ID with recv_id=~0x4F44=0xB0BB.
// Receiving it means the ID exchange is done and the command phase begins.
constexpr uint32_t ID_DONE_WORD = 0xB0BB8001;

enum RfuState : uint8_t
{
    stIdle = 0,
    stHost,
    stConnecting,
    stClient
};

// "RFU1" network frames (gpsp rfu.c:168-225). Fixed total size per ptype
// makes the stream self-framing across 64-byte transport chunks.
constexpr uint8_t RFU1_MAGIC[4] = { 0x52, 0x46, 0x55, 0x31 };

constexpr uint32_t NET_BROADCAST    = 0x00;  // 36 bytes (6 BE words payload)
constexpr uint32_t NET_CONNECT_REQ  = 0x01;  // 16 bytes
constexpr uint32_t NET_CONNECT_ACK  = 0x02;  // 16 bytes
constexpr uint32_t NET_CONNECT_NACK = 0x03;  // 16 bytes
constexpr uint32_t NET_DISCONNECT   = 0x04;  // 16 bytes
constexpr uint32_t NET_HOST_SEND    = 0x05;  // 104 bytes (92 LE data bytes)
constexpr uint32_t NET_CLIENT_SEND  = 0x06;  // 104 bytes
constexpr uint32_t NET_CLIENT_ACK   = 0x07;  // 16 bytes
// Celio extension (not in gpsp): child→host backpressure. Real adapters
// never need it — both pumps ride the shared RF frame clock — but over the
// relay the two game clocks are independent, and a jitter burst leaves a
// standing backlog in the receiving adapter that the games cannot see or
// drain (consumption is hard-capped at one frame per game frame). The child
// signals when its inbound FIFO is deep; the host adapter then paces its
// game's wait-data events so production dips below the child's consumption
// until the backlog clears. hdata: bit0 = active, bits 8-15 = queue depth.
constexpr uint32_t NET_FLOWCTL      = 0x08;  // 16 bytes

inline int rfu1FrameSize(uint32_t ptype)
{
    switch (ptype)
    {
        case NET_BROADCAST:                      return 36;
        case NET_HOST_SEND: case NET_CLIENT_SEND: return 104;
        case NET_CONNECT_REQ: case NET_CONNECT_ACK:
        case NET_CONNECT_NACK: case NET_DISCONNECT:
        case NET_CLIENT_ACK: case NET_FLOWCTL:   return 16;
        default:                                 return -1;
    }
}

constexpr size_t maxFrameBytes = 104;

inline void pack32be(uint8_t* out, uint32_t v)
{
    out[0] = v >> 24; out[1] = v >> 16; out[2] = v >> 8; out[3] = v;
}
inline uint32_t unpack32be(const uint8_t* p)
{
    return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
}
inline uint32_t unpack32le(const uint8_t* p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}

// ≙ gpsp rfu_net_send_cmd: 16-byte frame, zero pad word
inline size_t rfu1SerializeCmd(uint8_t out[16], uint32_t ptype, uint32_t hdata)
{
    std::memcpy(out, RFU1_MAGIC, 4);
    pack32be(out + 4, ptype);
    pack32be(out + 8, hdata);
    std::memset(out + 12, 0, 4);
    return 16;
}

// ≙ gpsp rfu_net_send_bcast: 36-byte frame, 6 big-endian payload words
inline size_t rfu1SerializeBcast(uint8_t out[36], uint32_t hdata, const uint32_t bdata[6])
{
    std::memcpy(out, RFU1_MAGIC, 4);
    pack32be(out + 4, NET_BROADCAST);
    pack32be(out + 8, hdata);
    for (int i = 0; i < 6; i++)
        pack32be(out + 12 + 4 * i, bdata[i]);
    return 36;
}

// ≙ gpsp rfu_net_send_data: 104-byte frame, words flattened to little-endian
// bytes (the presumed over-the-air format), zero padded to 92
inline size_t rfu1SerializeData(uint8_t out[104], uint32_t ptype, uint32_t hdata,
                                const uint32_t* words, uint8_t byteLen)
{
    std::memcpy(out, RFU1_MAGIC, 4);
    pack32be(out + 4, ptype);
    pack32be(out + 8, hdata);
    for (uint8_t i = 0; i < byteLen; i++)
        out[12 + i] = static_cast<uint8_t>(words[i / 4] >> (8 * (i & 3)));
    std::memset(out + 12 + byteLen, 0, 92 - byteLen);
    return 104;
}

// Reassembles RFU1 frames from the 64-byte transport chunk stream (chunk
// tails are zero padded; zeros never match the magic).
class Rfu1StreamParser
{
public:
    using FrameFn = void(*)(void* ctx, uint32_t ptype, uint32_t hdata,
                            const uint8_t* payload, uint16_t payloadLen);

    void setCallback(FrameFn cb, void* ctx) { m_cb = cb; m_ctx = ctx; }

    void push(const uint8_t* data, size_t len)
    {
        for (size_t i = 0; i < len; i++) pushByte(data[i]);
    }

private:
    void pushByte(uint8_t b)
    {
        if (m_magicIdx < 4)
        {
            if (b == RFU1_MAGIC[m_magicIdx]) m_magicIdx++;
            else m_magicIdx = (b == RFU1_MAGIC[0]) ? 1 : 0;
            return;
        }

        if (m_hdrIdx < 8)
        {
            m_hdr[m_hdrIdx++] = b;
            if (m_hdrIdx == 8)
            {
                m_ptype = unpack32be(&m_hdr[0]);
                m_hdata = unpack32be(&m_hdr[4]);
                const int total = rfu1FrameSize(m_ptype);
                if (total < 0)
                {
                    restart();  // unknown ptype — resync on the next magic
                    return;
                }
                m_payloadLen = static_cast<uint16_t>(total - 12);
                m_payloadIdx = 0;
                if (m_payloadLen == 0) finishFrame();
            }
            return;
        }

        m_payload[m_payloadIdx++] = b;
        if (m_payloadIdx == m_payloadLen) finishFrame();
    }

    void finishFrame()
    {
        if (m_cb) m_cb(m_ctx, m_ptype, m_hdata, m_payload, m_payloadLen);
        restart();
    }

    void restart() { m_magicIdx = 0; m_hdrIdx = 0; }

    FrameFn m_cb = nullptr;
    void* m_ctx = nullptr;

    uint8_t  m_magicIdx = 0;
    uint8_t  m_hdrIdx = 0;
    uint8_t  m_hdr[8] = {};
    uint32_t m_ptype = 0;
    uint32_t m_hdata = 0;
    uint16_t m_payloadLen = 0;
    uint16_t m_payloadIdx = 0;
    uint8_t  m_payload[92] = {};
};

struct RfuCounters
{
    uint32_t slaveTransfers = 0;
    uint32_t masterTransfers = 0;
    uint32_t commands = 0;
    uint32_t errResponses = 0;
    uint32_t waitEventTransfers = 0;  // GBA clocked us while in WAITEVENT
    uint32_t txFrames = 0;
    uint32_t rxFrames = 0;
    uint32_t rxDropQueueFull = 0;
    uint32_t rxDropMalformed = 0;
    uint32_t bcastsSent = 0;
    uint32_t loginRestarts = 0;
    uint32_t deliveryAborts = 0;      // hard abort: wait abandoned mid-frame
    uint32_t deliveryRetries = 0;     // soft abort: nothing on the wire yet, re-poll
};

class RfuCore
{
public:
    // Serialized RFU1 frame ready for the transport (any of the three sizes).
    using EmitFn = void(*)(void* ctx, const uint8_t* frame, size_t len);
    using Rand16Fn = uint16_t(*)(void* ctx);

    EmitFn emit = nullptr;
    void* emitCtx = nullptr;
    Rand16Fn rand16 = nullptr;
    void* randCtx = nullptr;

    RfuCounters counters;

    // Response staged for the next GBA-master transfer (read by the PIO push
    // path; the PIO keeps exactly one word in its TX FIFO).
    volatile uint32_t stagedTx = 0;

    ComState comstate = comIdWait;
    RfuState state = stIdle;

    // Web-session role hint (SetMode variant byte; set ONCE by the section
    // ctor after reset()). Historically this gated broadcasts and outgoing
    // connects to break a theoretical mesh livelock, but FRLG never
    // auto-connects (a player must hail), so every gate has been retired and
    // the Union Room runs fully symmetric — any member may host, scan, and
    // initiate. Kept for the wire protocol (the page still sends it) and as
    // a diagnostic label.
    static constexpr uint8_t ROLE_SYMMETRIC = 0, ROLE_HOST = 1, ROLE_CLIENT = 2;
    uint8_t roleLock = ROLE_SYMMETRIC;

    //-//////////////////////////////////////////////////////////////////////-//
    // GBA-master (slave-role) transfer — PIO done-ISR
    //-//////////////////////////////////////////////////////////////////////-//

    // Bring-up diagnostics (read by the section's LED indicator). These let a
    // glance at the LED localize a detection failure to the physical layer:
    //   - dbgAnyRx false        → the GBA's clock/data never reached us
    //                             (wiring / pin map / SC not detected)
    //   - dbgAnyRx, !dbgNintendo → we receive words but never the 0x494E
    //                             marker → wire bit order is likely wrong
    //   - dbgNintendo, stuck    → bit order is right but we don't converge
    //                             → SC sample-edge (clock polarity) or timing
    //   - reached comWaitCmd     → detection passed
    volatile bool dbgAnyRx = false;
    volatile bool dbgNintendo = false;
    volatile uint32_t dbgFirstRx = 0;
    volatile uint32_t dbgLastRx = 0;

    // Expanded post-detection telemetry (read by the section's 0x0F frame). The
    // command ring captures the opcode sequence into a WAIT/clock-reversal freeze.
    // Accessors expose otherwise-private FSM fields (inline bodies see the whole class).
    uint8_t dbgLastCmd()  const { return m_cmd; }
    uint8_t dbgLastPlen() const { return m_plen; }
    bool    dbgWaitAck()  const { return m_waitAckRead; }
    volatile uint8_t dbgCmdRing[8] = {};
    volatile uint8_t dbgCmdRingHead = 0;   // next write slot (one past newest)
    volatile uint8_t dbgCmdRingCount = 0;
    // Wait-event class counters + the last event header delivered.
    volatile uint8_t dbgEvData = 0, dbgEvRtx = 0, dbgEvTimeo = 0, dbgEvDisc = 0;
    volatile uint32_t dbgLastEvent = 0;
    // Link-strength forensics (the librfu watchLink 4-strike disconnect).
    volatile uint32_t dbgLastLinkPwr = 0;
    volatile uint8_t dbgLinkPwrZero = 0;
    // Slot-wipe provenance: which path last cleared an OCCUPIED host client
    // slot (evict = lastHeard timeout, netDisc = inbound NET_DISCONNECT,
    // hostStart = HOST_START-from-idle, cmdDisc = game's 0x30, reset =
    // resetLinkState). Saturating 4-bit when packed into telemetry.
    volatile uint8_t dbgWipeEvict = 0, dbgWipeNetDisc = 0, dbgWipeHostStart = 0,
                     dbgWipeCmdDisc = 0, dbgWipeReset = 0;
    // Flow-control forensics: backpressure hints sent (child), paced data
    // events while held (host), inbound FIFO high-water since last report,
    // stale-tail sheds (host, 8-seq-frame units).
    volatile uint8_t dbgFlowHints = 0, dbgFlowHolds = 0;
    volatile uint8_t dbgFifoHigh = 0;
    volatile uint8_t dbgSheds = 0;
    volatile uint8_t dbgIdleRetx = 0;  // radio keepalive re-broadcasts (wraps)

    uint8_t dbgSlotOccupancy() const
    {
        return (m_host.clients[0].devid ? 1u : 0) | (m_host.clients[1].devid ? 2u : 0) |
               (m_host.clients[2].devid ? 4u : 0) | (m_host.clients[3].devid ? 8u : 0);
    }

    void onSlaveTransfer(uint32_t rx, uint32_t nowMs)
    {
        counters.slaveTransfers++;
        if (rx != 0)
        {
            if (!dbgAnyRx) dbgFirstRx = rx;
            dbgAnyRx = true;
            dbgLastRx = rx;
        }
        if ((rx & 0xFFFF) == 0x494E) dbgNintendo = true;
        consume(rx, nowMs);
        stagedTx = produce();
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Adapter-master delivery (WAITRESP) — driven synchronously by the
    // section thread (runDelivery), which clocks eventWord(0..count-1) into
    // the GBA with the librfu inter-word handshake between words.
    //-//////////////////////////////////////////////////////////////////////-//

    // True when pollWaitEvent has built a response frame to clock into the GBA.
    bool deliveryPending() const { return comstate == comWaitResp; }

    uint8_t  eventWordCount() const { return m_plen; }
    uint32_t eventWord(uint8_t i) const { return m_buf[i]; }

    // All event words clocked and the final handshake completed — re-arm as
    // GBA-slave for the next command.
    void finishDelivery()
    {
        comstate = comWaitCmd;
        stagedTx = BUSY_WORD;
    }

    // Delivery never started or stalled (GBA not receptive) — give up and
    // return to the command phase, like a real adapter whose wait the game
    // abandoned.
    void abortDelivery()
    {
        counters.deliveryAborts++;
        comstate = comWaitCmd;
        stagedTx = BUSY_WORD;
    }

    // Delivery failed before ANY word reached the wire (pre-check or word-1
    // exchange). The GBA is still armed and waiting — librfu runs no timer
    // before word 1 — so put the event back: pollWaitEvent rebuilds it on the
    // next tick and the delivery retries. Deadlines stay armed; a game-side
    // soft reset (0x494E) still breaks out via restartIdExchange.
    void retryDelivery()
    {
        counters.deliveryRetries++;
        comstate = comWaitEvent;
        stagedTx = BUSY_WORD;
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Wait-event evaluation (≙ gpsp rfu_update, rfu.c:871-929) — section
    // thread, under irq_lock. Returns true when a response frame was built
    // and the section should run the adapter-master delivery. gpsp's "GBA
    // must be in slave mode" SIOCNT gate becomes runDelivery's SO-high
    // pre-check (an armed slave presents its 0x80000000 pre-load MSB).
    //-//////////////////////////////////////////////////////////////////////-//

    bool pollWaitEvent(uint32_t nowMs)
    {
        // comWaitEvent is entered when the WAIT-class ACK is STAGED; the GBA
        // still has to clock one more master transfer to read it (in-band, as
        // the tail of its own exchange — librfu sio32intr_clock_master state 1)
        // before it drops to slave. Only after that transfer (m_waitAckRead,
        // set by consume()) may the adapter take the bus — flipping earlier
        // would discard the staged ACK and drive SC against the GBA. After the
        // ACK read the GBA clocks nothing further, so answer freely. No event
        // is a stall: librfu arms no timer between the WAIT ack and word 1
        // (librfu_intr.c:107,161), so a client with no queued data simply
        // waits for the ~533ms TIMEO, like gpsp.
        if (comstate != comWaitEvent || !m_waitAckRead) return false;

        // Arm the wait deadlines once per WAIT, on the first poll (produce()
        // had no nowMs). The TIMEO / host-rtx windows run from here.
        if (!m_waitArmed)
        {
            m_waitDeadlineMs = nowMs + m_timeoutFrames * 1000u / 60u;
            m_rtxDeadlineMs = nowMs + m_rtxMax * 1000u / 360u;
            m_waitArmed = true;
        }

        if (state == stIdle)
        {
            // Disconnected while waiting
            m_buf[0] = 0x99660000 | (1 << 8) | CMD_RESP_DISC;
            m_buf[1] = 0xF;
            m_buf[2] = BUSY_WORD;
            m_plen = 3;
        }
        // Client data events are PACED to the real adapter's cadence: one
        // per RF frame (~16.7ms). The child's librfu MSC callback enqueues
        // every delivered aggregate into the game's 20-slot recvQueue, which
        // the game's main loop drains at exactly one per frame — and during
        // held-keys streaming the drain path never skips (only all-zero
        // frames are skipped, link_rfu_3.c:573-578). Unpaced, a relay burst
        // resolves waits back-to-back (~2.7ms/cycle), ratchets that queue to
        // its latched-full state and throws the games' FATAL "communication
        // error / power OFF and ON" screen (F_RFU_ERROR_5|6|7). 16ms (vs the
        // 16.74ms frame) leaves ~3Hz of headroom to drain our own inbound
        // FIFO after a burst, well inside the game's HANDLE_RECV_QUEUE flow
        // control envelope.
        //
        // A HOST is normally unpaced (its game consumes via 0x26 polls; its
        // recvQueue is never used) — EXCEPT under child backpressure
        // (NET_FLOWCTL): then its data events are held to flowDrainGapMs so
        // its game's pump — and with it, production — runs at ~half rate
        // while the child drains its standing backlog at full consumption
        // speed. This bounds the inter-game lag that a jitter burst would
        // otherwise turn into a permanent sync offset (one side reaching a
        // battle's end while the other is still seconds in the past).
        else if (dataAvail() && timeReached(nowMs, m_nextDataEvtMs))
        {
            // Under a peer hint, both roles trim to the gentle gap (~9%
            // slower); otherwise clients pace at the RF frame and hosts
            // resolve immediately (their game is the 60Hz cap).
            const bool trimmed = !timeReached(nowMs, m_flowHoldUntilMs);
            if (trimmed)
                m_nextDataEvtMs = nowMs + flowTrimGapMs;
            else if (state == stClient)
                m_nextDataEvtMs = nowMs + dataEventGapMs;
            else
                m_nextDataEvtMs = 0;
            m_buf[0] = 0x99660000 | CMD_RESP_DATA;
            m_buf[1] = BUSY_WORD;
            m_plen = 2;
        }
        // rtx must not preempt paced-out data: with frames pending but the
        // delivery gated, resolving the wait "empty" would defeat the pacing
        // and hand the game a no-data frame it didn't need to see.
        else if (state == stHost && !dataAvail() &&
                 timeReached(nowMs, m_rtxDeadlineMs))
        {
            // The simulated retransmission window elapsed without client
            // data — report "no response" exactly as gpsp does.
            m_buf[0] = 0x99660000 | CMD_RESP_DATA | (1 << 8);
            m_buf[1] = 0x00000F0F;
            m_buf[2] = BUSY_WORD;
            m_plen = 3;
        }
        // No empty-data keepalive here, deliberately (it existed once): a
        // fake per-17ms RESP_DATA unblocks the child's wait WITHOUT peer
        // data, so its pump free-runs on its own clock instead of being
        // paced by the parent's real frames. gpsp's WAIT rendezvous — data
        // event only on actual arrivals — is what keeps the two pumps in
        // lockstep and the FIFOs shallow; a pumping parent sends ~60Hz, so
        // the child's events still come every ~17ms, carrying real frames.
        else if (timeReached(nowMs, m_waitDeadlineMs))
        {
            m_buf[0] = 0x99660000 | CMD_RESP_TIMEO;
            m_buf[1] = BUSY_WORD;
            m_plen = 2;
        }
        else
            return false;

        m_cnt = 0;
        comstate = comWaitResp;
        // Telemetry: which event class resolves each wait — the games' UNI
        // pumps care (a TIMEO mid-pump fails the parent's DRAC-ACK check).
        dbgLastEvent = m_buf[0];
        switch (static_cast<uint8_t>(m_buf[0]))
        {
            case CMD_RESP_DATA:
                if (m_buf[0] & 0x100) dbgEvRtx = dbgEvRtx + 1;
                else                  dbgEvData = dbgEvData + 1;
                break;
            case CMD_RESP_TIMEO: dbgEvTimeo = dbgEvTimeo + 1; break;
            case CMD_RESP_DISC:  dbgEvDisc = dbgEvDisc + 1;  break;
        }
        return true;
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Periodic upkeep (≙ gpsp rfu_frame_update) — section thread, under
    // irq_lock. Broadcast cadence and TTL expiry, translated from frame
    // counts to wall-clock ms.
    //-//////////////////////////////////////////////////////////////////////-//

    void tick(uint32_t nowMs)
    {
        if (comstate == comIdWait) return;

        for (auto& p : m_peers)
            if (p.valid && static_cast<int32_t>(nowMs - p.lastSeenMs) >
                               static_cast<int32_t>(peerTtlMs))
                p.valid = false;

        // Idle retransmission — the real radio's autonomous re-broadcast of
        // the current send buffer, one per RF frame. When a game goes quiet
        // (flash save; standby barriers; scene transitions) a real adapter's
        // carrier keeps delivering its last frame at 60Hz, and BOTH the peer
        // game's link supervision (~5s giveups) and the child's entire pump
        // cadence (MSC per RF frame) are calibrated against that. Without
        // it, a quiet parent drops the child's pump to the 533ms TIMEO
        // crawl — the battle-end standby barrier then takes 5+ seconds on
        // one side and the games diverge into the save/room overlap desync.
        //
        // Direction asymmetry, both halves forced by the games' code:
        //  - HOST re-broadcasts ANY content: the child validates nothing
        //    (librfu overwrites without dedup — real RF hands the child the
        //    parent's re-broadcast as fresh data every frame) and every
        //    consumer is dup-tolerant (level-coded keys, one-shot guards,
        //    idempotent block bitmasks, generation-checked standby).
        //  - CLIENT re-broadcasts only ALL-ZERO frames: a replayed non-zero
        //    child frame repeats its 3-bit seq and the parent's +1-mod-8
        //    check treats consecutive duplicates as link failure. Zeros are
        //    exempt and suffice as carrier/liveness; the parent's pump
        //    self-clocks (rtx) and never needs child data to advance.
        if (m_txDirty)
        {
            m_lastUniTxMs = nowMs;
            m_carrierOn = false;
            m_txDirty = false;
        }
        // NOT gated on m_txBuf.blen: the games issue ZERO-LENGTH sends
        // ("nothing to say") during the win/save screens — one of those
        // used to disarm the carrier exactly when the peer depended on it,
        // dropping the peer's pump to the 533ms crawl and firing its ~6s
        // giveup (test #57). blen=0 falls through to the natural-size zero
        // frame in emitUniCarrier.
        else if ((state == stClient || state == stHost) &&
                 (nowMs - m_lastUniTxMs) >=
                     (m_carrierOn ? idleRetxGapMs : idleRetxStartMs))
        {
            // ENGAGE LATE, RUN AT RF CADENCE: the threshold to start the
            // carrier is several frame periods, because at one frame period
            // it RACES the live 59.7Hz stream — any send landing a hair
            // late gets a zero frame slipped in ahead of it, the zeros eat
            // the receiver's paced delivery slots, real throughput halves
            // and the games grind to a crawl (test #55). Active streams
            // never gap 60ms; genuine quiets (standby barrier, saves) are
            // picked up after 60ms — still ~9x faster than the 533ms TIMEO
            // crawl this exists to prevent.
            m_carrierOn = true;
            // CADENCE WITHOUT CONTENT: when the buffer is non-zero, a zero
            // frame of the same shape is synthesized instead of repeating
            // it. Repeating content is how the real radio does it, but our
            // transport is lossless — nobody needs the repetition — and
            // repeated non-zero frames are actively fatal here: they DO
            // enqueue into the peer game's 20-slot recvQueue (only all-zero
            // aggregates are skipped), they hold the room-entry gate
            // (recvQueue<=2) open forever, and a parent quiet mid-block
            // re-broadcasting an 0x89 chunk at 60Hz ratchets the child's
            // game straight into its latched-full reboot fatal (test #54).
            // All-zero frames give the peer everything it actually needs:
            // the child's pump clock (MSC per RF frame) and liveness.
            emitUniCarrier();
            dbgIdleRetx = dbgIdleRetx + 1;
            m_lastUniTxMs = nowMs;
        }

        // GENTLE symmetric backpressure (v2: the 2.4.5 brake halved the
        // peer's pump — 33ms events — and split the games by ~10s at battle
        // end; retired in 2.4.13). Standing inbound queues (10-16 frames of
        // REAL content that never drains — consumption is capped at the
        // game's frame rate) stretch the games' echo round-trip ~10x, and
        // that stretch lands the final block-send's echo-verify callback
        // inside the battle-end Rfu_SetLinkStandbyCallback window — which
        // SILENTLY SKIPS arming when gRfu.callback is busy (link_rfu_2.c:
        // 1595-1601): the skipper races to win/lose+save+room while its
        // partner waits black at the barrier for a ready-signal that only
        // comes at the skipper's NEXT standby. The trim here paces the
        // AHEAD side ~9% (16→18ms), draining a 16-frame backlog in ~4s of
        // ordinary play — imperceptible, safe during block exchanges, and
        // symmetric (both roles hint, both roles brake).
        {
            const uint8_t depth = (state == stClient)
                                      ? m_client.pkt.count
                                      : maxHostSlotDepth();
            if (!m_flowActive && depth >= flowHighWater)
            {
                m_flowActive = true;
                m_lastFlowHintMs = nowMs;
                dbgFlowHints = dbgFlowHints + 1;
                emitCmd(NET_FLOWCTL, 1u | (static_cast<uint32_t>(depth) << 8));
            }
            else if (m_flowActive && depth <= flowLowWater)
            {
                m_flowActive = false;
                emitCmd(NET_FLOWCTL, 0);
            }
            else if (m_flowActive && (nowMs - m_lastFlowHintMs) >= flowRehintMs)
            {
                m_lastFlowHintMs = nowMs;
                emitCmd(NET_FLOWCTL, 1u | (static_cast<uint32_t>(depth) << 8));
            }
        }

        if (state == stConnecting)
        {
            // The RF-retry analog: keep re-requesting across the target's
            // advertise/scan cycling for roughly the game's connect window
            // (~8 x 300ms ≈ 2.4s), then report failure via ISCONNECTED.
            if (m_connectLastMs == 0)
                m_connectLastMs = nowMs;
            else if ((nowMs - m_connectLastMs) >= connectRetryMs)
            {
                if (++m_connectRetries > connectRetryMax)
                {
                    state = stIdle;
                }
                else
                {
                    m_connectLastMs = nowMs;
                    emitCmd(NET_CONNECT_REQ, m_connectTarget);
                }
            }
        }

        if (state == stHost)
        {
            // Only an OPEN room advertises (EndHost stops the beacon while
            // keeping the connected clients alive). The Union Room is a
            // symmetric presence mesh: every member broadcasts while open so
            // each renders the others. The beacon carries joinability in
            // hdata bits 16-23 (next free slot, 0xFF = full) so scanners can
            // gate their hails like they would on a real adapter.
            if (m_host.open &&
                (m_host.bcastNow || (nowMs - m_host.lastBcastMs) >= bcastPeriodMs))
            {
                m_host.bcastNow = false;
                m_host.lastBcastMs = nowMs;
                uint8_t frame[36];
                emitFrame(frame, rfu1SerializeBcast(
                    frame, m_host.devid | (static_cast<uint32_t>(nextFreeSlot()) << 16),
                    m_host.bdata));
                counters.bcastsSent++;
            }

            for (auto& c : m_host.clients)
            {
                // Signed delta: lastHeardMs is stamped from transport context
                // with its own uptime read, which can be a hair AHEAD of this
                // thread's nowMs — unsigned subtraction would underflow to
                // ~2^32 and evict a live client instantly.
                if (c.devid && static_cast<int32_t>(nowMs - c.lastHeardMs) >
                                   static_cast<int32_t>(clientTimeoutMs))
                {
                    c = HostClient{};
                    dbgWipeEvict = dbgWipeEvict + 1;
                    m_host.bcastNow = true;  // joinability changed
                }
            }
        }
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Network frame from the remote adapters (≙ gpsp rfu_net_receive,
    // rfu.c:718-868) — transport context, under irq_lock.
    // gpsp addresses peers by transport id; the Celio relay instead routes
    // frames per ptype (CONNECT_REQ to the devid's owner, ACK/NACK back to
    // the requester, HOST_SEND to the host's clients, CLIENT_* to the host),
    // so identity here is purely the devid each frame carries. Up to 4 remote
    // peers (a 5-member room) are tracked, keyed by devid.
    //-//////////////////////////////////////////////////////////////////////-//

    void applyNetPacket(uint32_t ptype, uint32_t hdata, const uint8_t* payload,
                        uint16_t plen, uint32_t nowMs)
    {
        counters.rxFrames++;

        switch (ptype)
        {
            case NET_BROADCAST:
            {
                // Everyone learns every broadcaster so each member renders
                // the others in the room (mutual visibility). Keyed by devid;
                // a full table evicts the stalest entry (TTL expiry in tick()
                // usually beats this).
                const uint16_t devid = hdata & 0xFFFF;
                PeerBcast* slot = nullptr;
                for (auto& p : m_peers)
                    if (p.valid && p.devid == devid) { slot = &p; break; }
                if (!slot)
                    for (auto& p : m_peers)
                        if (!p.valid) { slot = &p; break; }
                if (!slot)
                {
                    slot = &m_peers[0];
                    for (auto& p : m_peers)
                        if (static_cast<int32_t>(slot->lastSeenMs - p.lastSeenMs) > 0)
                            slot = &p;
                }
                slot->valid = true;
                slot->lastSeenMs = nowMs;
                slot->devid = devid;
                slot->nextSlot = static_cast<uint8_t>(hdata >> 16);
                for (int j = 0; j < 6; j++)
                    slot->data[j] = unpack32be(&payload[j * 4]);
                break;
            }

            case NET_CONNECT_REQ:
                // The relay routes a CONNECT_REQ to the member whose broadcast
                // devid it targets and serializes concurrent requests, so a
                // request landing here is for us and the next ACK/NACK we emit
                // is routed back to that requester. Only an OPEN room accepts
                // (EndHost closes it; a real adapter refuses connects then).
                if (state == stHost && m_host.open)
                {
                    for (unsigned i = 0; i < m_maxClients; i++)
                    {
                        if (!m_host.clients[i].devid)
                        {
                            const uint16_t newid = newDevid();
                            m_host.clients[i].devid = newid;
                            m_host.clients[i].lastHeardMs = nowMs;
                            m_host.bcastNow = true;  // joinability changed
                            emitCmd(NET_CONNECT_ACK, newid | (i << 16));
                            return;
                        }
                    }
                    emitCmd(NET_CONNECT_NACK, 0);
                }
                else
                    emitCmd(NET_CONNECT_NACK, 0);
                break;

            case NET_CONNECT_ACK:
                if (state == stConnecting)
                {
                    m_client = ClientState{};
                    m_client.devid = hdata & 0xFFFF;
                    m_client.clnum = (hdata >> 16) & 0x3;
                    state = stClient;
                }
                break;

            case NET_CONNECT_NACK:
                // Stay in stConnecting: a NACK usually just means the target
                // was mid-scan (the Union Room alternates advertise/scan every
                // second). A real adapter's RF layer retries the connect for
                // the game's whole connect window; tick() re-emits until the
                // retry budget runs out, then ISCONNECTED reports failure.
                break;

            case NET_DISCONNECT:
                if (state == stHost)
                {
                    const unsigned clnum = (hdata >> 16) & 0x3;
                    if (m_host.clients[clnum].devid == (hdata & 0xFFFF))
                    {
                        m_host.clients[clnum] = HostClient{};
                        dbgWipeNetDisc = dbgWipeNetDisc + 1;
                    }
                }
                else if (state == stClient)
                {
                    // Only our own disconnect: the host addresses a specific
                    // client (devid | clnum<<16); with more members in the room
                    // another client's disconnect must not tear us down.
                    if ((hdata & 0xFFFF) == m_client.devid)
                    {
                        m_client = ClientState{};
                        state = stIdle;
                    }
                }
                break;

            case NET_HOST_SEND:
                if (state == stClient)
                {
                    const uint32_t blen = hdata & 0x7F;
                    if (plen >= blen)
                    {
                        // ACK so the host knows we are alive
                        emitCmd(NET_CLIENT_ACK, m_client.devid | (m_client.clnum << 16));
                        // An ALL-ZERO frame (idle stream or carrier) only
                        // exists to tick our GBA's pump — one pending frame
                        // does that; stacking them is pure harm. During a
                        // save the game drains nothing while the peer's
                        // carrier runs at 60Hz: zeros pegged the FIFO at
                        // 32/32 and the peer's REAL end-of-battle frames
                        // tail-dropped behind them ("one GBA misses the
                        // end game sequence", test #58). Enqueue a zero
                        // only when the queue is empty; real frames still
                        // queue in full order.
                        {
                            bool allZero = true;
                            for (uint32_t zi = 0; zi < blen && allZero; zi++)
                                if (payload[zi]) allZero = false;
                            if (allZero && m_client.pkt.pending())
                                break;
                        }
                        m_client.pkt.push(payload, static_cast<uint8_t>(blen),
                                          counters.rxDropQueueFull);
                        if (m_client.pkt.count > dbgFifoHigh)
                            dbgFifoHigh = m_client.pkt.count;
                    }
                }
                break;

            case NET_CLIENT_SEND:
                if (state == stHost)
                {
                    const uint16_t cdevid = hdata & 0xFFFF;
                    const unsigned clid = (hdata >> 16) & 0x3;
                    const uint32_t blen = hdata >> 24;

                    if (m_host.clients[clid].devid == cdevid && blen <= 16 && plen >= blen)
                    {
                        m_host.clients[clid].lastHeardMs = nowMs;
                        // FULLY-zero child frames are pure no-ops to the
                        // parent's game (an empty poll reads identically,
                        // and the game's own enqueue discards all-zero
                        // aggregates) — but queued they are dead weight:
                        // a screen-transition burst of zeros stood 13 deep
                        // for a whole end-of-battle sequence, adding ~220ms
                        // of lag the shed couldn't touch (zeros carry no
                        // seq steps). Don't queue them; liveness is
                        // lastHeardMs, above. The check must be the WHOLE
                        // payload: byte0 alone can carry meaning — the LL
                        // comm=0 close frame is 80 00 (a byte1-only check
                        // ate it and stalled every contact at ENDING), and
                        // a zero-cmd frame with an ack nibble in byte0
                        // advances the parent's block sends.
                        {
                            bool allZero = true;
                            for (uint32_t zi = 0; zi < blen && allZero; zi++)
                                if (payload[zi]) allZero = false;
                            if (allZero)
                                break;
                        }
                        m_host.clients[clid].pkt.push(payload,
                                                      static_cast<uint8_t>(blen),
                                                      counters.rxDropQueueFull);
                        if (m_host.clients[clid].pkt.count > dbgFifoHigh)
                            dbgFifoHigh = m_host.clients[clid].pkt.count;
                        shedStaleTail(m_host.clients[clid].pkt);
                    }
                }
                break;

            case NET_CLIENT_ACK:
                if (state == stHost)
                {
                    const unsigned clid = (hdata >> 16) & 0x3;
                    if (m_host.clients[clid].devid == (hdata & 0xFFFF))
                        m_host.clients[clid].lastHeardMs = nowMs;
                }
                break;

            case NET_FLOWCTL:
                // The peer's inbound queue is standing deep — WE are the
                // ahead side. Apply the gentle trim (see tick()); expires
                // so a lost clear can't stick, peer re-hints while deep.
                m_flowHoldUntilMs = (hdata & 1) ? nowMs + flowHoldMs : nowMs;
                dbgFlowHolds = dbgFlowHolds + 1;
                break;

            default:
                counters.rxDropMalformed++;
                break;
        }
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // The game pulsed the SD line (AgbRFU_SoftReset): everything resets,
    // including a command mid-parse — the GBA re-runs checkID next. Called
    // from the section thread under irq_lock.
    void onSdReset()
    {
        restartIdExchange();
        m_cnt = 0;
        m_plen = 0;
        m_waitArmed = false;
        m_waitAckRead = false;
        stagedTx = 0;  // comIdWait answers 0 until the GBA sends 0x494E
    }

    // Reset (≙ gpsp rfu_reset) — SD-pulse reset (onSdReset) plus a fresh
    // session start.
    //-//////////////////////////////////////////////////////////////////////-//

    void reset()
    {
        m_lastRx = 0;
        m_danceIdx = 0;
        m_cnt = 0;
        state = stIdle;
        comstate = comIdWait;
        m_timeoutFrames = defTimeoutFrames;
        m_rtxMax = defRtxMax;
        m_maxClients = 4;
        m_syscfg = (defRtxMax << 8) | defTimeoutFrames;
        m_waitArmed = false;
        m_waitAckRead = false;
        m_nextDataEvtMs = 0;
        m_flowActive = false;
        m_flowHoldUntilMs = 0;
        m_host = HostState{};
        m_client = ClientState{};
        for (auto& p : m_peers) p = PeerBcast{};
        stagedTx = 0;  // comIdWait answers 0 until the GBA sends 0x494E
    }

private:
    // gpsp config defaults (rfu.c:33-34) and frame→ms translations
    static constexpr uint8_t  defTimeoutFrames = 32;
    static constexpr uint8_t  defRtxMax = 4;
    static constexpr uint32_t bcastPeriodMs = 500;    // 30 frames
    static constexpr uint32_t peerTtlMs = 4250;       // 255 frames
    static constexpr uint32_t clientTimeoutMs = 4000; // 240 frames
    static constexpr uint32_t connectRetryMs = 300;   // RF connect-retry cadence
    static constexpr uint8_t  connectRetryMax = 8;    // ≈ the game's connect window
    static constexpr uint32_t dataEventGapMs = 16;    // ≈ one RF frame (16.74ms)
    // Backpressure: child hints at highWater, clears at lowWater; the host
    // paces data events to flowDrainGapMs while held (child keeps consuming
    // at 60Hz → backlog drains ~30/s). Hold auto-expires so a lost clear
    // can't stick; the child re-hints while still deep.
    static constexpr uint8_t  flowHighWater = 6;
    static constexpr uint8_t  flowLowWater = 2;
    static constexpr uint32_t flowRehintMs = 150;
    static constexpr uint32_t flowHoldMs = 300;
    static constexpr uint32_t flowTrimGapMs = 18;     // gentle ~9% trim
    // Host-side stale-tail shed: at >= this many seq-carrying child frames
    // queued for one slot, drop the oldest 8 (one full mod-8 seq wrap).
    static constexpr uint8_t  hostShedAtSeqFrames = 10;
    static constexpr uint32_t idleRetxGapMs = 17;     // radio re-broadcast cadence
    static constexpr uint32_t idleRetxStartMs = 60;   // quiet threshold to engage

    // Inbound packet FIFO (≙ gpsp's pkts[4], rfu.c:138-153). The games' UNI
    // command/block protocols (chat exit, trade data) assume the lossless
    // shared RF frame clock: every distinct frame must reach the reader
    // exactly once, in order. A latest-wins buffer silently skips a frame
    // whenever two arrive between the reader's polls — fatal for one-shot
    // commands. gpsp: front-pop on RECV, tail-drop on overflow (rfu.c:827,
    // 851), zero-length return when starved, no dedup/no coalescing — the
    // WAIT rendezvous paces the two pumps so the queue stays shallow. Depth
    // 8 (vs gpsp's 4) buys headroom for relay jitter bursts.
    template <uint8_t MAXLEN, uint8_t DEPTH>
    struct PktQueue
    {
        uint8_t q[DEPTH][MAXLEN];
        uint8_t qlen[DEPTH];
        uint8_t head = 0, count = 0;

        void push(const uint8_t* p, uint8_t len, volatile uint32_t& dropCounter)
        {
            if (!len) return;  // len 0 marks an empty slot, as in gpsp
            if (count == DEPTH)
            {
                dropCounter = dropCounter + 1;  // tail-drop, like gpsp
                return;
            }
            const uint8_t slot = (head + count) % DEPTH;
            std::memcpy(q[slot], p, len);
            qlen[slot] = len;
            count++;
        }

        bool pending() const { return count != 0; }

        const uint8_t* peek(uint8_t i) const { return q[(head + i) % DEPTH]; }
        uint8_t peekLen(uint8_t i) const { return qlen[(head + i) % DEPTH]; }

        void dropFront(uint8_t n)
        {
            if (n > count) n = count;
            head = (head + n) % DEPTH;
            count -= n;
        }

        // Pop the front frame; 0 = queue empty (gpsp returns a zero
        // byte-count header then, never a stale copy). The returned pointer
        // stays valid until the next push — callers copy it out within the
        // same irq_lock/ISR section.
        uint8_t read(const uint8_t** out)
        {
            if (!count) return 0;
            *out = q[head];
            const uint8_t len = qlen[head];
            head = (head + 1) % DEPTH;
            count--;
            return len;
        }
    };

    struct HostClient
    {
        uint16_t devid = 0;          // 0 = empty slot
        uint32_t lastHeardMs = 0;
        // Depth 32: must absorb browser-side stall-then-burst delivery
        // (~300ms tab pauses ≈ 18 frames) without dropping — a dropped
        // nonzero child frame is a seq gap the parent's game never forgives.
        PktQueue<16, 32> pkt = {};
    };
    struct HostState
    {
        uint16_t devid = 0;
        // Open = advertising + accepting joiners. EndHost (0x1B) CLOSES the
        // room: broadcast stops and new connects are refused, but existing
        // clients stay linked — the standard FRLG flow right after a join.
        bool open = false;
        bool bcastNow = false;
        uint32_t lastBcastMs = 0;
        uint32_t bdata[6] = {};
        HostClient clients[4];
    };
    struct ClientState
    {
        uint16_t devid = 0;
        uint8_t clnum = 0;
        // Depth 32, drained at the paced ~62Hz — rides out ~500ms of
        // upstream jitter; a dropped host frame can hang a block transfer
        // (chunks are sent exactly once, no re-request on the child).
        PktQueue<128, 32> pkt = {};
    };
    struct PeerBcast
    {
        bool valid = false;
        uint32_t lastSeenMs = 0;
        uint16_t devid = 0;
        uint8_t nextSlot = 0;  // joinability from the beacon (0xFF = full/closed)
        uint32_t data[6] = {};
    };

    HostState m_host;
    ClientState m_client;
    PeerBcast m_peers[4];

    uint32_t m_buf[255] = {};
    struct { uint32_t buf[23]; uint8_t blen; } m_txBuf = {};
    uint8_t m_cmd = 0;
    uint8_t m_plen = 0;
    uint16_t m_cnt = 0;
    uint32_t m_errCode = 0;

    uint8_t m_timeoutFrames = defTimeoutFrames;
    uint8_t m_rtxMax = defRtxMax;
    // SYSCFG bits 16-17 encode the room size (0=5 players .. 3=2 players) as a
    // client-slot cap; clients always send 0 there, so the default stays 4.
    uint8_t m_maxClients = 4;
    uint32_t m_syscfg = (defRtxMax << 8) | defTimeoutFrames;  // raw 0x17 payload (CFGSTAT echoes it)
    uint32_t m_waitDeadlineMs = 0;
    uint32_t m_rtxDeadlineMs = 0;
    uint32_t m_nextDataEvtMs = 0;   // data-event pacer (client: RF frame; host: drain hold)
    // Backpressure state: child side (hint emission) + host side (hold).
    bool m_flowActive = false;
    uint32_t m_lastFlowHintMs = 0;
    uint32_t m_flowHoldUntilMs = 0;
    // Idle-retransmission state (the radio's autonomous re-broadcast).
    bool m_txDirty = false;
    bool m_carrierOn = false;
    uint32_t m_lastUniTxMs = 0;
    // Latches once the wait deadlines have been armed for the current
    // comWaitEvent (armed lazily on the first pollWaitEvent, which has nowMs).
    bool m_waitArmed = false;
    // Latches when the GBA clocks the staged WAIT-class ACK out (the one
    // in-band read before it drops to slave) — the delivery gate.
    bool m_waitAckRead = false;

    // Connect-retry state (the RF layer's transparent retrying, emulated)
    uint16_t m_connectTarget = 0;
    uint32_t m_connectLastMs = 0;
    uint8_t m_connectRetries = 0;

    uint32_t m_lastRx = 0;
    uint8_t m_danceIdx = 0;       // index into ID_DANCE during comIdDance

    static bool timeReached(uint32_t nowMs, uint32_t deadlineMs)
    {
        return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
    }

    uint16_t newDevid()
    {
        // ≙ gpsp new_devid: any non-zero 16-bit value
        for (;;)
        {
            const uint16_t n = rand16 ? rand16(randCtx) : 0xBEEF;
            if (n) return n;
        }
    }

    void emitFrame(const uint8_t* frame, size_t len)
    {
        if (emit) emit(emitCtx, frame, len);
        counters.txFrames++;
    }

    void emitCmd(uint32_t ptype, uint32_t hdata)
    {
        uint8_t frame[16];
        emitFrame(frame, rfu1SerializeCmd(frame, ptype, hdata));
    }

    void emitData(uint32_t ptype, uint32_t hdata, const uint32_t* words, uint8_t byteLen)
    {
        uint8_t frame[104];
        emitFrame(frame, rfu1SerializeData(frame, ptype, hdata, words, byteLen));
    }

    // Joinability byte for the beacon / broadcast-read metadata word: the
    // index of the next free client slot, or 0xFF when the room is full or
    // closed (post-EndHost) — the game refuses to hail 0xFF rooms.
    uint8_t nextFreeSlot() const
    {
        if (state != stHost || !m_host.open) return 0xFF;
        for (unsigned i = 0; i < m_maxClients; i++)
            if (!m_host.clients[i].devid)
                return static_cast<uint8_t>(i);
        return 0xFF;
    }

    uint8_t maxHostSlotDepth() const
    {
        uint8_t d = 0;
        for (const auto& c : m_host.clients)
            if (c.devid && c.pkt.count > d)
                d = c.pkt.count;
        return d;
    }

    bool dataAvail() const
    {
        // ≙ gpsp rfu_data_avail — level-triggered on a non-empty front slot
        if (state == stClient)
            return m_client.pkt.pending();
        if (state == stHost)
        {
            for (const auto& c : m_host.clients)
                if (c.devid && c.pkt.pending())
                    return true;
        }
        return false;
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Receive-side transitions (the listening half of gpsp rfu_transfer)
    //-//////////////////////////////////////////////////////////////////////-//

    void consume(uint32_t rx, uint32_t nowMs)
    {
        (void)nowMs;  // kept for parity with gpsp's clocked receive path
        switch (comstate)
        {
            case comIdWait:
                // Send zeros until the GBA transmits its first NINTENDO word;
                // then start the canned dance on the next staged transfer.
                if ((rx & 0xFFFF) == 0x494E)
                {
                    comstate = comIdDance;
                    m_danceIdx = 0;
                }
                break;

            case comIdDance:
                // The GBA's converged word ends the ID exchange; the command
                // phase begins on the next transfer.
                if (rx == ID_DONE_WORD || m_danceIdx >= 9)
                {
                    comstate = comWaitCmd;
                }
                break;

            case comWaitCmd:
                if ((rx >> 16) == 0x9966)
                {
                    m_plen = static_cast<uint8_t>(rx >> 8);
                    m_cmd = static_cast<uint8_t>(rx);
                    m_cnt = 0;
                    if (!m_plen)
                        finishCommand();
                    else
                        comstate = comWaitDat;
                }
                else if ((rx & 0xFFFF) == 0x494E)
                {
                    // The game re-ran AgbRFU_checkID (soft reset without a
                    // GPIO pulse): restart the ID exchange. A real 0x9966xx4E
                    // command header is consumed by the branch above first.
                    counters.loginRestarts++;
                    restartIdExchange();
                }
                break;

            case comWaitDat:
                m_buf[m_cnt++] = rx;
                if (m_cnt == m_plen)
                {
                    m_cnt = 0;
                    finishCommand();
                }
                break;

            case comWaitEvent:
                // Exactly ONE GBA-master transfer lands here on the happy path:
                // the in-band read of the staged WAIT-class ACK, after which
                // the GBA drops to slave and waits to be clocked. It opens the
                // delivery gate. Anything else is a mid-wait soft reset
                // (AgbRFU_checkID, 0x494E without a 0x9966 header), which
                // restarts the ID exchange.
                if ((rx & 0xFFFF) == 0x494E && (rx >> 16) != 0x9966)
                {
                    counters.loginRestarts++;
                    restartIdExchange();
                }
                else
                {
                    counters.waitEventTransfers++;
                    m_waitAckRead = true;
                }
                break;

            case comWaitResp:
                // The GBA is not expected to clock us here (it is the bus
                // slave) — ≙ gpsp's no-op branch, plus the restart escape.
                if ((rx & 0xFFFF) == 0x494E && (rx >> 16) != 0x9966)
                {
                    counters.loginRestarts++;
                    restartIdExchange();
                }
                else
                    counters.waitEventTransfers++;
                break;

            // Response-delivery states ignore the incoming word (the GBA
            // clocks dummy values while reading our data) — ≙ gpsp's
            // "disregard the input value".
            default:
                break;
        }

        m_lastRx = rx;
    }

    // Host-side stale-tail shed. During held-keys streaming the parent's
    // game FREEWHEELS past missing child data (the 11ms rtx event advances
    // it with an empty slot), so once a child falls behind, its late
    // catch-up frames land at a parent that is already past them — and with
    // both games' link pumps capped at one step per frame, that tail can
    // never drain: it becomes a permanent child->parent lag (one side
    // finishing a battle seconds before the other). THOSE frames are dead
    // and sheddable in units of EXACTLY 8 seq STEPS, which is invisible to
    // the parent game's 3-bit +1-mod-8 check (seq +9 ≡ +1).
    //
    // Two hard constraints, both learned on hardware:
    //  - CONTENT: only held-keys frames with empty/dpad codes freewheel.
    //    Block chunks (0x89 etc.) are data the parent's game actively
    //    WAITS on (GetBlockReceivedStatus) — a deep queue of those is
    //    required backlog; shedding 8 mid-battle chunks stalls the block
    //    FSM into its ~1s link error. Any non-freewheelable frame in the
    //    window vetoes the shed.
    //  - STEPS, not frames: the child's backup-queue re-sends duplicate
    //    frames with an UNCHANGED seq whenever our delivery briefly
    //    outruns its game, so 8 frames != 8 seq steps; dups are dropped
    //    alongside their step without counting.
    // Zero-cmd frames don't participate in the sequence (link_rfu_2.c:
    // 876-907 gates on a nonzero cmd byte) and shed alongside freely.
    void shedStaleTail(PktQueue<16, 32>& q)
    {
        const auto seqCarrying = [&](uint8_t i) {
            return q.peekLen(i) >= 2 && q.peek(i)[1] != 0;
        };
        const auto freewheelable = [&](uint8_t i) {
            if (q.peekLen(i) < 3) return false;
            const uint8_t* p = q.peek(i);
            if (p[1] != 0xBE) return false;  // RFUCMD_SEND_HELD_KEYS only
            const uint8_t key = p[2];        // LINK_KEY_CODE_*: empty/dpad
            return key == 0 || (key >= 0x11 && key <= 0x15);
        };

        // Total distinct seq steps queued — the real standing depth.
        int last = -1;
        uint8_t total = 0;
        for (uint8_t i = 0; i < q.count; i++)
        {
            if (!seqCarrying(i)) continue;
            const int s = q.peek(i)[0] >> 5;
            if (s != last) { total++; last = s; }
        }
        if (total < hostShedAtSeqFrames) return;

        // Find the first frame of step 9 (the drop boundary), vetoing if
        // any seq-carrying frame inside the 8-step window is not dead.
        last = -1;
        uint8_t steps = 0;
        for (uint8_t i = 0; i < q.count; i++)
        {
            if (!seqCarrying(i)) continue;
            const int s = q.peek(i)[0] >> 5;
            if (s != last)
            {
                if (steps == 8)
                {
                    q.dropFront(i);
                    dbgSheds = dbgSheds + 1;
                    return;
                }
                steps++;
                last = s;
            }
            if (!freewheelable(i)) return;
        }
    }

    // Emit an all-zero carrier frame: the current buffer's shape, or the
    // mode's natural frame size when the game's last send was zero-length.
    // The content is ALWAYS zeros — never a repeat of live data (a repeated
    // child seq trips the parent's dup detection; repeated non-zero
    // aggregates ratchet the child game's 20-slot recvQueue).
    void emitUniCarrier()
    {
        static constexpr uint32_t zeros[23] = {};
        if (state == stHost)
        {
            uint32_t blen = m_txBuf.blen ? m_txBuf.blen : 70u;
            if (blen > 90) blen = 90;
            for (const auto& c : m_host.clients)
                if (c.devid)
                {
                    emitData(NET_HOST_SEND, blen, zeros, blen);
                    break;
                }
        }
        else if (state == stClient)
        {
            uint32_t blen = m_txBuf.blen ? m_txBuf.blen : 14u;
            if (blen > 16) blen = 16;
            emitData(NET_CLIENT_SEND,
                     (blen << 24) | (m_client.clnum << 16) | m_client.devid,
                     zeros, blen);
        }
    }

    // Emit the current UNI staging buffer to the peer(s) (≙ the RF frame
    // carrying the send buffer). Used by the SEND commands and by the idle
    // retransmitter in tick().
    void emitUniTx()
    {
        if (state == stHost)
        {
            // One frame regardless of client count — the relay fans it
            // out to every connected client (the real adapter's host
            // payload is broadcast to all children over the air too).
            if (m_txBuf.blen <= 90)
                for (const auto& c : m_host.clients)
                    if (c.devid)
                    {
                        emitData(NET_HOST_SEND, m_txBuf.blen, m_txBuf.buf, m_txBuf.blen);
                        break;
                    }
        }
        else if (state == stClient)
        {
            if (m_txBuf.blen <= 16)
                emitData(NET_CLIENT_SEND,
                         (static_cast<uint32_t>(m_txBuf.blen) << 24) |
                             (m_client.clnum << 16) | m_client.devid,
                         m_txBuf.buf, m_txBuf.blen);
        }
    }

    void restartIdExchange()
    {
        comstate = comIdWait;
        m_danceIdx = 0;
        // A re-run of AgbRFU_checkID is always preceded by AgbRFU_SoftReset —
        // the adapter's link state is gone on real hardware, so drop ours
        // too. Peers are told first so their slots free instantly instead of
        // via the 4s timeout. (Belt-and-braces with the SD-pulse reset, which
        // usually fires first; resetLinkState is idempotent.)
        resetLinkState();
    }


    // Reset everything the game's soft reset / RESET command (0x10) clears,
    // WITHOUT touching the command-transfer machinery (comstate/m_cmd/m_plen
    // may be mid-exchange when this runs from processCommand).
    void resetLinkState()
    {
        if (state == stClient && m_client.devid)
            emitCmd(NET_DISCONNECT, m_client.devid | (m_client.clnum << 16));
        else if (state == stHost)
            for (unsigned i = 0; i < 4; i++)
                if (m_host.clients[i].devid)
                {
                    emitCmd(NET_DISCONNECT, m_host.clients[i].devid | (i << 16));
                    dbgWipeReset = dbgWipeReset + 1;
                }

        const uint16_t keepDevid = m_host.devid;  // stable across room cycles
        state = stIdle;
        m_host = HostState{};
        m_host.devid = keepDevid;
        m_client = ClientState{};
        m_txBuf = {};
        m_timeoutFrames = defTimeoutFrames;
        m_rtxMax = defRtxMax;
        m_maxClients = 4;
        m_syscfg = (defRtxMax << 8) | defTimeoutFrames;
        m_waitArmed = false;
        m_waitAckRead = false;
        m_nextDataEvtMs = 0;
        m_flowActive = false;
        m_flowHoldUntilMs = 0;
        m_txDirty = false;
        m_carrierOn = false;
    }

    void finishCommand()
    {
        counters.commands++;
        // Record the opcode (set in consume()) for the diagnostic command ring.
        dbgCmdRing[dbgCmdRingHead] = m_cmd;
        dbgCmdRingHead = (dbgCmdRingHead + 1) & 7;
        if (dbgCmdRingCount < 8) dbgCmdRingCount = dbgCmdRingCount + 1;
        const int32_t ret = processCommand();
        if (ret < 0)
        {
            counters.errResponses++;
            comstate = comRespErr;
            m_errCode = static_cast<uint32_t>(-ret);
            m_plen = 1;
        }
        else
        {
            comstate = comRespCmd;
            m_plen = static_cast<uint8_t>(ret);
        }
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Delivery-side transitions: returns the word the GBA reads in the NEXT
    // transfer and advances the response stream (the producing half of gpsp
    // rfu_transfer, one step ahead).
    //-//////////////////////////////////////////////////////////////////////-//

    uint32_t produce()
    {
        switch (comstate)
        {
            case comIdWait:
                return 0;  // ≙ gpsp RESET: zeros until the GBA sends 0x494E

            case comIdDance:
                // Play the canned dance, one word per transfer. The index was
                // reset to 0 when the first 0x494E arrived; the consume side
                // leaves comIdDance (so this isn't reached) once exhausted.
                return (m_danceIdx < 9) ? ID_DANCE[m_danceIdx++] : BUSY_WORD;

            case comWaitCmd:
                return BUSY_WORD;

            case comRespCmd:
            {
                const uint32_t ack = 0x99660080 | m_cmd | (m_plen << 8);
                if (m_cmd == CMD_WAIT || m_cmd == CMD_RTX_WAIT || m_cmd == CMD_SEND_DATAW ||
                    m_cmd == CMD_WAIT2)
                {
                    // Roles flip: the adapter answers next as bus master. The
                    // GBA reads THIS ACK in-band, as the tail of its own master
                    // REQ/ACK exchange (one more transfer — m_waitAckRead), then
                    // immediately drops SIOCNT to external clock (slave) and
                    // clocks nothing more (librfu_intr.c:111-124; gpsp
                    // rfu.c:639-652). Deadlines arm at the first poll after
                    // that (pollWaitEvent has nowMs).
                    comstate = comWaitEvent;
                    m_waitArmed = false;
                    m_waitAckRead = false;
                }
                else
                {
                    comstate = m_plen ? comRespDat : comWaitCmd;
                    m_cnt = 0;
                }
                return ack;
            }

            case comRespDat:
            {
                const uint32_t w = m_buf[m_cnt++];
                if (m_cnt == m_plen)
                    comstate = comWaitCmd;
                return w;
            }

            case comRespErr:
                comstate = comRespErr2;
                return 0x996601EE;

            case comRespErr2:
                comstate = comWaitCmd;
                return m_errCode;

            default:  // comWaitDat, comWaitEvent, comWaitResp
                return BUSY_WORD;
        }
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Command execution (≙ gpsp rfu_process_command, rfu.c:265-558).
    // Returns the response word count, or negative for an error response.
    //-//////////////////////////////////////////////////////////////////////-//

    int32_t processCommand()
    {
        switch (m_cmd)
        {
            case CMD_INIT1:
            case CMD_INIT2:
                // Pure ACKs, like gpsp. Resets are owned by the SD-line pulse
                // (onSdReset — AgbRFU_SoftReset always precedes a real
                // re-init) and the 0x494E checkID escape. 0x10 must NOT clear
                // link state itself: the union-room parent's link manager
                // sends a routine 0x10 when it restarts its advertise cycle,
                // and one landing right after a child connects (relay skew
                // widens that race beyond what frame-synchronous RF allows)
                // wiped the fresh client slot — dropping every subsequent
                // child frame at slot validation and starving the strength
                // watchdog into a link-loss kill (HW test #39, pwr0 counter).
                return 0;

            case CMD_SYSCFG:
                m_syscfg = m_buf[0];
                m_timeoutFrames = static_cast<uint8_t>(m_buf[0]);
                m_rtxMax = static_cast<uint8_t>(m_buf[0] >> 8);
                // Bits 16-17: room size (0=5 players .. 3=2 players). gpsp
                // ignores these; honoring them makes a smaller room actually
                // reject the surplus joiners.
                m_maxClients = static_cast<uint8_t>(4 - ((m_buf[0] >> 16) & 0x3));
                return 0;

            case CMD_CFGSTAT:
                // Adapter configuration readback (afska wireless_adapter.md):
                // host = broadcast data + the SYSCFG word + 0x101; client = six
                // zeros + 0x101. gpsp returns nothing here; FRLG doesn't issue
                // it, but the real adapter answers and it costs nothing.
                if (state == stHost)
                {
                    std::memcpy(m_buf, m_host.bdata, sizeof(m_host.bdata));
                    m_buf[6] = m_syscfg;
                    m_buf[7] = 0x101;
                    return 8;
                }
                for (int j = 0; j < 6; j++) m_buf[j] = 0;
                m_buf[6] = 0x101;
                return 7;

            case CMD_SYSVER:
                m_buf[0] = 0x00830117;
                return 1;

            case CMD_SYSSTAT:
                if (state == stHost)
                    m_buf[0] = (1 << 24) | m_host.devid;
                else if (state == stClient)
                    m_buf[0] = (5 << 24) | ((1 << m_client.clnum) << 16) | m_client.devid;
                else
                    m_buf[0] = 0;
                return 1;

            case CMD_SLOTSTAT:
                if (state == stHost)
                {
                    uint32_t cnt = 0;
                    m_buf[cnt++] = 0;
                    for (unsigned i = 0; i < 4; i++)
                    {
                        if (m_host.clients[i].devid)
                        {
                            m_buf[0]++;
                            m_buf[cnt++] = m_host.clients[i].devid | (i << 16);
                        }
                    }
                    return cnt;
                }
                return 0;

            case CMD_LINKPWR:
                // Signal strength, byte per slot. librfu's watchLink polls
                // this every ~4 frames and FOUR consecutive zero readings for
                // a connected slot force a link-loss disconnect (the parent's
                // 0x1B/0x30/0x19 kill we traced) — so report full strength
                // for every occupied slot, keyed on OCCUPANCY alone: the
                // game's connSlotFlag outlives our transient host/scan state
                // cycling, and a zero during any window is a loss strike.
                m_buf[0] = (m_host.clients[0].devid ? 0x000000FFu : 0) |
                           (m_host.clients[1].devid ? 0x0000FF00u : 0) |
                           (m_host.clients[2].devid ? 0x00FF0000u : 0) |
                           (m_host.clients[3].devid ? 0xFF000000u : 0);
                if (state == stClient)
                    m_buf[0] = 0xFFu << (m_client.clnum * 8);
                // Telemetry: strength polls answered all-zero while the game
                // could believe a link exists are exactly the loss strikes.
                dbgLastLinkPwr = m_buf[0];
                // Count only meaningful zeros: hosting with no clients is a
                // legitimate zero; a zero while a client is (or was just)
                // linked is a watchdog loss-strike.
                if (m_buf[0] == 0 && state == stClient)
                    dbgLinkPwrZero = dbgLinkPwrZero + 1;
                return 1;

            case CMD_BCRD_START:
                return 0;

            case CMD_BCRD_STOP:
            case CMD_BCRD_FETCH:
            {
                // gpsp picks up to 4 random valid peers; with our 4-entry
                // table a rotated scan keeps the same fairness intent.
                uint32_t cnt = 0;
                const unsigned start = rand16 ? (rand16(randCtx) % 4) : 0;
                for (unsigned j = 0; j < 4 && cnt < 4 * 7; j++)
                {
                    const auto& p = m_peers[(start + j) % 4];
                    if (p.valid)
                    {
                        // Metadata word: devid + the next-available-slot byte
                        // from the beacon (0xFF = full/closed) — the game
                        // gates hails on it (partner.slot != 0xFF).
                        m_buf[cnt++] = p.devid |
                                       (static_cast<uint32_t>(p.nextSlot) << 16);
                        std::memcpy(&m_buf[cnt], p.data, sizeof(p.data));
                        cnt += 6;
                    }
                }
                return cnt;
            }

            case CMD_BCST_DATA:
                if (m_plen == 6)
                    std::memcpy(m_host.bdata, m_buf, sizeof(m_host.bdata));
                return 0;

            case CMD_HOST_START:
                if (state == stClient)
                    return -1;
                if (state == stIdle)
                {
                    // Keep the devid stable across the Union Room's constant
                    // HOST_STOP -> scan -> HOST_START cycles: a player hails
                    // using the devid from their (snapshot) scan list, and a
                    // rotating id invalidates the target between scan and
                    // connect — the connect can then never be routed. gpsp
                    // re-mints here too, but its lockstep emulation masks it;
                    // a real adapter keeps its session id across EndHost.
                    if (!m_host.devid) m_host.devid = newDevid();
                    for (auto& c : m_host.clients)
                    {
                        if (c.devid) dbgWipeHostStart = dbgWipeHostStart + 1;
                        c = HostClient{};
                    }
                    state = stHost;
                }
                m_host.open = true;      // (re)open the room to joiners
                m_host.bcastNow = true;  // ≙ tx_ttl = 0xff: broadcast immediately
                return 0;

            case CMD_HOST_STOP:
                if (state == stIdle)
                    return -1;
                if (state == stHost)
                {
                    // EndHost CLOSES the room: no more beacons, no new
                    // joiners; existing clients stay linked. This is FRLG's
                    // standard move right after a successful join.
                    m_host.open = false;
                    for (const auto& c : m_host.clients)
                        if (c.devid)
                            return 0;  // clients remain: stay in host mode
                    state = stIdle;
                }
                return 0;

            case CMD_HOST_ACCEPT:
            {
                if (state == stIdle)
                    return -1;
                uint32_t cnt = 0;
                for (unsigned i = 0; i < 4; i++)
                    if (m_host.clients[i].devid)
                        m_buf[cnt++] = m_host.clients[i].devid | (i << 16);
                return cnt;
            }

            case CMD_CONNECT:
            {
                if (state == stHost)
                    return -1;
                // Either side may initiate (the player who hails in the Union
                // Room; FRLG never auto-connects). Emit even when the target
                // devid is not in our peer table: the game hails from a scan
                // SNAPSHOT and the peer's beacon may have TTL'd out during
                // its scan phase — the relay matches against each member's
                // recent devid history and is the better judge. tick() keeps
                // re-emitting while stConnecting (the RF-retry analog), so a
                // NACK from a mid-scan target self-heals within its next
                // advertise window.
                m_connectTarget = static_cast<uint16_t>(m_buf[0] & 0xFFFF);
                m_connectLastMs = 0;   // armed by the first tick (it has nowMs)
                m_connectRetries = 0;
                emitCmd(NET_CONNECT_REQ, m_connectTarget);
                state = stConnecting;
                return 0;
            }

            case CMD_ISCONNECTED:
                if (state == stHost)
                    return -1;
                if (state == stConnecting)
                    m_buf[0] = CONN_INPROGRESS;
                else if (state == stIdle)
                    m_buf[0] = CONN_FAILED;
                else
                    m_buf[0] = m_client.devid | (m_client.clnum << 16);
                return 1;

            case CMD_CONCOMPL:
                if (state == stHost)
                    return -1;
                if (state == stClient)
                {
                    m_buf[0] = m_client.devid | (m_client.clnum << 16);
                }
                else
                {
                    m_buf[0] = CONN_COMP_FAIL;
                    state = stIdle;
                }
                return 1;

            case CMD_SEND_DATAW:
            case CMD_SEND_DATA:
                if (!m_plen)
                    return 0;

                {
                    // Clamp to the staging buffer — legit traffic maxes at 23
                    // payload words (90 bytes), but m_plen is attacker/GBA
                    // controlled up to 255.
                    uint32_t words = m_plen - 1u;
                    if (words > sizeof(m_txBuf.buf) / sizeof(uint32_t))
                        words = sizeof(m_txBuf.buf) / sizeof(uint32_t);
                    if (state == stHost)
                    {
                        m_txBuf.blen = m_buf[0] & 0x7F;
                        std::memcpy(m_txBuf.buf, &m_buf[1], words * sizeof(uint32_t));
                    }
                    else if (state == stClient)
                    {
                        // The byte count sits at a slot-dependent position.
                        m_txBuf.blen = (m_buf[0] >> (8 + m_client.clnum * 5)) & 0x1F;
                        std::memcpy(m_txBuf.buf, &m_buf[1], words * sizeof(uint32_t));
                    }
                }
                [[fallthrough]];
            case CMD_RTX_WAIT:
                if (state != stHost && state != stClient)
                    return -1;
                emitUniTx();
                m_txDirty = true;  // tick() stamps m_lastUniTxMs (it has the clock)
                return 0;

            case CMD_RECV_DATA:
                if (state == stHost)
                {
                    uint32_t cnt = 0, bufbytes = 0;
                    uint8_t tmp[16 * 4] = {};
                    m_buf[cnt++] = 0;  // per-client byte counts as a bitfield
                    for (unsigned i = 0; i < 4; i++)
                    {
                        auto& c = m_host.clients[i];
                        if (!c.devid) continue;
                        const uint8_t* p;
                        uint32_t dlen = c.pkt.read(&p);  // one front pop per poll
                        if (dlen > 16) dlen = 16;        // gpsp truncates (rfu.c:496)
                        if (dlen)
                        {
                            std::memcpy(&tmp[bufbytes], p, dlen);
                            bufbytes += dlen;
                            m_buf[0] |= dlen << (8 + i * 5);
                        }
                    }
                    for (uint32_t i = 0; i < (bufbytes + 3) / 4; i++)
                        m_buf[cnt++] = unpack32le(&tmp[i * 4]);
                    return cnt;
                }
                else if (state == stClient)
                {
                    uint32_t cnt = 0;
                    const uint8_t* p;
                    const uint32_t dlen = m_client.pkt.read(&p);
                    m_buf[cnt++] = dlen;
                    for (uint32_t j = 0; j < (dlen + 3) / 4; j++)
                        m_buf[cnt++] = unpack32le(&p[j * 4]);
                    return cnt;
                }
                return 0;

            case CMD_WAIT:
                // The role reversal is handled when the ACK is produced.
                return 0;

            case CMD_DISCONNECT:
                if (state == stClient)
                {
                    emitCmd(NET_DISCONNECT, m_client.devid | (m_client.clnum << 16));
                    state = stIdle;
                }
                else if (state == stHost)
                {
                    // Only occupied slots: the game's slot view lags ours
                    // (clientTimeoutMs frees slots autonomously), and a
                    // devid-0 DISCONNECT would be garbage for the relay.
                    for (unsigned i = 0; i < 4; i++)
                    {
                        if ((m_buf[0] & (1u << i)) && m_host.clients[i].devid)
                        {
                            emitCmd(NET_DISCONNECT, m_host.clients[i].devid | (i << 16));
                            m_host.clients[i] = HostClient{};
                            dbgWipeCmdDisc = dbgWipeCmdDisc + 1;
                        }
                    }
                }
                return 0;

            default:
                return 0;  // unknown commands are ACKed, like gpsp
        }
    }
};

}  // namespace rfuproto
