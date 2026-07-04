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

inline int rfu1FrameSize(uint32_t ptype)
{
    switch (ptype)
    {
        case NET_BROADCAST:                      return 36;
        case NET_HOST_SEND: case NET_CLIENT_SEND: return 104;
        case NET_CONNECT_REQ: case NET_CONNECT_ACK:
        case NET_CONNECT_NACK: case NET_DISCONNECT:
        case NET_CLIENT_ACK:                     return 16;
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
            m_keepaliveMs = nowMs + clientKeepaliveMs;
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
        else if (dataAvail())
        {
            m_buf[0] = 0x99660000 | CMD_RESP_DATA;
            m_buf[1] = BUSY_WORD;
            m_plen = 2;
        }
        else if (state == stHost && timeReached(nowMs, m_rtxDeadlineMs))
        {
            // The simulated retransmission window elapsed without client
            // data — report "no response" exactly as gpsp does.
            m_buf[0] = 0x99660000 | CMD_RESP_DATA | (1 << 8);
            m_buf[1] = 0x00000F0F;
            m_buf[2] = BUSY_WORD;
            m_plen = 3;
        }
        else if (state == stClient && timeReached(nowMs, m_keepaliveMs))
        {
            // No host payload crossed the relay yet: deliver an EMPTY data
            // event, like the real adapter's per-RF-frame cadence (a child
            // gets an event every ~17ms while connected — the parent's radio
            // retransmits continuously even when the game sends nothing).
            // Without this the child's whole pump — including its NI
            // fragment retries — runs at the 533ms TIMEO cadence, ~30x
            // slower than hardware, and any sub-second window on the parent
            // side (name acceptance) expires before one retry. The short
            // grace lets a genuinely in-flight host frame win the poll.
            m_buf[0] = 0x99660000 | CMD_RESP_DATA;
            m_buf[1] = BUSY_WORD;
            m_plen = 2;
        }
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
                        // Real-adapter semantics: the receive buffer is ONE
                        // packet and the LATEST frame wins (unread data is
                        // lost — the LL layer retransmits by design). A FIFO
                        // here jams with stale retransmits under the games'
                        // per-frame pacing and starves the fresh window.
                        if (m_client.pkt.hblen) counters.rxDropQueueFull++;
                        std::memcpy(m_client.pkt.hdata, payload, blen);
                        m_client.pkt.hblen = blen;
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
                        // Keep-latest, single buffer (see NET_HOST_SEND).
                        auto& p = m_host.clients[clid].pkt;
                        if (p.datalen) counters.rxDropQueueFull++;
                        std::memcpy(p.data, payload, blen);
                        p.datalen = static_cast<uint8_t>(blen);
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
    static constexpr uint32_t clientKeepaliveMs = 17; // ≈ one RF frame

    struct HostClient
    {
        uint16_t devid = 0;          // 0 = empty slot
        uint32_t lastHeardMs = 0;
        // One packet, latest wins — the real adapter's buffer depth.
        struct { uint8_t datalen; uint8_t data[16]; } pkt = {};
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
        // One packet, latest wins — the real adapter's buffer depth.
        struct { uint8_t hblen; uint8_t hdata[128]; } pkt = {};
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
    uint32_t m_keepaliveMs = 0;
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

    bool dataAvail() const
    {
        // ≙ gpsp rfu_data_avail
        if (state == stClient)
            return m_client.pkt.hblen != 0;
        if (state == stHost)
        {
            for (const auto& c : m_host.clients)
                if (c.devid && c.pkt.datalen)
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
                else
                    return -1;
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
                        const uint32_t dlen = c.pkt.datalen > 16 ? 16 : c.pkt.datalen;
                        if (c.devid && dlen)
                        {
                            std::memcpy(&tmp[bufbytes], c.pkt.data, dlen);
                            bufbytes += dlen;
                            m_buf[0] |= dlen << (8 + i * 5);
                            c.pkt.datalen = 0;
                        }
                    }
                    for (uint32_t i = 0; i < (bufbytes + 3) / 4; i++)
                        m_buf[cnt++] = unpack32le(&tmp[i * 4]);
                    return cnt;
                }
                else if (state == stClient)
                {
                    uint32_t cnt = 0;
                    const uint32_t dlen = m_client.pkt.hblen;
                    m_buf[cnt++] = dlen;
                    for (uint32_t j = 0; j < (dlen + 3) / 4; j++)
                        m_buf[cnt++] = unpack32le(&m_client.pkt.hdata[j * 4]);
                    m_client.pkt.hblen = 0;
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
