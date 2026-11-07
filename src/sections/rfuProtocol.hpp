#include <cstdint>
#include <cstring>
#include <cstddef>

#pragma once

// GBA Wireless Adapter (RFU, AGB-015) protocol core. Port of gpsp's
// emulation (gpsp/rfu.c): the adapter answers every SIO transfer from its own
// state, while broadcasts, connections and payloads move over the Celio relay
// as "RFU1" frames (byte-compatible with gpsp's netpackets).
//
// gpsp's rfu_transfer(sent) answers within the same transfer; real hardware
// preloads its TX word one transfer ahead, so the FSM splits into consume(rx)
// (receive-side transitions) and produce() (the next response word plus the
// delivery-side transitions). Command-phase responses are input-independent,
// so the shift reproduces gpsp's stream exactly; the SIO32 ID exchange cannot
// be a same-cycle echo on hardware and is answered one transfer behind (see
// produce(), comIdDance).
//
// Zephyr-free so the FSM and framing compile on the host. Concurrency
// contract (enforced by rfuProtocolSection): onSlaveTransfer() runs in the
// PIO done-ISR; applyNetPacket(), pollWaitEvent(), tick() and the delivery
// bookkeeping run under irq_lock(); stagedTx is written by the FSM and read
// by the PIO ISR push path.

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
    comIdDance,     // answering the NINTENDO ID exchange
    comWaitCmd,
    comWaitDat,
    comRespCmd,
    comRespDat,
    comRespErr,
    comRespErr2,
    comWaitEvent,   // waiting for data/timeout; adapter will turn bus master
    comWaitResp,    // streaming an adapter-master response frame
    comReplayWait,  // the GBA restarted a command mid-response: busy until it sends the header again
    comReplayDat    // skipping the re-sent command words before answering the same response again
};

// SIO32 ID exchange (FRLG AgbRFU_checkID / librfu_sio32id.c). The GBA, as SIO
// master, walks the "NINTENDO" connection words and expects the adapter
// (slave) to answer with a complement dance that converges to RFU_ID.
// The answer is built from the word the GBA just sent (see produce,
// comIdDance): its id in the high half, the complement of the previous high
// in the low half, stepping one id ahead once a pair has been answered. A
// fixed sequence cannot work: until the check first succeeds the GBA's main
// loop clocks extra transfers whose replies are overwritten unread.
constexpr uint32_t ID_RFU = 0x00008001;
constexpr uint16_t ID_NINTENDO[4] = { 0x494E, 0x544E, 0x4E45, 0x4F44 };

// The id the game walks to after id, or the adapter's own id after the last one.
constexpr uint16_t idSuccessor(uint16_t id)
{
    for (int i = 0; i < 3; i++)
        if (id == ID_NINTENDO[i]) return ID_NINTENDO[i + 1];
    return static_cast<uint16_t>(ID_RFU);
}

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
// Celio extension (not in gpsp): backpressure. Real adapters share one RF
// frame clock; over the relay the two game clocks are independent, so a
// jitter burst leaves a standing backlog the games cannot drain (consumption
// is capped at one frame per game frame). The peer signals a deep inbound
// FIFO and the sender then paces its wait-data events until it clears.
// hdata: bit0 = active, bits 8-15 = queue depth.
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
// bytes, zero padded to 92
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
                    restart();  // unknown ptype: resync on the next magic
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
    uint32_t waitEventTransfers = 0;  // GBA clocked the adapter while in WAITEVENT
    uint32_t txFrames = 0;
    uint32_t rxFrames = 0;
    uint32_t rxDropQueueFull = 0;
    uint32_t rxDropMalformed = 0;
    uint32_t bcastsSent = 0;
    uint32_t loginRestarts = 0;
    uint32_t deliveryAborts = 0;      // hard abort: wait abandoned mid-frame
    uint32_t deliveryRetries = 0;     // soft abort: nothing on the wire yet, re-poll
    uint32_t commandRestarts = 0;     // GBA re-sent a command header mid-exchange (librfu STWI restart)
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

    // Web-session role hint (SetMode variant byte; set once by the section
    // ctor after reset()). It gates nothing: FRLG never auto-connects, so the
    // Union Room runs symmetric and any member may host, scan and initiate.
    // Kept for the wire protocol and as a diagnostic label.
    static constexpr uint8_t ROLE_SYMMETRIC = 0, ROLE_HOST = 1, ROLE_CLIENT = 2;
    uint8_t roleLock = ROLE_SYMMETRIC;

    //-//////////////////////////////////////////////////////////////////////-//
    // GBA-master (slave-role) transfer: PIO done-ISR
    //-//////////////////////////////////////////////////////////////////////-//

    // Bring-up diagnostics (read by the section's LED indicator). They
    // localize a detection failure to the physical layer:
    //   - !dbgAnyRx              → the GBA's clock/data never arrived
    //                              (wiring / pin map / SC not detected)
    //   - dbgAnyRx, !dbgNintendo → words arrive but never the 0x494E marker
    //                              → wire bit order is likely wrong
    //   - dbgNintendo, stuck     → bit order right but no convergence → SC
    //                              sample-edge (clock polarity) or timing
    //   - reached comWaitCmd     → detection passed
    volatile bool dbgAnyRx = false;
    volatile bool dbgNintendo = false;
    volatile uint32_t dbgFirstRx = 0;
    volatile uint32_t dbgLastRx = 0;

    // Post-detection telemetry (read by the section's 0x2E frame). The command
    // ring holds the opcodes leading up to a WAIT/clock-reversal stall.
    uint8_t dbgLastCmd()  const { return m_cmd; }
    uint8_t dbgLastPlen() const { return m_plen; }
    bool    dbgWaitAck()  const { return m_waitAckRead; }
    volatile uint8_t dbgCmdRing[8] = {};
    volatile uint8_t dbgCmdRingHead = 0;   // next write slot (one past newest)
    volatile uint8_t dbgCmdRingCount = 0;
    // Timestamped trace (commands, wait events, delivery outcomes, restarts,
    // resets), oldest overwritten. Written from the PIO ISR and from the
    // section thread under irq_lock; snapshotted when the game resets its
    // adapter mid-link, which is how a game-side link error ends.
    enum : uint8_t { TR_CMD = 1, TR_EVENT, TR_RESTART, TR_DELIVERY, TR_SDRESET, TR_LOGIN };
    static constexpr uint8_t kTraceLen = 14;
    struct TraceEntry { uint8_t kind; uint8_t code; uint16_t dtMs; };
    TraceEntry dbgTrace[kTraceLen] = {};
    uint8_t dbgTraceHead = 0;    // next write slot
    uint8_t dbgTraceCount = 0;
    uint32_t dbgTraceLastMs = 0;
    void trace(uint8_t kind, uint8_t code, uint32_t nowMs)
    {
        if (nowMs == 0) nowMs = dbgTraceLastMs;
        const uint32_t dt = dbgTraceCount ? nowMs - dbgTraceLastMs : 0;
        dbgTraceLastMs = nowMs;
        dbgTrace[dbgTraceHead] = { kind, code, static_cast<uint16_t>(dt > 65535 ? 65535 : dt) };
        dbgTraceHead = static_cast<uint8_t>((dbgTraceHead + 1) % kTraceLen);
        if (dbgTraceCount < kTraceLen) dbgTraceCount++;
    }
    // Wait-event class counters + the last event header delivered.
    volatile uint8_t dbgEvData = 0, dbgEvRtx = 0, dbgEvTimeo = 0, dbgEvDisc = 0;
    volatile uint32_t dbgLastEvent = 0;
    // Link-strength telemetry (the librfu watchLink 4-strike disconnect).
    volatile uint32_t dbgLastLinkPwr = 0;
    volatile uint8_t dbgLinkPwrZero = 0;
    // Slot-wipe provenance: which path cleared an occupied host client
    // slot (evict = lastHeard timeout, netDisc = inbound NET_DISCONNECT,
    // hostStart = HOST_START-from-idle, cmdDisc = game's 0x30, reset =
    // resetLinkState). Saturating 4-bit when packed into telemetry.
    volatile uint8_t dbgWipeEvict = 0, dbgWipeNetDisc = 0, dbgWipeHostStart = 0,
                     dbgWipeCmdDisc = 0, dbgWipeReset = 0;
    // Flow-control counters: backpressure hints sent (child), paced data
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
    // Adapter-master delivery (WAITRESP), driven synchronously by the
    // section thread (runDelivery), which clocks eventWord(0..count-1) into
    // the GBA with the librfu inter-word handshake between words.
    //-//////////////////////////////////////////////////////////////////////-//

    // True when pollWaitEvent has built a response frame to clock into the GBA.
    bool deliveryPending() const { return comstate == comWaitResp; }

    uint8_t  eventWordCount() const { return m_plen; }
    uint32_t eventWord(uint8_t i) const { return m_buf[i]; }

    // All event words clocked and the final handshake completed: re-arm as
    // GBA-slave for the next command.
    void finishDelivery(uint32_t nowMs = 0)
    {
        comstate = comWaitCmd;
        stagedTx = BUSY_WORD;
        trace(TR_DELIVERY, 0, nowMs);
    }

    // Delivery never started or stalled (GBA not receptive): return to the
    // command phase, like a real adapter whose wait the game abandoned.
    void abortDelivery(uint32_t nowMs = 0)
    {
        counters.deliveryAborts++;
        comstate = comWaitCmd;
        stagedTx = BUSY_WORD;
        trace(TR_DELIVERY, 2, nowMs);
    }

    // Delivery failed before any word reached the wire. The GBA is still armed
    // (librfu runs no timer before word 1), so the event is put back and
    // pollWaitEvent rebuilds it on the next tick. Deadlines stay armed; a
    // game-side soft reset (0x494E) still breaks out via restartIdExchange.
    void retryDelivery(uint32_t nowMs = 0)
    {
        counters.deliveryRetries++;
        comstate = comWaitEvent;
        stagedTx = BUSY_WORD;
        trace(TR_DELIVERY, 1, nowMs);
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Wait-event evaluation (≙ gpsp rfu_update, rfu.c:871-929). Section
    // thread, under irq_lock. Returns true when a response frame was built and
    // the section should run the adapter-master delivery. gpsp's "GBA must be
    // in slave mode" SIOCNT gate becomes the m_waitAckRead gate: an armed
    // slave shows no level on SO to check (see runDelivery).
    //-//////////////////////////////////////////////////////////////////////-//

    bool pollWaitEvent(uint32_t nowMs)
    {
        // comWaitEvent is entered when the WAIT-class ACK is staged; the GBA
        // still clocks one more master transfer to read it (librfu
        // sio32intr_clock_master state 1) before dropping to slave. The adapter
        // may take the bus only after that transfer (m_waitAckRead, set by
        // consume()): flipping earlier discards the staged ACK and drives SC
        // against the GBA. librfu arms no timer between the WAIT ack and word 1
        // (librfu_intr.c:107,161), so a wait with no event is not a stall; it
        // runs to the ~533ms TIMEO.
        if (comstate != comWaitEvent || !m_waitAckRead) return false;

        // Arm the wait deadlines once per WAIT, on the first poll (produce()
        // has no nowMs). The TIMEO / host-rtx windows run from here.
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
        // Client data events are paced to one per RF frame (~16.7ms): the
        // child's librfu MSC callback enqueues every aggregate into the game's
        // 20-slot recvQueue, which drains one per frame (only all-zero frames
        // are skipped, link_rfu_3.c:573-578). Unpaced, a relay burst resolves
        // waits back-to-back, latches that queue full and raises the games'
        // fatal "communication error" screen (F_RFU_ERROR_5|6|7). 16ms leaves
        // headroom to drain the inbound FIFO after a burst.
        // A host is unpaced (its game consumes via 0x26 polls, its recvQueue
        // is unused) except under peer backpressure, which trims both roles.
        else if (dataAvail() && timeReached(nowMs, m_nextDataEvtMs))
        {
            // Under a peer hint, both roles trim to flowTrimGapMs (~9%
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
        // rtx must not preempt paced-out data: resolving the wait "empty"
        // while frames are pending would defeat the pacing.
        else if (state == stHost && !dataAvail() &&
                 timeReached(nowMs, m_rtxDeadlineMs))
        {
            // The simulated retransmission window elapsed without client
            // data: report "no response" as gpsp does.
            m_buf[0] = 0x99660000 | CMD_RESP_DATA | (1 << 8);
            m_buf[1] = 0x00000F0F;
            m_buf[2] = BUSY_WORD;
            m_plen = 3;
        }
        // There is no empty-data keepalive: a periodic RESP_DATA without peer
        // data would unblock the child's wait and let its pump free-run
        // instead of being paced by the parent's frames. gpsp's WAIT
        // rendezvous (a data event only on an actual arrival) keeps the two
        // pumps in lockstep and the FIFOs shallow.
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
        // Telemetry: which event class resolves each wait. It matters to the
        // games' UNI pumps (a TIMEO mid-pump fails the parent's DRAC-ACK check).
        dbgLastEvent = m_buf[0];
        trace(TR_EVENT, static_cast<uint8_t>((m_buf[0] & 0x7F) | ((m_buf[0] & 0x100) ? 0x80 : 0)), nowMs);
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
    // Periodic upkeep (≙ gpsp rfu_frame_update). Section thread, under
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

        // Idle retransmission: the real radio re-broadcasts its send buffer
        // once per RF frame. The peer game's link supervision (~5s giveups)
        // and the child's pump cadence (MSC per RF frame) are both calibrated
        // against that carrier; without it a quiet game (flash save, standby
        // barrier, scene transition) drops the peer's pump to the 533ms TIMEO
        // rate and the two sides diverge.
        if (m_txDirty)
        {
            m_lastUniTxMs = nowMs;
            m_carrierOn = false;
            m_txDirty = false;
        }
        // Not gated on m_txBuf.blen: the games issue zero-length sends during
        // the win/save screens, when the peer depends on the carrier. blen=0
        // falls through to the natural-size zero frame in emitUniCarrier.
        else if ((state == stClient || state == stHost) &&
                 (nowMs - m_lastUniTxMs) >=
                     (m_carrierOn ? idleRetxGapMs : idleRetxStartMs))
        {
            // The carrier engages late (idleRetxStartMs), then runs at RF
            // cadence. A one-frame start threshold races the live 59.7Hz
            // stream: zeros slip in ahead of a late send, take the receiver's
            // paced delivery slots and halve throughput. Active streams never
            // gap 60ms, and engaging there is still ~9x faster than the 533ms
            // TIMEO rate.
            m_carrierOn = true;
            // A zero frame of the same shape is synthesized rather than
            // repeating the buffer. The transport is lossless, and repeated
            // non-zero frames enqueue into the peer game's 20-slot recvQueue
            // (only all-zero aggregates are skipped), hold its room-entry gate
            // (recvQueue<=2) open and fill it to its latched-full fatal error.
            // Zeros carry all the peer needs: the child's pump clock (MSC per
            // RF frame) and liveness.
            emitUniCarrier();
            dbgIdleRetx = dbgIdleRetx + 1;
            m_lastUniTxMs = nowMs;
        }

        // Symmetric backpressure. A standing inbound queue never drains
        // (consumption is capped at the game's frame rate) and stretches the
        // games' echo round-trip ~10x; that lands the final block-send's
        // echo-verify callback inside the battle-end Rfu_SetLinkStandbyCallback
        // window, which skips arming while gRfu.callback is busy
        // (link_rfu_2.c:1595-1601) and strands the partner at the barrier. The
        // trim slows the side that is ahead by ~9% (16→18ms); a harder brake
        // splits the games instead of resyncing them. Both roles hint and trim.
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
            // Only an open room advertises (EndHost stops the beacon but keeps
            // its clients). Every Union Room member broadcasts while open so
            // each renders the others. The beacon carries joinability in hdata
            // bits 16-23 (next free slot, 0xFF = full) so scanners can gate
            // their hails.
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
                // Signed delta: lastHeardMs is stamped from the transport
                // context's own uptime read and can be ahead of this thread's
                // nowMs; unsigned subtraction would underflow to ~2^32 and
                // evict a live client at once.
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
    // rfu.c:718-868). Transport context, under irq_lock. The Celio relay
    // routes per ptype (CONNECT_REQ to the devid's owner, ACK/NACK back to the
    // requester, HOST_SEND to the host's clients, CLIENT_* to the host), so
    // identity here is the devid each frame carries. Up to 4 remote peers (a
    // 5-member room) are tracked, keyed by devid.
    //-//////////////////////////////////////////////////////////////////////-//

    void applyNetPacket(uint32_t ptype, uint32_t hdata, const uint8_t* payload,
                        uint16_t plen, uint32_t nowMs)
    {
        counters.rxFrames++;

        switch (ptype)
        {
            case NET_BROADCAST:
            {
                // Every broadcaster is learned so each member renders the
                // others. Keyed by devid; a full table evicts the stalest
                // entry.
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
                // The relay routes CONNECT_REQ to the targeted devid's owner
                // and serializes concurrent requests, so the next ACK/NACK
                // emitted goes back to that requester. Only an open room
                // accepts; a real adapter refuses connects after EndHost.
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
                // Stay in stConnecting: a NACK usually means the target was
                // mid-scan (the Union Room alternates advertise/scan every
                // second). A real adapter retries for the game's whole connect
                // window; tick() re-emits until the retry budget runs out,
                // then ISCONNECTED reports failure.
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
                    // Only this adapter's own disconnect: the host addresses a
                    // specific client (devid | clnum<<16), so another client's
                    // disconnect must not tear this link down.
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
                        // ACK so the host knows this client is alive
                        emitCmd(NET_CLIENT_ACK, m_client.devid | (m_client.clnum << 16));
                        // An all-zero frame (idle stream or carrier) only
                        // ticks the GBA's pump, and one pending frame does
                        // that. Stacked, they fill the FIFO while the game
                        // drains nothing (save screens) and the peer's real
                        // frames tail-drop behind them. A zero is enqueued
                        // only when the queue is empty; real frames still
                        // queue in full order.
                        {
                            bool allZero = true;
                            for (uint32_t zi = 0; zi < blen && allZero; zi++)
                                if (payload[zi]) allZero = false;
                            if (allZero && m_client.pkt.pending())
                                break;
                        }
                        // A parent UNI frame is the unreliable "current state"
                        // stream: the real link overwrites it and never
                        // queues. A parent is never silent (its idle frames
                        // carry the LLSF header), so a pump paused for a save
                        // or screen transition fills this FIFO, every later
                        // frame arrives stale and the link-standby handshake
                        // never arms. A UNI frame behind an unconsumed UNI
                        // frame replaces it; NI frames (sequenced, acked)
                        // still queue in full order.
                        if (isParentUni(payload, blen) && m_client.pkt.pending()
                            && isParentUni(m_client.pkt.tail(), m_client.pkt.tailLen())
                            && !parentUniHasCommand(m_client.pkt.tail(), m_client.pkt.tailLen()))
                        {
                            m_client.pkt.replaceTail(payload, static_cast<uint8_t>(blen));
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
                        // All-zero child frames are no-ops to the parent's
                        // game (an empty poll reads identically, and its
                        // enqueue discards all-zero aggregates), but queued
                        // they take slots the shed cannot free, since zeros
                        // carry no seq steps. Liveness is lastHeardMs, above.
                        // The check covers the whole payload: byte0 alone
                        // carries meaning (the LL comm=0 close frame is 80 00;
                        // an ack nibble there advances the parent's block
                        // sends).
                        {
                            bool allZero = true;
                            for (uint32_t zi = 0; zi < blen && allZero; zi++)
                                if (payload[zi]) allZero = false;
                            if (allZero)
                                break;
                        }
                        HostClient& c = m_host.clients[clid];
                        if (childHasCommand(payload, blen))
                        {
                            // An unchanged re-send would get a new tag and be
                            // acted on twice.
                            if (c.restamp && blen == c.lastLen && std::memcmp(payload, c.last, blen) == 0)
                                break;
                            std::memcpy(c.last, payload, blen);
                            c.lastLen = static_cast<uint8_t>(blen);
                        }
                        c.pkt.push(payload, static_cast<uint8_t>(blen), counters.rxDropQueueFull);
                        if (c.pkt.count > dbgFifoHigh)
                            dbgFifoHigh = c.pkt.count;
                        shedSuperseded(c);
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
                // The peer's inbound queue is standing deep, so this side is
                // ahead. Apply the trim (see tick()); the hold expires so a
                // lost clear cannot stick, and the peer re-hints while deep.
                m_flowHoldUntilMs = (hdata & 1) ? nowMs + flowHoldMs : nowMs;
                dbgFlowHolds = dbgFlowHolds + 1;
                break;

            default:
                counters.rxDropMalformed++;
                break;
        }
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Reset
    //-//////////////////////////////////////////////////////////////////////-//

    // The game pulsed the SD line (AgbRFU_SoftReset): everything resets,
    // including a command mid-parse; the GBA re-runs checkID next. Called
    // from the section thread under irq_lock.
    void onSdReset(uint32_t nowMs = 0)
    {
        trace(TR_SDRESET, static_cast<uint8_t>(comstate), nowMs);
        restartIdExchange();
        m_cnt = 0;
        m_plen = 0;
        m_waitArmed = false;
        m_waitAckRead = false;
        stagedTx = 0;  // comIdWait answers 0 until the GBA sends 0x494E
    }

    // ≙ gpsp rfu_reset: the SD-pulse reset (onSdReset) plus a fresh session
    // start.
    void reset()
    {
        m_header = 0;
        m_lastRx = 0;
        m_idStarted = false;
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
    // Backpressure: hint at highWater, clear at lowWater; the peer trims its
    // data events while held. The hold auto-expires so a lost clear can't
    // stick; the sender re-hints while still deep.
    static constexpr uint8_t  flowHighWater = 6;
    static constexpr uint8_t  flowLowWater = 2;
    static constexpr uint32_t flowRehintMs = 150;
    static constexpr uint32_t flowHoldMs = 300;
    static constexpr uint32_t flowTrimGapMs = 18;     // gentle ~9% trim
    // Host side: child frames queued per slot before superseded reports shed.
    static constexpr uint8_t  hostShedAt = 3;
    static constexpr uint32_t idleRetxGapMs = 17;     // radio re-broadcast cadence
    static constexpr uint32_t idleRetxStartMs = 60;   // quiet threshold to engage

public:
    // Parent-side LLSF header (3 bytes): bmSlot<<18 | state<<14 | ack<<13 | n<<11 | phase<<9 | size.
    static bool isParentUni(const uint8_t* p, uint32_t len)
    {
        if (len < 3) return false;
        const uint32_t f = p[0] | (p[1] << 8) | (p[2] << 16);
        return ((f >> 14) & 0xF) == 4;
    }

    // True when any slot of a parent UNI frame carries a command word. Such a frame is not redundant
    // state: the peer sends it once before reverting to idle (a link-standby round, a block request),
    // so dropping or overwriting it strands the game's handshake.
    static bool parentUniHasCommand(const uint8_t* p, uint32_t len)
    {
        for (uint32_t off = 3; off + 2 <= len; off += 14)
            if (p[off] || p[off + 1]) return true;
        return false;
    }
private:

    // Inbound packet FIFO (≙ gpsp's pkts[4], rfu.c:138-153). The games' UNI
    // command/block protocols assume the lossless shared RF frame clock: every
    // distinct frame must reach the reader exactly once, in order. A
    // latest-wins buffer would skip one whenever two arrive between polls,
    // which is fatal for one-shot commands. Semantics follow gpsp: front-pop on
    // RECV, tail-drop on overflow (rfu.c:827,851), zero-length when starved.
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
        const uint8_t* tail() const { return q[(head + count - 1) % DEPTH]; }
        uint8_t tailLen() const { return qlen[(head + count - 1) % DEPTH]; }

        // Overwrite the newest queued frame in place (count must be non-zero).
        void replaceTail(const uint8_t* p, uint8_t len)
        {
            const uint8_t slot = (head + count - 1) % DEPTH;
            std::memcpy(q[slot], p, len);
            qlen[slot] = len;
        }

        void removeAt(uint8_t i)
        {
            for (uint8_t j = i; j + 1 < count; j++)
            {
                const uint8_t to = (head + j) % DEPTH, from = (head + j + 1) % DEPTH;
                std::memcpy(q[to], q[from], MAXLEN);
                qlen[to] = qlen[from];
            }
            count--;
        }

        void dropFront(uint8_t n)
        {
            if (n > count) n = count;
            head = (head + n) % DEPTH;
            count -= n;
        }

        // Pop the front frame; 0 = queue empty (gpsp returns a zero
        // byte-count header then, never a stale copy). The returned pointer
        // stays valid until the next push; callers copy it out within the
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
        // Depth 32: absorbs browser-side stall-then-burst delivery (~300ms
        // tab pauses ≈ 18 frames) without dropping. A dropped nonzero child
        // frame is a seq gap the parent's game does not recover from.
        PktQueue<16, 32> pkt = {};
        // After the first shed, delivery stamps seq tags; outTag follows the
        // child's own tags until then. last/lastLen detect re-sends.
        bool restamp = false;
        uint8_t outTag = 0;
        uint8_t last[16] = {};
        uint8_t lastLen = 0;
    };
    struct HostState
    {
        uint16_t devid = 0;
        // Open = advertising + accepting joiners. EndHost (0x1B) closes the
        // room: broadcast stops and new connects are refused, but existing
        // clients stay linked (the standard FRLG flow right after a join).
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
        // Depth 32, drained at the paced ~62Hz: covers ~500ms of upstream
        // jitter. A dropped host frame can hang a block transfer (chunks are
        // sent once, with no re-request from the child).
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
    uint32_t m_header = 0;   // header word of the command in progress (restart detection)
    uint8_t m_hdrLen = 0;    // its command word count (m_plen becomes the response length)
    bool m_respErr = false;  // its response is the error frame
    uint32_t m_nowMs = 0;    // clock of the transfer being consumed

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
    // in-band read before it drops to slave); gates delivery.
    bool m_waitAckRead = false;

    // Connect-retry state (the RF layer's transparent retrying, emulated)
    uint16_t m_connectTarget = 0;
    uint32_t m_connectLastMs = 0;
    uint8_t m_connectRetries = 0;

    uint32_t m_lastRx = 0;
    uint32_t m_idLastRx = 0;      // last word of the checkID exchange
    uint16_t m_idLastHigh = 0;    // id the last reply offered
    uint16_t m_idPrevHigh = 0;    // and the one before it
    bool m_idStarted = false;     // a reply has been staged since the exchange began

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
    // closed (post-EndHost); the game does not hail 0xFF rooms.
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
        // ≙ gpsp rfu_data_avail: level-triggered on a non-empty front slot
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
        m_nowMs = nowMs;
        if (isCommandRestart(rx))
        {
            onCommandRestart();
            m_lastRx = rx;
            return;
        }
        switch (comstate)
        {
            case comIdWait:
                // Send zeros until the GBA transmits its first NINTENDO word;
                // the answering sequence starts on the next staged transfer.
                if ((rx & 0xFFFF) == ID_NINTENDO[0])
                {
                    comstate = comIdDance;
                    m_idStarted = false;
                    m_idLastRx = rx;
                }
                break;

            case comIdDance:
                // The game sends its own id once checkID has converged; the
                // command phase begins on the next transfer.
                if ((rx & 0xFFFF) == static_cast<uint16_t>(ID_RFU))
                {
                    comstate = comWaitCmd;
                    break;
                }
                m_idLastRx = rx;
                break;

            // While a response streams (or waits to be replayed) the GBA sends only
            // 0x80000000 fillers, so a header here is a new command: the exchange in
            // progress was abandoned (restarts exhausted).
            case comRespDat:
            case comRespErr2:
            case comReplayWait:
            case comWaitCmd:
                if ((rx >> 16) == 0x9966)
                {
                    m_header = rx;
                    m_plen = static_cast<uint8_t>(rx >> 8);
                    m_hdrLen = m_plen;
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

            case comReplayDat:
                // A re-sent command word of a command that already ran.
                if (++m_cnt >= m_hdrLen)
                {
                    m_cnt = 0;
                    comstate = m_respErr ? comRespErr : comRespCmd;
                }
                break;

            case comWaitEvent:
                // Normally one GBA-master transfer lands here: the in-band read
                // of the staged WAIT-class ACK, which opens the delivery gate;
                // the GBA then drops to slave. A 0x494E without a 0x9966 header
                // is a mid-wait soft reset (AgbRFU_checkID), which restarts the
                // ID exchange.
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
                // The GBA is the bus slave here and is not expected to clock
                // (≙ gpsp's no-op branch), apart from the restart escape.
                if ((rx & 0xFFFF) == 0x494E && (rx >> 16) != 0x9966)
                {
                    counters.loginRestarts++;
                    restartIdExchange();
                }
                else
                    counters.waitEventTransfers++;
                break;

            // Response-delivery states ignore the incoming word (the GBA
            // clocks dummy values while reading the response), ≙ gpsp's
            // "disregard the input value".
            default:
                break;
        }

        m_lastRx = rx;
    }

    // Child frame: 2-byte LLSF header, 14-byte slot. Slot byte 0 bits 5-7 seq
    // tag, byte 1 command high byte, byte 2 held keys. Only command frames
    // carry a seq step (link_rfu_2.c:876-907).
    static bool childHasCommand(const uint8_t* p, uint32_t len)
    {
        return len >= 4 && p[3] != 0;
    }

    // Held-keys report of no key or a direction; a later report supersedes it.
    static bool childSuperseded(const uint8_t* p, uint32_t len)
    {
        if (len < 5 || p[3] != 0xBE) return false;  // RFUCMD_SEND_HELD_KEYS only
        const uint8_t key = p[4];
        return key == 0 || (key >= 0x11 && key <= 0x15);
    }

    // The parent reads one child frame per frame, so frames left standing after
    // a burst are lag. Beyond hostShedAt the oldest superseded report is shed
    // (never the newest) and delivery starts stamping tags.
    void shedSuperseded(HostClient& c)
    {
        PktQueue<16, 32>& q = c.pkt;
        while (q.count > hostShedAt)
        {
            uint8_t i = 0;
            while (i + 1 < q.count && !childSuperseded(q.peek(i), q.peekLen(i))) i++;
            if (i + 1 >= q.count) return;
            q.removeAt(i);
            c.restamp = true;
            dbgSheds = dbgSheds + 1;
        }
    }

    // Emit an all-zero carrier frame: the current buffer's shape, or the
    // mode's natural frame size when the last send was zero-length. Live data
    // is never repeated: a repeated child seq trips the parent's dup
    // detection, and repeated non-zero aggregates fill the child game's
    // 20-slot recvQueue.
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
    // carrying the send buffer).
    void emitUniTx()
    {
        if (state == stHost)
        {
            // One frame regardless of client count: the relay fans it
            // out to every connected client, as the real adapter's host
            // payload is broadcast to all children over the air.
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
        m_idStarted = false;
        m_header = 0;
        // A re-run of AgbRFU_checkID always follows AgbRFU_SoftReset, which
        // wipes the adapter's link state on real hardware. Peers are told
        // first so their slots free at once instead of via the 4s timeout.
        resetLinkState();
    }


    // Reset everything the game's soft reset clears, without touching the
    // command-transfer machinery (comstate/m_cmd/m_plen), which the caller
    // handles.
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
        trace(TR_CMD, m_cmd, m_nowMs);
        const int32_t ret = processCommand();
        m_respErr = ret < 0;
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

    // librfu re-runs a master exchange from its header word when a reply is late
    // or unexpected (STWI_restart_Command, at most twice, then a fatal
    // CLOCK_DRIFT error and the game's link error screen). The header is re-sent
    // unchanged, and the GBA never sends a header value while clocking out a
    // response or reading a WAIT-class ACK, so the in-progress header arriving
    // in one of those states is a restart.
    bool isCommandRestart(uint32_t rx) const
    {
        if (rx != m_header || (m_header >> 16) != 0x9966) return false;
        switch (comstate)
        {
            case comWaitDat:
            case comRespDat:
            case comRespErr2:
            case comWaitEvent:
            case comWaitResp:
            case comReplayWait:
            case comReplayDat:
                return true;
            default:
                return false;
        }
    }

    void onCommandRestart()
    {
        counters.commandRestarts++;
        trace(TR_RESTART, static_cast<uint8_t>(comstate), m_nowMs);
        switch (comstate)
        {
            case comWaitDat:
                // The GBA gave up during its own command words. This attempt's
                // header was answered busy, so the attempt goes through:
                // collect again.
                m_cnt = 0;
                break;

            case comWaitEvent:
            case comWaitResp:
                // The WAIT-class ACK was never taken. The command already ran
                // (its frame is out), so answer this attempt without running it
                // again; any event built meanwhile is dropped (runDelivery
                // re-checks the state under lock before taking the bus).
                m_waitArmed = false;
                m_waitAckRead = false;
                m_respErr = false;
                m_plen = 0;
                m_cnt = 0;
                comstate = m_hdrLen ? comReplayDat : comRespCmd;
                break;

            case comReplayWait:
                // The attempt after the failed one: same response again.
                m_cnt = 0;
                comstate = m_hdrLen ? comReplayDat : (m_respErr ? comRespErr : comRespCmd);
                break;

            case comReplayDat:
                m_cnt = 0;
                break;

            default:  // comRespDat, comRespErr2
                // Mid-response: this attempt's header went out against a stale
                // response word, so the GBA fails it too. Answer busy until the
                // next attempt, then replay the response instead of re-running
                // the command: RECV_DATA would hand out the next frame and this
                // one would never reach the game.
                comstate = comReplayWait;
                break;
        }
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Delivery-side transitions: returns the word the GBA reads in the next
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
            {
                // Answer checkID from the word the GBA just sent rather than
                // from a fixed list: it restarts the exchange whenever its own
                // check fails, and while still on the first id its main loop
                // starts extra transfers (librfu Sio32IDMain case 1) whose
                // replies go unread, so a fixed list loses its place.
                //
                // The high half answers the id the GBA is walking; the low half
                // is the complement of the previous high, which is what its
                // handler compares against (Sio32IDIntr). Once the last two
                // replies both carried the current id, its counter advances
                // when it takes the staged one, so the next id is offered
                // early; otherwise it latches the previous id as the adapter's.
                // Skipped on the first id, where unread replies would
                // desynchronise it.
                const uint16_t id = static_cast<uint16_t>(m_idLastRx);
                uint16_t high = id;
                if (m_idStarted && id != ID_NINTENDO[0] &&
                    m_idLastHigh == id && m_idPrevHigh == id)
                {
                    high = idSuccessor(id);
                }
                const uint16_t low = m_idStarted
                                         ? static_cast<uint16_t>(~m_idLastHigh)
                                         : static_cast<uint16_t>(m_idLastRx >> 16);
                m_idPrevHigh = m_idLastHigh;
                m_idLastHigh = high;
                m_idStarted = true;
                return (static_cast<uint32_t>(high) << 16) | low;
            }

            case comWaitCmd:
                return BUSY_WORD;

            case comRespCmd:
            {
                const uint32_t ack = 0x99660080 | m_cmd | (m_plen << 8);
                if (m_cmd == CMD_WAIT || m_cmd == CMD_RTX_WAIT || m_cmd == CMD_SEND_DATAW ||
                    m_cmd == CMD_WAIT2)
                {
                    // Roles flip: the adapter answers next as bus master. The
                    // GBA reads this ACK in-band, as the tail of its own master
                    // REQ/ACK exchange (one more transfer, see m_waitAckRead),
                    // then drops SIOCNT to external clock (slave) and clocks
                    // nothing more (librfu_intr.c:111-124; gpsp rfu.c:639-652).
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
                // Plain ACKs, like gpsp. Resets come from the SD-line pulse
                // (onSdReset; AgbRFU_SoftReset always precedes a real re-init)
                // and the 0x494E checkID escape. 0x10 must not clear link
                // state: the union-room parent sends a routine 0x10 when it
                // restarts its advertise cycle, and one landing right after a
                // child connects would wipe the new slot.
                return 0;

            case CMD_SYSCFG:
                m_syscfg = m_buf[0];
                m_timeoutFrames = static_cast<uint8_t>(m_buf[0]);
                m_rtxMax = static_cast<uint8_t>(m_buf[0] >> 8);
                // Bits 16-17: room size (0=5 players .. 3=2 players). gpsp
                // ignores these; honoring them makes a smaller room reject
                // the surplus joiners.
                m_maxClients = static_cast<uint8_t>(4 - ((m_buf[0] >> 16) & 0x3));
                return 0;

            case CMD_CFGSTAT:
                // Adapter configuration readback (afska wireless_adapter.md):
                // host = broadcast data + the SYSCFG word + 0x101; client = six
                // zeros + 0x101. gpsp returns nothing here, but the real
                // adapter answers.
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
                // this every ~4 frames and four consecutive zero readings for
                // a connected slot force a link-loss disconnect, so full
                // strength is reported from slot occupancy alone: the game's
                // connSlotFlag outlives the transient host/scan state cycling,
                // and a zero during any window is a loss strike.
                m_buf[0] = (m_host.clients[0].devid ? 0x000000FFu : 0) |
                           (m_host.clients[1].devid ? 0x0000FF00u : 0) |
                           (m_host.clients[2].devid ? 0x00FF0000u : 0) |
                           (m_host.clients[3].devid ? 0xFF000000u : 0);
                if (state == stClient)
                    m_buf[0] = 0xFFu << (m_client.clnum * 8);
                dbgLastLinkPwr = m_buf[0];
                // Telemetry counts only the zeros that are watchLink loss
                // strikes: hosting with no clients is a legitimate zero, a
                // zero while linked as a client is not.
                if (m_buf[0] == 0 && state == stClient)
                    dbgLinkPwrZero = dbgLinkPwrZero + 1;
                return 1;

            case CMD_BCRD_START:
                return 0;

            case CMD_BCRD_STOP:
            case CMD_BCRD_FETCH:
            {
                // gpsp picks up to 4 random valid peers; a rotated scan over
                // the 4-entry table keeps the same fairness.
                uint32_t cnt = 0;
                const unsigned start = rand16 ? (rand16(randCtx) % 4) : 0;
                for (unsigned j = 0; j < 4 && cnt < 4 * 7; j++)
                {
                    const auto& p = m_peers[(start + j) % 4];
                    if (p.valid)
                    {
                        // Metadata word: devid + the beacon's next-available-
                        // slot byte (0xFF = full/closed); the game gates
                        // hails on it (partner.slot != 0xFF).
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
                    // using the devid from their snapshot scan list, so a
                    // rotating id invalidates the target between scan and
                    // connect and the connect can never be routed. A real
                    // adapter keeps its session id across EndHost.
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
                    // EndHost closes the room: no more beacons, no new
                    // joiners; existing clients stay linked (FRLG does this
                    // right after a successful join).
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
                // Either side may initiate (a player hails; FRLG never
                // auto-connects). Emit even when the target devid is absent
                // from the peer table: the game hails from a scan snapshot
                // whose beacon may have TTL'd out, and the relay matches
                // against each member's recent devid history. tick() re-emits
                // while stConnecting, so a mid-scan NACK is retried.
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
                    // Clamp to the staging buffer: valid traffic maxes at 23
                    // payload words (90 bytes), but m_plen is GBA-controlled
                    // up to 255.
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
                            // The parent accepts only previous tag + 1; shedding
                            // leaves gaps in the child's numbering.
                            if (childHasCommand(&tmp[bufbytes], dlen))
                            {
                                if (c.restamp)
                                    tmp[bufbytes + 2] = static_cast<uint8_t>((tmp[bufbytes + 2] & 0x1F) | (c.outTag << 5));
                                c.outTag = ((tmp[bufbytes + 2] >> 5) + 1) & 7;
                            }
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
                    // Only occupied slots: the game's slot view lags the
                    // adapter's (clientTimeoutMs frees slots autonomously) and
                    // a devid-0 DISCONNECT would be garbage for the relay.
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
