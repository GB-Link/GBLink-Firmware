#include <cstdint>
#include <cstring>
#include <cstddef>

#pragma once

// Advance Wars 1/2 VS-link protocol proxy.
//
// Port of gpsp's working netplay implementation (gpsp/serial_proto.c,
// serialaw_* functions). Instead of relaying raw link words (which breaks as
// soon as the network adds latency — the GBA expects protocol-correct answers
// every round), each adapter runs this proxy: it impersonates the remote GBA
// locally, answering every MULTI-mode round immediately from its own state,
// and exchanges only whole protocol packets ("MAW1" frames) asynchronously
// over USB → web client → Celio-Server. The games tolerate multi-second waits
// at the packet level (they poll), so network latency only delays packets,
// never corrupts the wire protocol.
//
// The protocol has three phases, driven entirely by the local GBA's words:
//   PACKETXG  - variable-length packet exchange (incl. the save-name exchange
//               at link start; no special handling needed, names are packets)
//   SYNC      - barrier: both sides repeat the variant sync word until each
//               has seen the other side reach SYNC
//   INTERSYNC - realtime key exchange (words 0x8000-0x9F00, bursts terminated
//               by a word with low 10 bits clear)
//
// This header is deliberately Zephyr-free so the FSM and framing can be
// compiled and tested on the host. Concurrency contract (enforced by the
// caller, see awProtocolSection):
//   - onRound() is the only writer of FSM/replay-pop state (PIO done-ISR).
//   - applyNetFrame() pushes to the replay queue (USB thread or UART ISR);
//     the caller must hold irq_lock() around it so it cannot interleave with
//     an onRound() pop.
//   - stagedTx/stagedTiming are written by onRound(), read by the PIO tx-ISR.

namespace awproto
{

constexpr uint16_t CMD_NONE = 0x7FFF;
constexpr uint16_t CMD_NOP  = 0x5FFF;

// "MAW1" — gpsp's netpacket magic (serial_proto.c:400). Keeping the frame
// format byte-compatible with gpsp keeps a future hardware↔emulator bridge
// possible without firmware changes.
constexpr uint8_t MAW1_MAGIC[4] = { 0x4D, 0x41, 0x57, 0x31 };

enum AwState : uint8_t
{
    statePacketXg  = 0,
    stateSync      = 1,
    stateIntersync = 2
};

enum AwParseState : uint8_t
{
    pstateCommands   = 0,
    pstatePacketHdr  = 1,
    pstatePacketBody = 2
};

enum class GameVariant : uint8_t
{
    aw1 = 1,
    aw2 = 2
};

struct AwConfig
{
    uint16_t syncWord;    // AW1: 0x5678, AW2: 0x9ABC
    uint8_t  packTailSz;  // words after the packet payload: AW1: 1, AW2: 2

    static constexpr AwConfig forVariant(GameVariant v)
    {
        return (v == GameVariant::aw2) ? AwConfig{ 0x9ABC, 2 }
                                       : AwConfig{ 0x5678, 1 };
    }
};

// PIO master-mode round pacing, in PIO delay-loop iterations (~542.5 ns each,
// clkdiv 67.816 — see gbaLinkLayer.cpp). Matches gpsp's fake-master rates:
// SLAVE_IRQ_CYCLES_2P (8.3 ms, 2/frame) and SLAVE_IRQ_CYCLES_H (16.7 ms,
// 1/frame, used in SYNC/INTERSYNC). In slave mode the value is ignored — the
// parent GBA paces the bus.
constexpr uint32_t timingPacketXg = 15370;
constexpr uint32_t timingSync     = 30765;

// Frame-level limits. cnt is carried in 8 bits of the MAW1 flags word, so a
// frame can never describe more than 255 words.
constexpr uint16_t maxFrameWords = 255;
constexpr size_t   maxFrameBytes = 8 + 2 * static_cast<size_t>(maxFrameWords);

// Replay queue size in words, same as gpsp's MAX_FPACK.
constexpr uint16_t replayQueueWords = 512;

// Packet accumulator: max packet = size word (1) + payload (255) + tail (2).
constexpr uint16_t accumWords = 300;

// Suppressed duplicate announcements are re-sent at least this often so the
// remote's link timeout is fed even when nothing changes.
constexpr uint32_t keepaliveMs = 250;

// Quiet time after which the link is reported as reconnecting. Matches
// gpsp's MAX_FRAME_TIMEOUT (240 frames ≈ 4 s) — which gpsp defines but never
// wired up for AW; here it drives a status report only, never a protocol
// change (the game's own link-error timeout is the final arbiter).
constexpr uint32_t linkTimeoutMs = 4000;

struct AwCounters
{
    uint32_t rounds = 0;
    uint32_t txFrames = 0;
    uint32_t txSuppressed = 0;     // deduplicated cnt==0 announcements
    uint32_t txDropOversize = 0;   // frame would exceed maxFrameWords
    uint32_t txDropRingFull = 0;   // outbound ring had no room for the frame
    uint32_t rxFrames = 0;
    uint32_t rxDropQueueFull = 0;
    uint32_t fallbackEcho = 0;     // INTERSYNC rounds answered with lastcmd & 0xFC00
    uint16_t rqHighWater = 0;
};

inline size_t maw1Serialize(uint8_t* out, uint16_t cmd, uint8_t state,
                            const uint16_t* words, uint16_t cnt)
{
    out[0] = MAW1_MAGIC[0];
    out[1] = MAW1_MAGIC[1];
    out[2] = MAW1_MAGIC[2];
    out[3] = MAW1_MAGIC[3];
    // flags = (cmd << 16) | (state << 8) | cnt, big endian on the wire
    out[4] = static_cast<uint8_t>(cmd >> 8);
    out[5] = static_cast<uint8_t>(cmd & 0xFF);
    out[6] = state;
    out[7] = static_cast<uint8_t>(cnt);
    for (uint16_t i = 0; i < cnt; i++)
    {
        out[8 + 2 * i]     = static_cast<uint8_t>(words[i] >> 8);
        out[8 + 2 * i + 1] = static_cast<uint8_t>(words[i] & 0xFF);
    }
    return 8 + 2 * static_cast<size_t>(cnt);
}

// Reassembles MAW1 frames from an arbitrary byte stream (the network path
// chunks frames into 51-byte transport packets and pads the tail of a chunk
// with zeros; zeros never match the magic, so the parser self-synchronizes).
class Maw1StreamParser
{
public:
    using FrameFn = void(*)(void* ctx, uint16_t cmd, uint8_t state,
                            const uint16_t* words, uint16_t cnt);

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
            if (b == MAW1_MAGIC[m_magicIdx]) m_magicIdx++;
            else m_magicIdx = (b == MAW1_MAGIC[0]) ? 1 : 0;
            return;
        }

        if (m_hdrIdx < 4)
        {
            m_hdr[m_hdrIdx++] = b;
            if (m_hdrIdx == 4)
            {
                m_cmd   = static_cast<uint16_t>((m_hdr[0] << 8) | m_hdr[1]);
                m_state = m_hdr[2];
                m_cnt   = m_hdr[3];
                m_payloadIdx = 0;
                if (m_cnt == 0) finishFrame();
            }
            return;
        }

        if ((m_payloadIdx & 1) == 0) m_words[m_payloadIdx / 2] = static_cast<uint16_t>(b) << 8;
        else                         m_words[m_payloadIdx / 2] |= b;
        m_payloadIdx++;
        if (m_payloadIdx == 2 * static_cast<uint16_t>(m_cnt)) finishFrame();
    }

    void finishFrame()
    {
        if (m_cb) m_cb(m_ctx, m_cmd, m_state, m_words, m_cnt);
        m_magicIdx = 0;
        m_hdrIdx = 0;
    }

    FrameFn m_cb = nullptr;
    void* m_ctx = nullptr;

    uint8_t  m_magicIdx = 0;
    uint8_t  m_hdrIdx = 0;
    uint8_t  m_hdr[4] = {};
    uint16_t m_cmd = 0;
    uint8_t  m_state = 0;
    uint8_t  m_cnt = 0;
    uint16_t m_payloadIdx = 0;
    uint16_t m_words[maxFrameWords] = {};
};

struct AwProxy
{
    using EmitFn = void(*)(void* ctx, uint16_t cmd, uint8_t state,
                           const uint16_t* words, uint16_t cnt);

    AwConfig cfg = AwConfig::forVariant(GameVariant::aw1);
    EmitFn emit = nullptr;
    void* emitCtx = nullptr;

    // Role-faithful porting flag. gpsp runs serialaw_master_send() against
    // the master GBA and serialaw_update() against slave GBAs, and the two
    // differ in exactly two ways (both preserved here, selected by role):
    //   - update() re-processes the word that exits the SYNC barrier through
    //     the new state's handler in the same round; master_send() does not.
    //   - master_send() tracks lastcmd on every word; update() only on
    //     in-range INTERSYNC words.
    // gbaIsMaster=true  → adapter in SLAVE link mode, GBA is bus master
    //                     → master_send() semantics.
    // gbaIsMaster=false → adapter in MASTER link mode, GBA is bus slave
    //                     → update() semantics.
    bool gbaIsMaster = true;

    // Local GBA proxy state (≙ gpsp serstate.aw.peer[self])
    uint8_t  state = statePacketXg;
    uint8_t  pstate = pstateCommands;
    uint16_t accum[accumWords] = {};
    uint16_t accumCount = 0;
    uint16_t lastcmd = 0;

    // Remote peer (≙ gpsp serstate.aw.peer[remote])
    volatile uint8_t  remoteState = statePacketXg;
    volatile uint32_t lastHeardMs = 0;   // 0 = never heard from the remote
    uint16_t rq[replayQueueWords] = {};
    uint16_t rqCount = 0;
    uint16_t rqRecvd = 0;

    // Response staged for the next wire round. The PIO loads the transmit
    // word at the start of a round — before the current round's incoming
    // word is visible — so every response is computed one round in advance
    // (exactly how a real GBA child preloads SIOMLT_SEND).
    volatile uint16_t stagedTx = CMD_NONE;
    volatile uint32_t stagedTiming = timingPacketXg;

    AwCounters counters;

    // Announcement dedup (see emitFrame)
    uint8_t  lastSentState = 0xFF;
    uint32_t lastTxMs = 0;
    bool     sentAnything = false;

    //-//////////////////////////////////////////////////////////////////////-//
    // Wire round: FSM step on the word received from the local GBA, then
    // stage the response for the following round.
    // Direct port of gpsp serialaw_master_send()/serialaw_update() — the two
    // are the same FSM, they only differ in who paces the rounds.
    //-//////////////////////////////////////////////////////////////////////-//

    void onRound(uint16_t mvalue, uint32_t nowMs)
    {
        counters.rounds++;

        if (state == stateSync)
        {
            if (mvalue == CMD_NONE)
            {
                state = statePacketXg;
                if (!gbaIsMaster) handlePacketXg(mvalue, nowMs);
            }
            else
            {
                if (mvalue != cfg.syncWord && mvalue != CMD_NOP)
                {
                    // First non-sync word: the game has seen our partner's
                    // sync response and moves to realtime exchange.
                    state = stateIntersync;
                    accumCount = 0;
                }
                emitFrame(0, state, nullptr, 0, nowMs);
                if (!gbaIsMaster && state == stateIntersync)
                    handleIntersync(mvalue, nowMs);
            }
        }
        else if (state == stateIntersync)
        {
            handleIntersync(mvalue, nowMs);
        }
        else
        {
            handlePacketXg(mvalue, nowMs);
        }

        if (gbaIsMaster) lastcmd = mvalue;

        stageNextTx();
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Network frame from the remote adapter (≙ gpsp serialaw_net_receive).
    // Caller must serialize against onRound() (irq_lock).
    //-//////////////////////////////////////////////////////////////////////-//

    void applyNetFrame(uint16_t cmd, uint8_t ste, const uint16_t* words,
                       uint16_t cnt, uint32_t nowMs)
    {
        counters.rxFrames++;
        remoteState = ste;
        // 0 means "never heard"; avoid storing it for a wrap-around tick.
        lastHeardMs = nowMs ? nowMs : 1;

        if (ste == stateIntersync && cnt >= 2)
        {
            if (rqCount + cnt + 1 < replayQueueWords)
            {
                rq[rqCount++] = cnt;
                std::memcpy(&rq[rqCount], words, cnt * sizeof(uint16_t));
                rqCount += cnt;
            }
            else counters.rxDropQueueFull++;
        }
        else if (ste == statePacketXg && cnt >= 2)
        {
            if (rqCount + cnt + 2 < replayQueueWords)
            {
                // Replayed as the remote GBA sent it on the wire: the 0x4Fxx
                // command word first, then size + payload + tail.
                rq[rqCount++] = cnt + 1;
                rq[rqCount++] = cmd;
                std::memcpy(&rq[rqCount], words, cnt * sizeof(uint16_t));
                rqCount += cnt;
            }
            else counters.rxDropQueueFull++;
        }
        // cnt < 2 frames are state announcements / keepalives only, same as
        // gpsp's cnt >= 2 guards.

        if (rqCount > counters.rqHighWater) counters.rqHighWater = rqCount;
    }

private:

    void handleIntersync(uint16_t mvalue, uint32_t nowMs)
    {
        if (mvalue >= 0x8000 && mvalue <= 0x9F00)
        {
            if (mvalue & 0x3FF)
            {
                if (accumCount < accumWords) accum[accumCount++] = mvalue;
            }
            else
            {
                // Low 10 bits clear terminates a burst; the cmd field carries
                // the previous word (receivers ignore it for INTERSYNC).
                emitFrame(lastcmd, stateIntersync, accum, accumCount, nowMs);
                accumCount = 0;
            }
            lastcmd = mvalue;
        }
        else if (mvalue != CMD_NOP)
        {
            state = statePacketXg;
        }
    }

    void handlePacketXg(uint16_t mvalue, uint32_t nowMs)
    {
        switch (pstate)
        {
            case pstateCommands:
                if ((mvalue >> 8) == 0x4F)
                {
                    pstate = pstatePacketHdr;
                }
                else if (mvalue == cfg.syncWord && rqCount == 0)
                {
                    // Only enter the sync barrier once the remote's queued
                    // packets have been fully replayed.
                    state = stateSync;
                }
                else
                {
                    emitFrame(mvalue, statePacketXg, nullptr, 0, nowMs);
                }
                break;

            case pstatePacketHdr:
                accum[0] = mvalue & 0xFF;
                accumCount = 1;
                pstate = pstatePacketBody;
                break;

            default:  // pstatePacketBody
            {
                const uint16_t pktlen = accum[0] + cfg.packTailSz;
                if (accumCount < accumWords) accum[accumCount++] = mvalue;
                if (accumCount == pktlen + 1)
                {
                    pstate = pstateCommands;
                    emitFrame(0x4FFF, statePacketXg, accum, accumCount, nowMs);
                }
                break;
            }
        }
    }

    void stageNextTx()
    {
        uint16_t tx;
        switch (state)
        {
            case stateSync:
                tx = (remoteState >= stateSync) ? cfg.syncWord : CMD_NONE;
                break;

            case stateIntersync:
            {
                const uint16_t v = popReplayVal();
                if (v) tx = v;
                else
                {
                    // gpsp answers idle INTERSYNC rounds with the command
                    // class of the current word (mvalue & 0xFC00). One round
                    // behind, lastcmd is that word, so this echoes the same
                    // class except on the round right after a class change —
                    // gpsp's own CMD_NOP path uses lastcmd & 0xFC00 too, so
                    // the game accepts class-stale echoes.
                    tx = lastcmd & 0xFC00;
                    counters.fallbackEcho++;
                }
                break;
            }

            default:  // statePacketXg
                tx = popReplay();
                break;
        }

        stagedTx = tx;
        stagedTiming = (state >= stateSync) ? timingSync : timingPacketXg;
    }

    // ≙ gpsp process_awpeer_val (serial_proto.c:438): replay queued INTERSYNC
    // words; idle → 0 (never a valid INTERSYNC word, callers use it as "no
    // data").
    uint16_t popReplayVal()
    {
        if (rqRecvd == 0 && rqCount > 0) rqRecvd = 1;
        if (rqRecvd == 0) return 0;

        const uint16_t numw = rq[0];
        if (rqRecvd >= numw)
        {
            const uint16_t ret = rq[rqRecvd];
            popEntry(numw);
            return ret;
        }
        return rq[rqRecvd++];
    }

    // ≙ gpsp process_awpeer (serial_proto.c:462): replay queued packets with
    // a trailing CMD_NONE per packet; idle → CMD_SYNC while the remote sits
    // at the sync barrier, CMD_NONE otherwise.
    uint16_t popReplay()
    {
        if (rqRecvd == 0 && rqCount > 0) rqRecvd = 1;
        if (rqRecvd == 0)
        {
            // gpsp gates the sync answer on peer[0].pstate == COMMANDS. On
            // its master instance peer[0] is the local parse state; on slave
            // instances peer[0].pstate is never written and the gate is
            // always true. Mirror that per role.
            const bool betweenPackets = !gbaIsMaster || (pstate == pstateCommands);
            return (betweenPackets && remoteState == stateSync)
                       ? cfg.syncWord : CMD_NONE;
        }

        const uint16_t numw = rq[0];
        if (rqRecvd > numw)
        {
            popEntry(numw);
            return CMD_NONE;
        }
        return rq[rqRecvd++];
    }

    void popEntry(uint16_t numw)
    {
        rqRecvd = 0;
        rqCount -= numw + 1;
        std::memmove(&rq[0], &rq[numw + 1], rqCount * sizeof(uint16_t));
    }

    void emitFrame(uint16_t cmd, uint8_t ste, const uint16_t* words,
                   uint16_t cnt, uint32_t nowMs)
    {
        if (cnt > maxFrameWords)
        {
            // cnt is 8 bits on the wire; a wrapped count would desync the
            // remote parser. Real AW packets stay far below this.
            counters.txDropOversize++;
            return;
        }

#ifndef AWPROTO_NO_DEDUP
        // gpsp emits an announcement for every idle round (~120/s); each of
        // ours becomes a Socket.IO message. Receivers ignore the payload of
        // cnt==0 frames (only the state field matters), so suppress repeats
        // and re-send as a keepalive instead.
        if (cnt == 0 && sentAnything && ste == lastSentState &&
            (nowMs - lastTxMs) < keepaliveMs)
        {
            counters.txSuppressed++;
            return;
        }
#endif

        if (emit) emit(emitCtx, cmd, ste, words, cnt);
        counters.txFrames++;
        lastSentState = ste;
        lastTxMs = nowMs;
        sentAnything = true;
    }
};

}  // namespace awproto
