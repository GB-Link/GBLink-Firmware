#include <cstdint>
#include <cstring>
#include <cstddef>

#pragma once

// Advance Wars 1/2 VS-link protocol proxy, for two to four players.
//
// The games expect a protocol-correct word from every player in every
// transfer, so the words can't be relayed over a network. Each adapter runs
// this proxy instead: it stands in for the other players' GBAs, answering
// every transfer at once from what it knows of them, and exchanges only whole
// packets and key presses ("MAW1" frames, the format of gpsp's netplay) over
// USB → web client → Celio-Server. The games poll at that level, so network
// latency delays them but never breaks a transfer.
//
// The local GBA's words drive three phases:
//   PACKETXG  - packets: the names when the link starts, the settings and a
//               match's moves
//   SYNC      - barrier: every GBA repeats the variant's sync word until all
//               players have reached it
//   INTERSYNC - the players' keys on the screens they go through together
//               (words 0x8000-0x9F00: the low 10 bits are the keys, so a
//               press is a run of words ending with one whose low 10 bits
//               are clear)
//
// The rules follow the games' own code:
//   - A packet is 0x4FFF, a size up to 0x80, one checksum word (two in
//     Advance Wars 2) and as many data words as the size says. A start word
//     followed by a larger size is no packet: a game that drops what it was
//     about to send leaves a start word that only idle words follow.
//   - In the key exchange a game's logic runs once per transfer, on the words
//     of all players. A transfer with a word that is no key word in any slot
//     counts for nobody, so the other players' slots always carry one.
//
// This header is Zephyr-free so the proxy and the framing can be compiled and
// tested on the host. Concurrency contract (see awProtocolSection):
//   - onRound() runs in the PIO done-ISR and is the only writer of the proxy's
//     own state and of the queues' read side.
//   - applyNetFrame() appends to a queue (USB thread or UART ISR); the caller
//     holds irq_lock() around it so it cannot interleave with onRound().
//   - stagedTx/stagedTiming are written by onRound(), read by the PIO tx-ISR.

namespace awproto
{

constexpr uint16_t CMD_NONE = 0x7FFF;
constexpr uint16_t CMD_NOP  = 0x5FFF;

constexpr uint16_t packetStart = 0x4FFF;
constexpr int16_t  maxPacketSize = 0x80;

constexpr uint8_t maxPeers = 3;

// "MAW1", the magic of gpsp's netpackets (serial_proto.c).
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
    uint16_t syncWord;       // AW1: 0x5678, AW2: 0x9ABC
    uint8_t  checksumWords;  // between a packet's size and its data: AW1: 1, AW2: 2

    static constexpr AwConfig forVariant(GameVariant v)
    {
        return (v == GameVariant::aw2) ? AwConfig{ 0x9ABC, 2 }
                                       : AwConfig{ 0x5678, 1 };
    }
};

// Gap before each transfer as the bus master, in PIO delay-loop iterations
// (~542.5 ns each). A parent GBA transfers twice a frame while packets are
// exchanged and once a frame from the sync barrier on. The gap comes on top of
// the transfer itself, so a child GBA is clocked a little slower than once a
// frame: its game waits for a transfer every frame, while two transfers in one
// frame leave it without a word for the second, which costs every player a
// step of that frame's keys. In slave mode the value is ignored: the parent
// GBA paces the bus.
constexpr uint32_t timingPacketXg = 15370;
constexpr uint32_t timingSync     = 30765;

// A frame's word count travels in 8 bits.
constexpr uint16_t maxFrameWords = 255;
constexpr size_t   maxFrameBytes = 8 + 2 * static_cast<size_t>(maxFrameWords);

// Words of one player that wait to be replayed, with one header word per
// packet or press.
constexpr uint16_t queueWords = 1024;

// A press that lasts is sent in parts of this many transfers (one a frame), so
// it has no longest length. Every part but the last has keysContinue for its
// command. The replay of such a press starts a little late, so that its next
// part is there in time, and its keys stay held if a part is late all the same.
constexpr uint16_t longPressPart     = 120;
constexpr uint16_t keysContinue      = 0xFFFE;
constexpr uint8_t  longPressLead     = 30;
constexpr uint16_t longPressPatience = 600;

// The largest packet (size, checksums, data) and the largest part of a press.
constexpr uint16_t accumWords = 1 + 2 + 0x80;

// A repeated state announcement is sent again this often, so the others still
// hear from this adapter.
constexpr uint32_t keepaliveMs = 250;

// Without a frame from a player for this long the link is reported as
// reconnecting. A status only: the GBA is answered either way.
constexpr uint32_t linkTimeoutMs = 4000;

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

// Reassembles MAW1 frames from one player's byte stream. The stream arrives
// in transport packets whose unused tail is zero; zeros never match the magic,
// so the parser finds the next frame by itself.
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

    // What is known of another player's GBA, and what it sent that waits to
    // be replayed to the local GBA.
    struct Peer
    {
        volatile uint8_t  state = statePacketXg;
        volatile uint32_t lastHeardMs = 0;   // 0 = never heard from

        // Entries of a header word (entryKeys, entryContinues, word count)
        // followed by the words: a packet as the player's GBA sent it, or the
        // keys of a press.
        uint16_t queue[queueWords] = {};
        uint16_t count = 0;      // words in the queue, headers included
        uint16_t position = 0;   // words of the first entry the GBA has been given

        bool     released = false;   // a press was just replayed to its end
        uint16_t held = 0;           // key word of a long press whose next part is awaited
        uint16_t heldFor = 0;
        uint8_t  lead = 0;           // transfers a long press has waited to start
    };

    AwConfig cfg = AwConfig::forVariant(GameVariant::aw1);
    EmitFn emit = nullptr;
    void* emitCtx = nullptr;

    // gbaIsMaster=true:  the adapter is in SLAVE link mode, the GBA is the bus
    //                    master (player 1).
    // gbaIsMaster=false: the adapter is the bus master, the GBA a child.
    // Two details differ: a child's word that leaves the sync barrier for
    // packets is handled again as one, and only a master GBA has to be between
    // packets to be shown that another player waits at the sync barrier.
    bool gbaIsMaster = true;

    // The other players, in seat order.
    uint8_t peerCount = 1;
    Peer peers[maxPeers];

    // The local GBA's phase.
    volatile uint8_t state = statePacketXg;

    // The other players' words for the next transfer, in seat order. The PIO
    // loads them at the start of a transfer, before the GBA's word in it is
    // known, so every answer is computed one transfer ahead.
    volatile uint16_t stagedTx[maxPeers] = { CMD_NONE, CMD_NONE, CMD_NONE };
    volatile uint32_t stagedTiming = timingPacketXg;

    //-//////////////////////////////////////////////////////////////////////-//
    // A finished transfer with the local GBA's word: follows the game and
    // stages the next transfer.
    //-//////////////////////////////////////////////////////////////////////-//

    void onRound(uint16_t word, uint32_t nowMs)
    {
        if (state == stateSync)
        {
            if (word == CMD_NONE)
            {
                leaveKeys();
                if (!gbaIsMaster) handlePacketXg(word, nowMs);
            }
            else
            {
                // Any other word: the game saw everyone at the barrier and
                // goes on to exchange keys.
                const bool passed = word != cfg.syncWord && word != CMD_NOP;
                if (passed) enterIntersync();
                emitFrame(0, passed ? stateIntersync : stateSync, nullptr, 0, nowMs);
                if (passed) handleIntersync(word, nowMs);
            }
        }
        else if (state == stateIntersync)
        {
            handleIntersync(word, nowMs);
        }
        else
        {
            handlePacketXg(word, nowMs);
        }

        stageNextTx();
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // A frame from another player. The caller serializes against onRound()
    // (irq_lock).
    //-//////////////////////////////////////////////////////////////////////-//

    void applyNetFrame(uint8_t peerIndex, uint16_t cmd, uint8_t ste,
                       const uint16_t* words, uint16_t cnt, uint32_t nowMs)
    {
        if (peerIndex >= peerCount) return;
        Peer& peer = peers[peerIndex];
        peer.state = ste;
        // 0 means "never heard"; avoid storing it for a wrap-around tick.
        peer.lastHeardMs = nowMs ? nowMs : 1;

        // A frame without words only announces the player's state.
        if (cnt == 0) return;

        if (ste == stateIntersync)
        {
            // Keys are for a game that exchanges keys, or waits at the barrier
            // for the screens to come.
            if (state == statePacketXg && !atBarrier) return;
            pushEntry(peer, entryKeys | (cmd == keysContinue ? entryContinues : 0), nullptr, words, cnt);
        }
        else if (ste == statePacketXg && cnt >= 2)
        {
            // Replayed as the player's GBA sent it: the start word, then the
            // size, the checksum and the data.
            pushEntry(peer, 0, &cmd, words, cnt);
        }
    }

private:

    static constexpr uint16_t entryKeys      = 0x8000;
    static constexpr uint16_t entryContinues = 0x4000;
    static constexpr uint16_t entryLength    = 0x3FFF;

    uint8_t  pstate = pstateCommands;
    // The game repeats the sync word while packets of the others still wait
    // to be replayed to it.
    bool     atBarrier = false;
    // The packet or the press the game is sending.
    uint16_t accum[accumWords] = {};
    uint16_t accumCount = 0;
    uint16_t bodyWords = 0;   // words of the packet being sent that follow its size
    uint8_t  syncRun = 0;     // sync words in a row inside a packet
    // The game's last key word: its screen number is what the other players
    // are shown at when they press nothing.
    uint16_t lastcmd = 0;
    // The game's keys of the last transfer and of the one before, as it
    // counts them.
    uint16_t held = 0;
    uint16_t heldBefore = 0;

    // Announcement dedup (see emitFrame)
    uint8_t  lastSentState = 0xFF;
    uint32_t lastTxMs = 0;
    bool     sentAnything = false;

    static bool isKeyWord(uint16_t word) { return word >= 0x8000 && word <= 0x9F00; }

    //-//////////////////////////////////////////////////////////////////////-//
    // The local GBA's words
    //-//////////////////////////////////////////////////////////////////////-//

    void handleIntersync(uint16_t word, uint32_t nowMs)
    {
        if (word == CMD_NOP)
        {
            // The game had no word ready: a transfer without it counts for
            // nobody, and the game goes on with every player's keys of the
            // transfer before the last. For the others that is so for this
            // game's keys.
            if (lastcmd) keyWord((lastcmd & 0xFC00) | (heldBefore & 0x3FF), nowMs);
        }
        else if (isKeyWord(word) && word != cfg.syncWord)
        {
            keyWord(word, nowMs);
            lastcmd = word;
        }
        else
        {
            // The game is back to packets, and this is the first word of
            // them: a packet's start, or the sync word (Advance Wars 2's looks
            // like a key word, with keys no GBA can press together).
            leaveKeys();
            handlePacketXg(word, nowMs);
        }
    }

    // The game's keys of one transfer: a press is collected until the keys
    // are released.
    void keyWord(uint16_t word, uint32_t nowMs)
    {
        heldBefore = held;
        held = word;
        if (word & 0x3FF)
        {
            accum[accumCount++] = word;
            // The last part of a press keeps two words.
            if (accumCount == longPressPart + 2)
            {
                emitFrame(keysContinue, stateIntersync, accum, longPressPart, nowMs);
                accum[0] = accum[longPressPart];
                accum[1] = accum[longPressPart + 1];
                accumCount = 2;
            }
        }
        else
        {
            // The keys are released: the press goes out.
            emitFrame(lastcmd, stateIntersync, accum, accumCount, nowMs);
            accumCount = 0;
        }
    }

    // The game is done with the screens its keys were for: what is left of
    // the others' keys came too late.
    void leaveKeys()
    {
        state = statePacketXg;
        for (uint8_t i = 0; i < peerCount; i++)
        {
            removeEntries(peers[i], true);
            peers[i].released = false;
            peers[i].held = 0;
            peers[i].lead = 0;
        }
    }

    void enterIntersync()
    {
        state = stateIntersync;
        atBarrier = false;
        accumCount = 0;
        held = heldBefore = 0;
        // Packets still queued were sent before their player reached the
        // barrier: repeats the game no longer reads.
        for (uint8_t i = 0; i < peerCount; i++) removeEntries(peers[i], false);
    }

    // True when the game left a packet unfinished to wait at the sync barrier:
    // it repeats the sync word (a child with CMD_NOP in between).
    bool packetGivenUp(uint16_t word)
    {
        if (word == cfg.syncWord) syncRun++;
        else if (word != CMD_NOP || syncRun == 0) syncRun = 0;
        if (syncRun < 2) return false;
        syncRun = 0;
        return true;
    }

    bool packetsQueued() const
    {
        for (uint8_t i = 0; i < peerCount; i++)
        {
            const Peer& peer = peers[i];
            for (uint16_t at = 0; at < peer.count; at += (peer.queue[at] & entryLength) + 1)
            {
                if (!(peer.queue[at] & entryKeys)) return true;
            }
        }
        return false;
    }

    void handlePacketXg(uint16_t word, uint32_t nowMs)
    {
        if (pstate != pstateCommands && packetGivenUp(word))
        {
            accumCount = 0;
            pstate = pstateCommands;
        }

        switch (pstate)
        {
            case pstateCommands:
                if (word == packetStart)
                {
                    atBarrier = false;
                    syncRun = 0;
                    pstate = pstatePacketHdr;
                }
                else if (word == cfg.syncWord)
                {
                    if (!packetsQueued())
                    {
                        // The barrier is only entered once the game has been
                        // given everyone's packets.
                        atBarrier = false;
                        state = stateSync;
                    }
                    else
                    {
                        // The others are told at once, as on a cable: a leader
                        // repeats its last packets until it sees everyone at
                        // the barrier, so waiting for its packets to run out
                        // could wait forever.
                        atBarrier = true;
                        emitFrame(0, stateSync, nullptr, 0, nowMs);
                    }
                }
                else if (word == CMD_NOP && atBarrier)
                {
                    // A child's game sends this between two sync words.
                }
                else if (atBarrier && isKeyWord(word))
                {
                    // The game saw everyone at the barrier in the words it was
                    // still being given, and exchanges keys already.
                    enterIntersync();
                    emitFrame(0, stateIntersync, nullptr, 0, nowMs);
                    handleIntersync(word, nowMs);
                }
                else
                {
                    atBarrier = false;
                    emitFrame(word, statePacketXg, nullptr, 0, nowMs);
                }
                break;

            case pstatePacketHdr:
            {
                // The size, signed. With a larger one this is no packet, and
                // the word is looked at again for what else it is.
                const int16_t size = static_cast<int16_t>(word);
                if (size > maxPacketSize)
                {
                    pstate = pstateCommands;
                    handlePacketXg(word, nowMs);
                    break;
                }
                accum[0] = word;
                accumCount = 1;
                bodyWords = static_cast<uint16_t>(size > 0 ? size : 0) + cfg.checksumWords;
                pstate = pstatePacketBody;
                break;
            }

            default:  // pstatePacketBody
                // By count, as the games take it: nothing inside a packet ends
                // it early.
                accum[accumCount++] = word;
                if (accumCount == bodyWords + 1)
                {
                    pstate = pstateCommands;
                    emitFrame(packetStart, statePacketXg, accum, accumCount, nowMs);
                }
                break;
        }
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // The other players' words
    //-//////////////////////////////////////////////////////////////////////-//

    void stageNextTx()
    {
        for (uint8_t i = 0; i < peerCount; i++)
        {
            Peer& peer = peers[i];
            uint16_t tx;
            switch (state)
            {
                case stateSync:
                    tx = (peer.state >= stateSync) ? cfg.syncWord : CMD_NONE;
                    break;

                case stateIntersync:
                    // Without keys of the player, a key word of the game's own
                    // screen number.
                    tx = replayKeys(peer);
                    if (!tx) tx = lastcmd & 0xFC00;
                    break;

                default:  // statePacketXg
                    tx = replayPacket(peer);
                    break;
            }
            stagedTx[i] = tx;
        }
        stagedTiming = (state >= stateSync) ? timingSync : timingPacketXg;
    }

    // The next word of the player's queued keys, 0 without any.
    uint16_t replayKeys(Peer& peer)
    {
        // Two presses that arrive together must not run into one long press.
        if (peer.released)
        {
            peer.released = false;
            return 0;
        }

        // A packet is for after the keys: its player has gone on already.
        if (peer.count == 0 || !(peer.queue[0] & entryKeys))
        {
            if (!peer.held) return 0;
            if (++peer.heldFor > longPressPatience) peer.held = 0;
            return peer.held;
        }

        const bool continues = peer.queue[0] & entryContinues;
        const uint16_t length = peer.queue[0] & entryLength;
        if (peer.position == 0 && continues && !peer.held && peer.lead < longPressLead)
        {
            peer.lead++;
            return 0;
        }

        const uint16_t word = peer.queue[1 + peer.position++];
        if (peer.position == length)
        {
            dropEntry(peer);
            peer.lead = 0;
            peer.heldFor = 0;
            peer.held = continues ? word : 0;
            peer.released = !continues;
        }
        return word;
    }

    // The next word of the player's queued packets, each followed by one
    // CMD_NONE. Without any: the sync word while the player waits at the
    // barrier, else CMD_NONE.
    uint16_t replayPacket(Peer& peer)
    {
        // Keys in the queue are for the screens after the barrier, from a
        // player who is through it already; a game that left the barrier
        // again won't get there.
        if (!atBarrier && peer.count && (peer.queue[0] & entryKeys)) removeEntries(peer, true);

        if (peer.count == 0 || (peer.queue[0] & entryKeys))
        {
            const bool betweenPackets = !gbaIsMaster || pstate == pstateCommands;
            // A player who is through the barrier already was at it: a game
            // still waiting there sees it so.
            const bool waits = peer.state == stateSync || (atBarrier && peer.state == stateIntersync);
            return (betweenPackets && waits) ? cfg.syncWord : CMD_NONE;
        }

        const uint16_t length = peer.queue[0] & entryLength;
        if (peer.position < length) return peer.queue[1 + peer.position++];
        dropEntry(peer);
        return CMD_NONE;
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Queue
    //-//////////////////////////////////////////////////////////////////////-//

    static void pushEntry(Peer& peer, uint16_t flags, const uint16_t* first,
                          const uint16_t* words, uint16_t cnt)
    {
        const uint16_t length = cnt + (first ? 1 : 0);
        if (peer.count + length + 1 > queueWords) return;
        uint16_t* entry = &peer.queue[peer.count];
        *entry++ = flags | length;
        if (first) *entry++ = *first;
        std::memcpy(entry, words, cnt * sizeof(uint16_t));
        peer.count += length + 1;
    }

    static void dropEntry(Peer& peer)
    {
        const uint16_t size = (peer.queue[0] & entryLength) + 1;
        peer.count -= size;
        std::memmove(&peer.queue[0], &peer.queue[size], peer.count * sizeof(uint16_t));
        peer.position = 0;
    }

    // Takes the presses (keys) or the packets out of a player's queue.
    static void removeEntries(Peer& peer, bool keys)
    {
        uint16_t kept = 0;
        for (uint16_t at = 0; at < peer.count;)
        {
            const uint16_t size = (peer.queue[at] & entryLength) + 1;
            if (static_cast<bool>(peer.queue[at] & entryKeys) != keys)
            {
                std::memmove(&peer.queue[kept], &peer.queue[at], size * sizeof(uint16_t));
                kept += size;
            }
            at += size;
        }
        peer.count = kept;
        peer.position = 0;
    }

    //-//////////////////////////////////////////////////////////////////////-//
    // Frames to the other players
    //-//////////////////////////////////////////////////////////////////////-//

    void emitFrame(uint16_t cmd, uint8_t ste, const uint16_t* words,
                   uint16_t cnt, uint32_t nowMs)
    {
        // The game announces its state in every idle transfer; the others only
        // need to hear a change, and that this adapter is still there.
        if (cnt == 0 && sentAnything && ste == lastSentState &&
            (nowMs - lastTxMs) < keepaliveMs)
        {
            return;
        }

        if (emit) emit(emitCtx, cmd, ste, words, cnt);
        lastSentState = ste;
        lastTxMs = nowMs;
        sentAnything = true;
    }
};

}  // namespace awproto
