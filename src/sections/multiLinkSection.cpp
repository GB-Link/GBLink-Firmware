#include "multiLinkSection.hpp"

#include <cstring>
#include <zephyr/kernel.h>

#include "../layers/transport.hpp"
#include "../linkStatus.hpp"
#include "../link_defines.h"

// Per-seat inbound command queues and the outbound queue of windows captured
// from the local cart. Inbound entries are [gen, pad, 16 bytes of window
// words]: commands are bound to their session generation at enqueue time and
// entries from dead generations are discarded at dequeue. The queues survive
// GO re-arms so that commands sent by peers that established slightly earlier
// (e.g. the master's one-shot exchange request) are not lost.
K_MSGQ_DEFINE(g_mlSeatQ0, 18, 256, 2);
K_MSGQ_DEFINE(g_mlSeatQ1, 18, 256, 2);
K_MSGQ_DEFINE(g_mlSeatQ2, 18, 256, 2);
K_MSGQ_DEFINE(g_mlSeatQ3, 18, 256, 2);
K_MSGQ_DEFINE(g_mlOutQ,   16, 64, 2);

namespace
{
    struct k_msgq* const kSeatQ[4] = { &g_mlSeatQ0, &g_mlSeatQ1, &g_mlSeatQ2, &g_mlSeatQ3 };

    // Network frame tags (low 2 bits = seat where applicable).
    constexpr uint8_t kTagPresence = 0xA0;  // [tag, handshakeReady]
    constexpr uint8_t kTagCommand  = 0xA4;  // [tag, gen, 8 x u16 command words]
    constexpr uint8_t kTagWindows  = 0xA8;  // [tag, gen, u32 window counter]
    constexpr uint8_t kTagGo       = 0xAC;  // [tag, playerCount, gen]

    // Pacing, in 540ns PIO cycles. Only joiner buses use these (seat 0's cart
    // paces its own bus). A real gen-3 master runs ~751us/word and one window
    // per game frame; the servo trims toward seat 0's actual window rate so
    // the per-seat queues stay near empty during sustained command streams.
    constexpr uint32_t kTimingHandshake = 30097;  // one handshake word per game frame
    constexpr uint32_t kTimingCommand   = 1400;   // ~756 us between window words
    constexpr int32_t  kPaceGain        = 96;     // cycles per window of drift
    constexpr int32_t  kPaceAdjustMax   = 2400;   // servo authority

    // Close detection thresholds. Armed (the cart announced the close with
    // LINKCMD_READY_CLOSE_LINK) reacts within a few frames. Unarmed
    // thresholds are wall-clock: pure line-idle can also mean the cart is
    // busy writing its save with the link still open, so silence alone only
    // times out as long-horizon cleanup. A cart that truly reopened its link
    // announces itself by streaming handshake words.
    constexpr uint16_t kFfffArmed        = 12;
    constexpr uint16_t kHsArmed          = 12;
    constexpr uint16_t kStreakMinFrames  = 12;
    constexpr uint32_t kFfffUnarmedMs    = 30000;
    constexpr uint32_t kHsUnarmedMs      = 1000;
    constexpr uint32_t kSilenceArmedMs   = 300;
    constexpr uint32_t kSilenceUnarmedMs = 30000;

    enum class MPhase : uint8_t { handshake = 0, established = 1 };

    volatile MPhase g_phase = MPhase::handshake;
    uint8_t  g_localSeat = 0;
    uint8_t  g_roomN = 2;              // wire size (slots on every bus)
    uint8_t  g_numTxSlots = 1;         // slots the adapter drives
    uint8_t  g_seatForSlot[4] = { 0 };
    volatile uint8_t g_cursor = 0;

    // Handshake state. Presence means "this seat's cart is handshake-ready";
    // it is relayed rather than the handshake words themselves, which are
    // constants the adapter synthesizes.
    volatile bool g_present[4] = { false };
    volatile bool g_selfPresent = false;
    volatile bool g_presenceDirty = false;
    volatile bool g_goArmed = false;          // joiner: present 0x8FFF in slot 0
    volatile uint8_t g_goCount = 0;           // joiner: the GO's player count
    volatile uint32_t g_goSeenMs = 0;         // last GO seen for the armed session
    volatile bool g_goBroadcast = false;      // seat 0: session established, send GO
    uint8_t  g_prevHsCount = 0;
    uint8_t  g_selfAbsentStreak = 0;

    // Session generation. Seat 0 mints a new value at each establishment and
    // the GO broadcast carries it; command and servo frames are tagged with
    // the sender's generation and receivers drop mismatches. This keeps the
    // teardown spam of a dying session (carts re-send READY_CLOSE_LINK every
    // window until the slowest one acks) out of the next session.
    volatile uint8_t g_gen = 0;

    // Established state.
    volatile uint8_t g_linkCount = 0;         // players in the session
    volatile uint8_t g_n = 0;                 // 0 = checksum frame, 1..8 = data
    uint8_t g_idleKeysStreak = 0;             // consecutive "held keys: none" windows
    uint16_t g_crc = 0;
    uint16_t g_crcOut = 0;
    uint16_t g_stagedWord[4] = { 0 };         // this frame's word per seat
    uint16_t g_play[4][8] = {};               // playing window per remote seat
    uint16_t g_capture[8] = {};               // local cart's window
    uint32_t g_windowCount = 0;

    // Joiner pace servo against seat 0's window counter.
    volatile int32_t g_paceAdjust = 0;
    bool     g_servoBased = false;
    uint32_t g_servoBaseOwn = 0;
    uint32_t g_servoBaseS0 = 0;

    // Close detection state. The game closes and reopens the link between
    // activities (twice entering a multi battle), so an established bus must
    // drop back to handshake when its cart leaves and let the handshake/GO
    // cycle run again.
    volatile uint32_t g_lastFrameMs = 0;
    volatile bool g_closeArmed = false;
    uint16_t g_armedTraffic = 0;
    uint16_t g_ffffStreak = 0;
    uint32_t g_ffffStartMs = 0;
    uint16_t g_hsStreak = 0;
    uint32_t g_hsStartMs = 0;

    volatile bool g_active = false;

    bool isHandshakeWord(uint16_t w)
    {
        return ((w & (uint16_t)~0x3) == LINK_SLAVE_HANDSHAKE) || (w == LINK_MASTER_HANDSHAKE);
    }

    // Window boundary: finalize the captured window, load the next command per
    // remote seat, and latch the forged checksum. Runs when staging frame 0.
    void windowBegin()
    {
        bool any = false;
        for (int i = 0; i < 8; i++) any = any || (g_capture[i] != 0);

        // The cart announcing LINKCMD_READY_CLOSE_LINK arms the fast close
        // detectors: a graceful close always follows.
        if (g_capture[0] == LINKCMD_READY_CLOSE_LINK)
        {
            g_closeArmed = true;
            g_armedTraffic = 0;
        }

        // In the trade/battle room the cart emits "held keys: none" every
        // window while idle. Peers keep a seat's last key state across idle
        // windows, so only the first of a streak (the keys-released edge)
        // carries information; the repeats stay off the network.
        bool idleKeysRepeat = false;
        if (g_capture[0] == LINKCMD_SEND_HELD_KEYS && g_capture[1] == LINK_KEY_CODE_EMPTY)
        {
            if (g_idleKeysStreak < 255) g_idleKeysStreak++;
            idleKeysRepeat = (g_idleKeysStreak > 1);
        }
        else
        {
            g_idleKeysStreak = 0;
        }

        // Word 0 of a real command is a LINKCMD id: never 0xFFFF (line idle
        // while the cart's serial is off) and never a handshake word (cart
        // already reopened). Don't relay those artifact windows while close
        // detection catches up.
        if (any && !idleKeysRepeat && g_capture[0] != 0xFFFF && !isHandshakeWord(g_capture[0]))
        {
            k_msgq_put(&g_mlOutQ, g_capture, K_NO_WAIT);
        }
        std::memset(g_capture, 0, sizeof(g_capture));

        for (uint8_t s = 0; s < g_roomN; s++)
        {
            if (s == g_localSeat) continue;
            bool got = false;
            uint8_t entry[18];
            if (s < g_linkCount)
            {
                while (k_msgq_get(kSeatQ[s], entry, K_NO_WAIT) == 0)
                {
                    if (entry[0] == g_gen) { got = true; break; }
                }
            }
            if (got)
            {
                std::memcpy(g_play[s], entry + 2, sizeof(g_play[s]));
                // Backlog catch-up: collapse identical consecutive repeats
                // (standby/keys spam is level-state, so playing it once is
                // equivalent). Block chunks are exempt: adjacent chunks can be
                // byte-identical yet positionally meaningful.
                uint16_t w0;
                std::memcpy(&w0, entry + 2, 2);
                if (w0 != LINKCMD_INIT_BLOCK && w0 != LINKCMD_CONT_BLOCK)
                {
                    uint8_t next[18];
                    while (k_msgq_num_used_get(kSeatQ[s]) > 2
                           && k_msgq_peek(kSeatQ[s], next) == 0
                           && next[0] == g_gen
                           && std::memcmp(next + 2, entry + 2, 16) == 0)
                    {
                        k_msgq_get(kSeatQ[s], next, K_NO_WAIT);
                    }
                }
            }
            else
            {
                std::memset(g_play[s], 0, sizeof(g_play[s]));
            }
        }

        g_crcOut = g_crc;
        g_crc = 0;
        g_windowCount++;
    }

    void enterEstablished(uint8_t count)
    {
        g_linkCount = count;
        g_crc = LINK_SLAVE_HANDSHAKE;  // first checksum slot, compare-skipped by the game
        g_crcOut = LINK_SLAVE_HANDSHAKE;
        std::memset(g_capture, 0, sizeof(g_capture));
        std::memset(g_play, 0, sizeof(g_play));
        g_windowCount = 0;
        g_n = 0;
        g_ffffStreak = 0;
        g_hsStreak = 0;
        g_idleKeysStreak = 0;
        g_closeArmed = false;
        g_armedTraffic = 0;
        g_servoBased = false;          // both window counters restart at 0
        g_selfPresent = true;
        g_present[g_localSeat] = true;
        g_presenceDirty = true;
        g_phase = MPhase::established;
        if (g_localSeat == 0)
        {
            g_gen++;                   // mint the new session generation
            g_goBroadcast = true;      // the GO carries it to the joiners
        }
    }

    // The cart left the link (activity transition): back to the handshake
    // phase and let presence/GO/establishment run again. g_paceAdjust is kept
    // as the starting servo estimate, and g_gen is kept so the generation
    // filter can recognize the dying session's tail traffic.
    void resetToHandshake()
    {
        g_phase = MPhase::handshake;   // first, so concurrent staging switches over
        g_n = 0;
        g_crc = 0;
        g_crcOut = 0;
        g_prevHsCount = 0;
        g_goArmed = false;
        g_goCount = 0;
        g_goBroadcast = false;
        g_linkCount = 0;
        g_selfPresent = false;
        for (int s = 0; s < 4; s++) g_present[s] = false;
        g_presenceDirty = true;
        g_selfAbsentStreak = 0;
        g_ffffStreak = 0;
        g_hsStreak = 0;
        g_idleKeysStreak = 0;
        g_closeArmed = false;
        g_armedTraffic = 0;
        std::memset(g_stagedWord, 0, sizeof(g_stagedWord));
        std::memset(g_play, 0, sizeof(g_play));
        std::memset(g_capture, 0, sizeof(g_capture));
        for (int s = 0; s < 4; s++) k_msgq_purge(kSeatQ[s]);
        k_msgq_purge(&g_mlOutQ);
        g_servoBased = false;
    }
}

struct NextTransmit MultiLinkSection::transmitCallback(void*)
{
    uint8_t cursor = g_cursor;
    if (cursor == 0 && g_phase == MPhase::established && g_n == 0) windowBegin();

    const uint8_t seat = g_seatForSlot[cursor];
    cursor++;
    g_cursor = (cursor >= g_numTxSlots) ? 0 : cursor;

    uint16_t word;
    uint32_t timing;

    if (g_phase == MPhase::handshake)
    {
        // Once the GO is armed its roster is authoritative: exactly goCount
        // seats, independent of the presence flags (which can still change as
        // late adapters reset). Every cart's own handshake logic then counts
        // the same players that seat 0's session established with.
        if (g_goArmed)
        {
            if (seat == 0)              word = LINK_MASTER_HANDSHAKE;
            else if (seat < g_goCount)  word = LINK_SLAVE_HANDSHAKE;
            else                        word = 0xFFFF;
        }
        else
        {
            word = g_present[seat] ? LINK_SLAVE_HANDSHAKE : 0xFFFF;
        }
        timing = kTimingHandshake;
    }
    else
    {
        if (g_n == 0)
        {
            word = g_crcOut;
        }
        else if (seat < g_linkCount)
        {
            word = g_play[seat][g_n - 1];
            g_crc += word;
        }
        else
        {
            word = 0xFFFF;  // seat beyond the established count: absent
        }
        int32_t t = (int32_t)kTimingCommand + g_paceAdjust;
        timing = (t < 100) ? 100u : (uint32_t)t;
    }

    g_stagedWord[seat] = word;
    return { word, timing };
}

void MultiLinkSection::receiveCallback(uint16_t rx, void*)
{
    if (g_phase != MPhase::established) return;
    const uint8_t n = g_n;
    if (n == 0) return;                 // the cart's own checksum word: discard
    g_capture[n - 1] = rx;
    g_crc += rx;
}

void MultiLinkSection::transiveDoneCallback(uint16_t rx, uint16_t, void*)
{
    g_lastFrameMs = k_uptime_get_32();

    if (g_phase == MPhase::established)
    {
        // Close detection. Serial-off reads line-idle (0xFFFF) every frame; a
        // cart that already reopened streams handshake words every frame.
        // Real window traffic can't sustain either run, because word 0 of
        // each window is a LINKCMD id and idle windows are all zero.
        if (rx == 0xFFFF)
        {
            g_hsStreak = 0;
            if (g_ffffStreak == 0) g_ffffStartMs = k_uptime_get_32();
            if (g_ffffStreak < 0xFFFF) g_ffffStreak++;
            if (g_closeArmed ? (g_ffffStreak >= kFfffArmed)
                             : (g_ffffStreak >= kStreakMinFrames
                                && (k_uptime_get_32() - g_ffffStartMs) >= kFfffUnarmedMs))
            {
                resetToHandshake();
                return;
            }
        }
        else if (isHandshakeWord(rx))
        {
            g_ffffStreak = 0;
            if (g_hsStreak == 0) g_hsStartMs = k_uptime_get_32();
            if (g_hsStreak < 0xFFFF) g_hsStreak++;
            if (g_closeArmed ? (g_hsStreak >= kHsArmed)
                             : (g_hsStreak >= kStreakMinFrames
                                && (k_uptime_get_32() - g_hsStartMs) >= kHsUnarmedMs))
            {
                resetToHandshake();
                return;
            }
        }
        else
        {
            g_ffffStreak = 0;
            g_hsStreak = 0;
            if (!g_selfPresent)
            {
                g_selfPresent = true;
                g_present[g_localSeat] = true;
                g_presenceDirty = true;
            }
            // Arm decay: sustained normal traffic after the last
            // READY_CLOSE_LINK means no close followed after all.
            if (g_closeArmed && ++g_armedTraffic >= 120)
            {
                g_closeArmed = false;
                g_armedTraffic = 0;
            }
        }

        // A cart that stopped answering must not be reported handshake-ready:
        // peers' re-handshake counts include our synthesized words, and the
        // master's re-link pulse is one-shot, so it has to fire against the
        // real roster.
        if ((g_ffffStreak >= 12 || g_hsStreak >= 12) && g_selfPresent)
        {
            g_selfPresent = false;
            g_present[g_localSeat] = false;
            g_presenceDirty = true;
        }

        uint8_t n = g_n;
        n++;
        g_n = (n >= 9) ? 0 : n;
        return;
    }

    // Handshake phase. Track whether the local cart is emitting handshake
    // words, debounced against its first (zero) transfer and stray words.
    if (isHandshakeWord(rx))
    {
        g_selfAbsentStreak = 0;
        if (!g_selfPresent) { g_selfPresent = true; g_presenceDirty = true; }
    }
    else if (g_selfPresent && ++g_selfAbsentStreak >= 3)
    {
        g_selfPresent = false;
        g_presenceDirty = true;
    }
    g_present[g_localSeat] = g_selfPresent;

    // Mirror the game's handshake on the matrix this cart just saw: our
    // staged words for the driven slots, the cart's own word for its slot.
    // Establishment must match the cart's own decision frame exactly, so the
    // window counting stays aligned from the first transfer.
    uint16_t matrix0 = 0xFFFF;
    uint8_t count = 0;
    for (uint8_t s = 0; s < g_roomN; s++)
    {
        const uint16_t w = (s == g_localSeat) ? rx : g_stagedWord[s];
        if (s == 0) matrix0 = w;
        if (isHandshakeWord(w))
        {
            count++;
        }
        else
        {
            if (w != 0xFFFF) count = 0;
            break;
        }
    }

    if (count > 1 && count == g_prevHsCount && matrix0 == LINK_MASTER_HANDSHAKE
        && isHandshakeWord(rx))   // the local cart must be in the session too
    {
        enterEstablished(count);
        return;
    }
    g_prevHsCount = count;
}

void multiLink_receiveHandler(std::span<const uint8_t> data, void*)
{
    if (!g_active || data.size() < 2) return;

    const uint8_t tag = data[0];
    const uint8_t type = tag & 0xFC;
    const uint8_t seat = tag & 0x03;

    switch (type)
    {
        case kTagPresence:
            if (seat != g_localSeat && seat < g_roomN)
            {
                g_present[seat] = (data[1] != 0);
            }
            break;

        case kTagCommand:
        {
            // Generation filter. Established: only current-session frames.
            // Handshake before the GO armed this seat: only frames from a
            // generation other than the one just left, which drops the dying
            // session's teardown spam while accepting the next session's
            // early commands. Handshake after GO-arm: the new generation is
            // already adopted, so accept exactly it; commands sent between
            // the arm and the cart's establishment must be queued.
            const bool genOk = (g_phase == MPhase::established)
                ? (data[1] == g_gen)
                : (g_goArmed ? (data[1] == g_gen) : (data[1] != g_gen));
            if (seat != g_localSeat && seat < g_roomN && data.size() >= 18 && genOk)
            {
                uint8_t entry[18];
                entry[0] = data[1];
                entry[1] = 0;
                std::memcpy(entry + 2, data.data() + 2, 16);
                if (k_msgq_put(kSeatQ[seat], entry, K_NO_WAIT) != 0)
                {
                    // Queue full: drop the oldest and keep this frame. While a
                    // bus is paused (peer saving, or between sessions) the
                    // inflow is standby-spam repeats, and the tail of the
                    // flood is what matters once the bus drains again.
                    uint8_t scratch[18];
                    k_msgq_get(kSeatQ[seat], scratch, K_NO_WAIT);
                    k_msgq_put(kSeatQ[seat], entry, K_NO_WAIT);
                }
            }
            break;
        }

        case kTagWindows:
            // Joiners servo their pace to seat 0's window rate (same session
            // generation only).
            if (seat == 0 && g_localSeat != 0 && data.size() >= 6
                && g_phase == MPhase::established && data[1] == g_gen)
            {
                uint32_t s0;
                std::memcpy(&s0, data.data() + 2, 4);
                if (!g_servoBased)
                {
                    g_servoBased = true;
                    g_servoBaseOwn = g_windowCount;
                    g_servoBaseS0 = s0;
                }
                else
                {
                    const int32_t drift = (int32_t)(g_windowCount - g_servoBaseOwn)
                                        - (int32_t)(s0 - g_servoBaseS0);
                    int32_t adjust = drift * kPaceGain;
                    if (adjust > kPaceAdjustMax) adjust = kPaceAdjustMax;
                    if (adjust < -kPaceAdjustMax) adjust = -kPaceAdjustMax;
                    g_paceAdjust = adjust;
                }
            }
            break;

        case kTagGo:
            if (g_localSeat != 0 && data.size() >= 3)
            {
                if (g_phase == MPhase::established)
                {
                    // A GO for a different generation while established means
                    // this session is dead and the live one is announcing:
                    // reset and fall through to arm for it.
                    if (data[2] == g_gen) break;   // this session's repeat
                    resetToHandshake();
                }

                if (g_goArmed && data[2] == g_gen)
                {
                    g_goSeenMs = k_uptime_get_32();   // armed session still alive
                    break;
                }
                // A stale GO replay from the session we just left carries the
                // generation we already have; arming on it would bind us to a
                // dead session. Fresh GOs always mint a new generation.
                if (data[2] == g_gen) break;
                const uint8_t goCount = (data[1] >= 2 && data[1] <= 4) ? data[1] : g_roomN;
                // A session smaller than our seat doesn't include us: stay in
                // handshake and let the cart time out per the game's own
                // rules, exactly as an unwired extra cart would.
                if (g_localSeat >= goCount) break;
                for (uint8_t s = 0; s < goCount && s < 4; s++) g_present[s] = true;
                g_goCount = goCount;
                g_gen = data[2];
                g_goArmed = true;
                g_goSeenMs = k_uptime_get_32();
            }
            break;

        default: break;
    }
}

MultiLinkSection::MultiLinkSection(uint8_t seat, uint8_t playerCount)
{
    g_roomN      = (playerCount >= 2 && playerCount <= 4) ? playerCount : 2;
    g_localSeat  = (seat < g_roomN) ? seat : 0;
    g_numTxSlots = (uint8_t)(g_roomN - 1);
    g_cursor     = 0;

    g_phase = MPhase::handshake;
    for (int s = 0; s < 4; s++) g_present[s] = false;
    g_selfPresent = false;
    g_presenceDirty = true;
    g_goArmed = false;
    g_goCount = 0;
    g_goBroadcast = false;
    g_prevHsCount = 0;
    g_selfAbsentStreak = 0;

    g_linkCount = 0;
    g_n = 0;
    g_crc = 0;
    g_crcOut = 0;
    std::memset(g_stagedWord, 0, sizeof(g_stagedWord));
    std::memset(g_play, 0, sizeof(g_play));
    std::memset(g_capture, 0, sizeof(g_capture));
    g_windowCount = 0;

    g_paceAdjust = 0;
    g_servoBased = false;
    g_lastFrameMs = k_uptime_get_32();
    g_ffffStreak = 0;
    g_hsStreak = 0;
    g_idleKeysStreak = 0;
    g_closeArmed = false;
    g_armedTraffic = 0;
    g_gen = 0;

    for (int s = 0; s < 4; s++) k_msgq_purge(kSeatQ[s]);
    k_msgq_purge(&g_mlOutQ);

    // Seats the adapter drives = every seat except our own, in slot order.
    uint8_t idx = 0;
    for (uint8_t s = 0; s < g_roomN && idx < 4; s++)
        if (s != g_localSeat) g_seatForSlot[idx++] = s;

    link_setTransmitCallback(&transmitCallback, nullptr);
    link_setReceiveCallback(&receiveCallback, nullptr);
    link_setTransiveDoneCallback(&transiveDoneCallback, nullptr);

    g_active = true;
}

MultiLinkSection::~MultiLinkSection()
{
    g_active = false;
    // The module disables the link before this runs, so no ISR races below.
    link_setTransmitCallback(nullptr, nullptr);
    link_setReceiveCallback(nullptr, nullptr);
    link_setTransiveDoneCallback(nullptr, nullptr);
}

void MultiLinkSection::process()
{
    uint32_t lastPresenceMs = 0;
    uint32_t lastWindowsMs = 0;
    uint32_t lastGoMs = 0;
    uint8_t goSendsLeft = 0;

    sendLinkStatus(LinkStatus::LinkConnected);

    while (!m_cancel)
    {
        const uint32_t now = k_uptime_get_32();

        // Arm expiry: seat 0 repeats its session's GO every 500ms. If the
        // session we armed for stops announcing, it died before our cart
        // arrived; disarm so the cart can't establish into it. The next
        // session's GO re-arms us with its fresh generation.
        if (g_phase == MPhase::handshake && g_goArmed
            && (now - g_goSeenMs) > 2000)
        {
            g_goArmed = false;
            g_goCount = 0;
        }

        // Host-side close detection: an established bus 0 sees frames every
        // few ms (the cart's timer pacing); prolonged silence means the cart
        // left the link. Joiner buses self-clock, so this never fires there.
        if (g_phase == MPhase::established
            && (now - g_lastFrameMs) > (g_closeArmed ? kSilenceArmedMs : kSilenceUnarmedMs))
        {
            resetToHandshake();
        }

        // Captured local windows -> seat-tagged command frames, stamped with
        // the session generation so they die with the session.
        uint8_t cmd[18];
        while (k_msgq_get(&g_mlOutQ, cmd + 2, K_NO_WAIT) == 0)
        {
            cmd[0] = kTagCommand | g_localSeat;
            cmd[1] = g_gen;
            Transport::sendData(std::span<const uint8_t>(cmd, sizeof(cmd)));
        }

        // Presence, on change plus a keepalive while handshaking. The value
        // means "my cart is handshake-ready": only true while we are in the
        // handshake phase and the cart is emitting handshake words, so a
        // peer's synthesized count only includes carts that are really at
        // the counter.
        if (g_presenceDirty
            || (g_phase == MPhase::handshake && (now - lastPresenceMs) > 250))
        {
            const uint8_t frame[2] = { (uint8_t)(kTagPresence | g_localSeat),
                                       (uint8_t)((g_phase == MPhase::handshake && g_selfPresent) ? 1 : 0) };
            Transport::sendData(std::span<const uint8_t>(frame, sizeof(frame)));
            g_presenceDirty = false;
            lastPresenceMs = now;
        }

        // The GO: burst right after establishing, then keep repeating it for
        // the session's lifetime so joiners whose reset lags the previous
        // teardown still find it. Repeats are idempotent (phase and
        // generation gates), and stale replays are rejected by generation.
        if (g_goBroadcast) { g_goBroadcast = false; goSendsLeft = 5; }
        if (g_localSeat == 0 && g_phase == MPhase::established
            && (goSendsLeft > 0 ? (now - lastGoMs) > 100 : (now - lastGoMs) > 500))
        {
            const uint8_t frame[3] = { kTagGo, (uint8_t)g_linkCount, g_gen };
            Transport::sendData(std::span<const uint8_t>(frame, sizeof(frame)));
            if (goSendsLeft > 0) goSendsLeft--;
            lastGoMs = now;
        }

        // Window counter for the joiners' pace servo.
        if (g_localSeat == 0
            && g_phase == MPhase::established && (now - lastWindowsMs) > 500)
        {
            uint8_t frame[6] = { (uint8_t)(kTagWindows | g_localSeat), g_gen };
            const uint32_t w = g_windowCount;
            std::memcpy(frame + 2, &w, 4);
            Transport::sendData(std::span<const uint8_t>(frame, sizeof(frame)));
            lastWindowsMs = now;
        }

        k_sleep(K_MSEC(2));
    }
}
