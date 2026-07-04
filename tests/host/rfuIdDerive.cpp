// Oracle: derive the SIO32 ID-exchange sequence the wireless adapter must
// play to pass FRLG's AgbRFU_checkID (pokefirered src/librfu_sio32id.c).
//
// checkID runs the GBA as SIO master, walking the "NINTENDO" connection
// words and expecting the adapter (slave) to answer with a complement dance
// that converges to RFU_ID = 0x8001. gpsp answers it SAME-CYCLE with
// (sent<<16)|~prev (rfu.c RESET/HANDSHAKE) — an emulator luxury. Real
// hardware preloads its TX word one transfer ahead, so a same-cycle echo is
// impossible; the real adapter (like gpsp's canned gbp_seq for the GB
// Player) plays a FIXED, input-independent sequence instead.
//
// This program models Sio32IDIntr faithfully, drives it with gpsp's
// same-cycle formula to capture the exact per-transfer values, then proves
// that replaying those exact values as a canned preload sequence also
// converges. The captured table is what the firmware will play back.
//
// Build & run:  g++ -std=c++20 -O1 rfuIdDerive.cpp -o /tmp/rfuIdDerive && /tmp/rfuIdDerive

#include <cstdio>
#include <cstdint>
#include <vector>

static const uint16_t kNintendo[4] = { 0x494E, 0x544E, 0x4E45, 0x4F44 };
static constexpr uint32_t RFU_ID = 0x00008001;

// Faithful model of the GBA side of AgbRFU_checkID, MASTER clock path
// (librfu_sio32id.c Sio32IDIntr + Sio32IDMain). One step() consumes the
// word received from the adapter and returns the next word the GBA will
// transmit (what it writes to REG_SIODATA32 for the following transfer).
struct GbaCheckId
{
    uint16_t send_id = 0, recv_id = 0, count = 0;
    uint32_t lastId = 0;

    // Returns the next G (word the GBA transmits) after processing received A.
    uint32_t step(uint32_t received)
    {
        const uint16_t lo = received & 0xFFFF;          // master: low16
        const uint16_t hi = (received >> 16) & 0xFFFF;  // master: high16

        if (lastId == 0)
        {
            if (lo == recv_id)
            {
                if (count < 4)
                {
                    if (recv_id == (uint16_t)~send_id)
                        if (hi == (uint16_t)~recv_id)
                            ++count;
                }
                else
                    lastId = hi;
            }
            else
                count = 0;
        }

        send_id = (count < 4) ? kNintendo[count] : (uint16_t)RFU_ID;
        recv_id = (uint16_t)~hi;
        return ((uint32_t)recv_id << 16) | send_id;  // G_{next}
    }
};

// gpsp's same-cycle adapter (rfu.c RESET/HANDSHAKE).
struct GpspAdapter
{
    enum { RESET, HANDSHAKE, WAITCMD } state = RESET;
    uint32_t prev = 0;

    uint32_t transfer(uint32_t sent)
    {
        uint32_t ret = 0x80000000;
        if (state == RESET)
        {
            ret = 0;
            if ((sent & 0xFFFF) == 0x494E) state = HANDSHAKE;
        }
        else if (state == HANDSHAKE)
        {
            if (sent == 0xB0BB8001) state = WAITCMD;
            ret = (sent << 16) | ((~prev) & 0xFFFF);
        }
        prev = sent;
        return ret;
    }
};

int main()
{
    // Phase 1: capture gpsp's per-transfer responses in the closed loop.
    GbaCheckId gba;
    GpspAdapter ad;
    std::vector<uint32_t> table;  // A_N, the value the GBA reads on transfer N

    uint32_t G = 0;  // GBA's first transmitted word (SIODATA32 after reset)
    int convergeTransfer = -1;
    for (int n = 1; n <= 64; n++)
    {
        const uint32_t A = ad.transfer(G);     // same-cycle response to G_n
        table.push_back(A);
        const uint32_t prevLastId = gba.lastId;
        G = gba.step(A);                        // GBA processes A, makes G_{n+1}
        if (prevLastId == 0 && gba.lastId != 0) { convergeTransfer = n; break; }
    }

    std::printf("gpsp closed loop: lastId=0x%04X at transfer %d (need 0x%04X)\n",
                gba.lastId, convergeTransfer, RFU_ID);
    std::printf("Captured %zu-entry canned table:\n", table.size());
    for (size_t i = 0; i < table.size(); i++)
        std::printf("    0x%08X,%s", table[i], (i % 4 == 3) ? "\n" : "");
    std::printf("\n");

    // Phase 2: replay the captured table as a canned preload sequence
    // (input-independent) against a fresh GBA. Must converge identically.
    GbaCheckId gba2;
    uint32_t convergeId = 0;
    int convergeAt = -1;
    for (size_t n = 0; n < table.size(); n++)
    {
        const uint32_t prevLastId = gba2.lastId;
        gba2.step(table[n]);
        if (prevLastId == 0 && gba2.lastId != 0) { convergeId = gba2.lastId; convergeAt = (int)n + 1; break; }
    }
    std::printf("canned replay: lastId=0x%04X at transfer %d\n", convergeId, convergeAt);

    const bool ok = (gba.lastId == RFU_ID) && (convergeId == RFU_ID);
    std::printf("%s\n", ok ? "PASS: canned sequence converges to RFU_ID" : "FAIL");
    return ok ? 0 : 1;
}
