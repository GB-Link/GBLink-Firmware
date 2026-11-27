// Host-side tests for the Advance Wars protocol proxy (src/sections/awProtocol.hpp).
// Build & run:  make -C tests/host
//
// The frames are gpsp's netpackets (gpsp/serial_proto.c, serialaw_senddata);
// the proxy's rules are the games' own, see the header.

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cstring>

#include "../../src/sections/awProtocol.hpp"

using namespace awproto;

static int g_failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

#define CHECK_EQ(a, b)                                                       \
    do {                                                                     \
        auto va = (a); auto vb = (b);                                        \
        if (!(va == vb)) {                                                   \
            std::printf("FAIL %s:%d: %s == %s  (0x%X vs 0x%X)\n", __FILE__,  \
                        __LINE__, #a, #b, (unsigned)va, (unsigned)vb);       \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

struct CapturedFrame
{
    uint16_t cmd;
    uint8_t state;
    std::vector<uint16_t> words;
};

static void captureEmit(void* ctx, uint16_t cmd, uint8_t state,
                        const uint16_t* words, uint16_t cnt)
{
    auto* out = static_cast<std::vector<CapturedFrame>*>(ctx);
    out->push_back({ cmd, state, std::vector<uint16_t>(words, words + cnt) });
}

//-//////////////////////////////////////////////////////////////////////////-//
// MAW1 serializer / stream parser
//-//////////////////////////////////////////////////////////////////////////-//

static void testFraming()
{
    std::vector<CapturedFrame> frames;
    Maw1StreamParser parser;
    parser.setCallback(&captureEmit, &frames);

    // Three frames with zero padding in between (the transport chunker pads
    // chunk tails with zeros), pushed in every possible split position.
    const uint16_t w1[3] = { 0x4FFF, 0x0001, 0xABCD };
    const uint16_t w2[5] = { 0x8001, 0x8002, 0x8003, 0x8000, 0x0000 };

    uint8_t stream[256];
    size_t len = 0;
    len += maw1Serialize(stream + len, 0x1234, 1, nullptr, 0);
    std::memset(stream + len, 0, 7); len += 7;          // inter-frame padding
    len += maw1Serialize(stream + len, 0x4FFF, 0, w1, 3);
    len += maw1Serialize(stream + len, 0x9ABC, 2, w2, 5);
    std::memset(stream + len, 0, 3); len += 3;          // trailing padding

    for (size_t split = 0; split <= len; split++)
    {
        frames.clear();
        Maw1StreamParser p;
        p.setCallback(&captureEmit, &frames);
        p.push(stream, split);
        p.push(stream + split, len - split);

        CHECK_EQ(frames.size(), 3u);
        if (frames.size() != 3) return;

        CHECK_EQ(frames[0].cmd, 0x1234);
        CHECK_EQ(frames[0].state, 1);
        CHECK_EQ(frames[0].words.size(), 0u);

        CHECK_EQ(frames[1].cmd, 0x4FFF);
        CHECK_EQ(frames[1].state, 0);
        CHECK(frames[1].words == std::vector<uint16_t>(w1, w1 + 3));

        CHECK_EQ(frames[2].cmd, 0x9ABC);
        CHECK_EQ(frames[2].state, 2);
        CHECK(frames[2].words == std::vector<uint16_t>(w2, w2 + 5));
    }

    // Byte-at-a-time delivery
    frames.clear();
    Maw1StreamParser p;
    p.setCallback(&captureEmit, &frames);
    for (size_t i = 0; i < len; i++) p.push(stream + i, 1);
    CHECK_EQ(frames.size(), 3u);

    // Max-size frame
    uint16_t big[maxFrameWords];
    for (uint16_t i = 0; i < maxFrameWords; i++) big[i] = 0xA000 | i;
    uint8_t bigBuf[maxFrameBytes];
    size_t bigLen = maw1Serialize(bigBuf, 0x4FFF, 0, big, maxFrameWords);
    CHECK_EQ(bigLen, maxFrameBytes);
    frames.clear();
    p.push(bigBuf, bigLen);
    CHECK_EQ(frames.size(), 1u);
    CHECK(frames[0].words == std::vector<uint16_t>(big, big + maxFrameWords));
}

//-//////////////////////////////////////////////////////////////////////////-//
// A player's proxy with the frames it sent, and one transfer at a time
//-//////////////////////////////////////////////////////////////////////////-//

constexpr uint16_t START = packetStart;

struct Link
{
    AwProxy proxy;
    std::vector<CapturedFrame> sent;
    uint32_t ms = 1000;

    Link(GameVariant variant, uint8_t seat, uint8_t players)
    {
        proxy.cfg = AwConfig::forVariant(variant);
        proxy.gbaIsMaster = (seat == 0);
        proxy.peerCount = players - 1;
        proxy.emit = &captureEmit;
        proxy.emitCtx = &sent;
    }

    // One transfer: the GBA sends word and gets what was staged for the other slots.
    std::vector<uint16_t> transfer(uint16_t word)
    {
        std::vector<uint16_t> got(proxy.stagedTx, proxy.stagedTx + proxy.peerCount);
        proxy.onRound(word, ms);
        return got;
    }

    // Transfers of one word each; returns what the GBA got from one peer.
    std::vector<uint16_t> send(const std::vector<uint16_t>& words, uint8_t peer = 0)
    {
        std::vector<uint16_t> got;
        for (uint16_t word : words) got.push_back(transfer(word)[peer]);
        return got;
    }

    // The frames with words: packets and presses.
    std::vector<CapturedFrame> data() const
    {
        std::vector<CapturedFrame> out;
        for (const auto& frame : sent) if (!frame.words.empty()) out.push_back(frame);
        return out;
    }
};

// Hands the frames one player sent to another, as its peer `peer`.
static void deliver(Link& from, Link& to, uint8_t peer = 0)
{
    for (const auto& f : from.sent)
    {
        to.proxy.applyNetFrame(peer, f.cmd, f.state, f.words.data(), static_cast<uint16_t>(f.words.size()), to.ms);
    }
}

static void delivered(Link& from) { from.sent.clear(); }

// A game's packet as it sends it: the start word, the size, the checksum
// words and the data.
static std::vector<uint16_t> packet(const std::vector<uint16_t>& data, GameVariant variant = GameVariant::aw1)
{
    std::vector<uint16_t> out = { START, static_cast<uint16_t>(data.size()), 0xC1C1 };
    if (variant == GameVariant::aw2) out.push_back(0xC2C2);
    out.insert(out.end(), data.begin(), data.end());
    return out;
}

static std::vector<uint16_t> repeat(uint16_t word, size_t count) { return std::vector<uint16_t>(count, word); }

static std::vector<uint16_t> join(std::initializer_list<std::vector<uint16_t>> parts)
{
    std::vector<uint16_t> out;
    for (const auto& part : parts) out.insert(out.end(), part.begin(), part.end());
    return out;
}

static std::vector<uint16_t> slice(const std::vector<uint16_t>& words, size_t from, size_t count)
{
    return std::vector<uint16_t>(words.begin() + from, words.begin() + from + count);
}

//-//////////////////////////////////////////////////////////////////////////-//
// Packets
//-//////////////////////////////////////////////////////////////////////////-//

static void testPacket(GameVariant variant)
{
    Link a(variant, 0, 2), b(variant, 1, 2);
    std::vector<uint16_t> data;
    for (uint16_t i = 0; i < 0x42; i++) data.push_back(0x1000 + i);
    const auto sentWords = packet(data, variant);

    a.send(join({ { CMD_NONE }, sentWords, { CMD_NONE } }));
    const auto frames = a.data();
    CHECK_EQ(frames.size(), 1u);
    if (frames.size() != 1) return;
    CHECK_EQ(frames[0].cmd, START);
    CHECK_EQ(frames[0].state, statePacketXg);
    CHECK(frames[0].words == slice(sentWords, 1, sentWords.size() - 1));

    // Replayed a transfer late, then one idle word
    deliver(a, b);
    const auto got = b.send(repeat(CMD_NONE, sentWords.size() + 3));
    CHECK(got == join({ { CMD_NONE }, sentWords, { CMD_NONE, CMD_NONE } }));
}

static void testLoneStartWord()
{
    Link a(GameVariant::aw1, 0, 2), b(GameVariant::aw1, 1, 2);
    const auto sentWords = packet({ 0x0036, 0x0001 });
    // The idle word after a start word is no size; nor is a second start word
    a.send(join({ { START, CMD_NONE, CMD_NONE, START }, sentWords, { CMD_NONE } }));
    CHECK_EQ(a.data().size(), 1u);
    deliver(a, b);
    const auto got = b.send(repeat(CMD_NONE, 8));
    CHECK(slice(got, 1, sentWords.size()) == sentWords);
}

static void testPacketGivenUpForBarrier()
{
    Link a(GameVariant::aw1, 0, 2), b(GameVariant::aw1, 1, 2);
    const uint16_t sync = a.proxy.cfg.syncWord;
    a.send({ START, 0x000D, 0xC1C1, 0x003F, 0x0000, sync, sync, sync });
    CHECK_EQ(a.data().size(), 0u);
    CHECK_EQ(a.proxy.state, stateSync);
    deliver(a, b);
    CHECK_EQ(b.send({ CMD_NONE, CMD_NONE })[1], sync);
}

//-//////////////////////////////////////////////////////////////////////////-//
// Sync barrier
//-//////////////////////////////////////////////////////////////////////////-//

static void testBarrier()
{
    Link a(GameVariant::aw1, 0, 3), b(GameVariant::aw1, 1, 3), c(GameVariant::aw1, 2, 3);
    const uint16_t sync = a.proxy.cfg.syncWord;

    // The leader keeps sending packets; b's game already waits at the barrier
    a.send(packet({ 0x003A, 0x0000 }));
    deliver(a, b, 0); deliver(a, c, 0); delivered(a);
    const auto atB = b.send({ sync, CMD_NOP, sync, CMD_NOP });
    CHECK(atB == std::vector<uint16_t>({ CMD_NONE, START, 2, 0xC1C1 }));   // still given the leader's packet
    deliver(b, a, 0); deliver(b, c, 1); delivered(b);
    CHECK(a.send({ CMD_NONE, CMD_NONE }, 0) == std::vector<uint16_t>({ CMD_NONE, sync }));   // the leader is told at once
    CHECK_EQ(a.transfer(CMD_NONE)[1], CMD_NONE);                           // c isn't there yet

    c.send({ sync, CMD_NOP, sync, CMD_NOP, sync, CMD_NOP, sync, CMD_NOP, sync });
    deliver(c, a, 1); deliver(c, b, 1); delivered(c);
    a.transfer(CMD_NONE);
    CHECK_EQ(a.proxy.stagedTx[0], sync);
    CHECK_EQ(a.proxy.stagedTx[1], sync);

    // The leader joins: every game sees the sync word in every slot
    a.send({ sync, sync, sync });
    deliver(a, b, 0); deliver(a, c, 0); delivered(a);
    b.send({ sync, CMD_NOP, sync, CMD_NOP, sync, CMD_NOP, sync });
    deliver(b, a, 0); deliver(b, c, 1); delivered(b);
    c.transfer(sync);
    CHECK_EQ(b.proxy.state, stateSync);
    CHECK_EQ(b.proxy.stagedTx[0], sync);
    CHECK_EQ(b.proxy.stagedTx[1], sync);
    CHECK_EQ(c.proxy.stagedTx[0], sync);
    CHECK_EQ(c.proxy.stagedTx[1], sync);
    CHECK_EQ(b.proxy.stagedTiming, timingSync);
}

//-//////////////////////////////////////////////////////////////////////////-//
// Key exchange
//-//////////////////////////////////////////////////////////////////////////-//

// Takes both games through the barrier into the key exchange, at screen 0x8800.
static void exchangeKeys(Link& a, Link& b)
{
    const uint16_t sync = a.proxy.cfg.syncWord;
    a.send({ sync, sync });
    deliver(a, b); delivered(a);
    b.send({ sync, sync });
    deliver(b, a); delivered(b);
    a.send({ sync, 0x8800, 0x8800 });
    b.send({ sync, 0x8800, 0x8800 });
    deliver(a, b); delivered(a);
    deliver(b, a); delivered(b);
}

static void testKeys()
{
    Link a(GameVariant::aw1, 0, 2), b(GameVariant::aw1, 1, 2);
    exchangeKeys(a, b);
    CHECK_EQ(a.proxy.state, stateIntersync);
    CHECK_EQ(b.proxy.state, stateIntersync);

    // Two presses of A that arrive together stay two presses
    a.send({ 0x8801, 0x8801, 0x8801, 0x8800, 0x8801, 0x8801, 0x8800 });
    deliver(a, b); delivered(a);
    const auto got = b.send(repeat(0x8800, 9));
    CHECK(slice(got, 1, 8) == std::vector<uint16_t>({ 0x8801, 0x8801, 0x8801, 0x8800, 0x8801, 0x8801, 0x8800, 0x8800 }));

    // A press of one transfer crosses
    a.send({ 0x8802, 0x8800 });
    deliver(a, b); delivered(a);
    const auto tap = b.send(repeat(0x8800, 4));
    CHECK_EQ(tap[1], 0x8802);
}

static void testLongPress()
{
    Link a(GameVariant::aw1, 0, 2), b(GameVariant::aw1, 1, 2);
    exchangeKeys(a, b);

    const size_t held = 400;
    a.send(join({ repeat(0x8880, held), { 0x8800 } }));
    const auto parts = a.data();
    CHECK(parts.size() > 1);
    size_t total = 0;
    for (const auto& part : parts)
    {
        CHECK(part.words.size() <= maxFrameWords);
        total += part.words.size();
    }
    CHECK_EQ(total, held);
    CHECK_EQ(parts.front().cmd, keysContinue);
    CHECK(parts.back().cmd != keysContinue);

    // Replayed as long as it was, without a break
    deliver(a, b); delivered(a);
    const auto got = b.send(repeat(0x8800, held + 60));
    size_t first = 0;
    while (first < got.size() && got[first] != 0x8880) first++;
    CHECK(first > 0 && first + held < got.size());
    if (first + held >= got.size()) return;
    CHECK(slice(got, first, held) == repeat(0x8880, held));
    CHECK_EQ(got[first + held], 0x8800);
}

static void testLatePartStaysHeld()
{
    Link a(GameVariant::aw1, 0, 2), b(GameVariant::aw1, 1, 2);
    exchangeKeys(a, b);

    a.send(repeat(0x8880, 200));
    deliver(a, b); delivered(a);
    const auto early = b.send(repeat(0x8800, 180));
    CHECK_EQ(early.back(), 0x8880);
    a.send(join({ repeat(0x8880, 50), { 0x8800 } }));
    deliver(a, b); delivered(a);
    const auto rest = b.send(repeat(0x8800, 200));
    size_t run = 0, longest = 0;
    for (uint16_t word : join({ early, rest }))
    {
        run = (word == 0x8880) ? run + 1 : 0;
        if (run > longest) longest = run;
    }
    CHECK(longest >= 250);
    CHECK_EQ(rest.back(), 0x8800);
}

static void testSkippedFrame()
{
    Link a(GameVariant::aw1, 0, 2), b(GameVariant::aw1, 1, 2);
    exchangeKeys(a, b);

    // B held over a transfer for which the game has no word ready
    a.send({ 0x8802, 0x8802, CMD_NOP, 0x8802, 0x8800 });
    deliver(a, b); delivered(a);
    size_t pressed = 0;
    for (uint16_t word : b.send(repeat(0x8800, 10))) if (word == 0x8802) pressed++;
    CHECK_EQ(pressed, 4u);

    // The other player's idle word is always a key word of the game's own screen number
    const auto echo = a.send({ CMD_NOP, 0x8800, CMD_NOP, CMD_NOP, 0x8C00, 0x8C00 });
    CHECK(slice(echo, 1, 5) == std::vector<uint16_t>({ 0x8800, 0x8800, 0x8800, 0x8800, 0x8C00 }));
}

static void testKeysWaitAtBarrier()
{
    Link a(GameVariant::aw1, 0, 2), b(GameVariant::aw1, 1, 2);
    const uint16_t sync = a.proxy.cfg.syncWord;

    // b's game waits at the barrier and is still given the leader's packet
    // when the leader, through it already, presses A
    std::vector<uint16_t> data;
    for (uint16_t i = 0; i < 0x20; i++) data.push_back(i);
    a.send(packet(data));
    deliver(a, b); delivered(a);
    b.send({ sync, CMD_NOP, sync });
    deliver(b, a); delivered(b);
    a.send({ sync, sync, sync, 0x8800, 0x8801, 0x8801, 0x8800 });
    deliver(a, b); delivered(a);

    std::vector<uint16_t> words;
    for (int i = 0; i < 60; i++) words.push_back(i % 2 ? CMD_NOP : sync);
    const auto got = b.send(join({ words, repeat(0x8800, 5) }));
    bool sawSync = false;
    size_t pressed = 0;
    for (uint16_t word : got) if (word == sync) sawSync = true;
    for (size_t i = got.size() - 4; i < got.size(); i++) if (got[i] == 0x8801) pressed++;
    CHECK(sawSync);
    CHECK_EQ(pressed, 2u);
}

static void testFirstWordAfterKeys(GameVariant variant)
{
    Link a(variant, 0, 2), b(variant, 1, 2);
    const uint16_t sync = a.proxy.cfg.syncWord;
    const std::vector<uint16_t> through = { sync, sync, sync, 0x8800, 0x8800 };
    a.send({ sync, sync });
    deliver(a, b); delivered(a);
    b.send(through);
    deliver(b, a); delivered(b);
    a.send(through);
    a.sent.clear();

    // Straight from keys into a packet, and from keys to the next barrier
    const auto sentWords = packet({ 0x003A, 0x0000 }, variant);
    a.send(join({ sentWords, { 0x8800 } }));
    const auto frames = a.data();
    CHECK_EQ(frames.size(), 1u);
    if (frames.size() == 1) CHECK(frames[0].words == slice(sentWords, 1, sentWords.size() - 1));
    b.send({ 0x8800, sync, CMD_NOP, sync });
    CHECK_EQ(b.sent.back().state, stateSync);
}

//-//////////////////////////////////////////////////////////////////////////-//
// Four players
//-//////////////////////////////////////////////////////////////////////////-//

static void testFourPlayers()
{
    Link a(GameVariant::aw1, 0, 4), b(GameVariant::aw1, 1, 4), c(GameVariant::aw1, 2, 4);
    std::vector<uint16_t> dataB, dataC;
    for (uint16_t i = 0; i < 0x30; i++) { dataB.push_back(0xB000 + i); dataC.push_back(0xC000 + i); }
    const auto fromB = packet(dataB), fromC = packet(dataC);
    b.send(fromB);
    c.send(fromC);
    deliver(b, a, 0);
    deliver(c, a, 1);

    std::vector<uint16_t> slotB, slotC;
    for (size_t i = 0; i < fromB.size() + 2; i++)
    {
        const auto got = a.transfer(CMD_NONE);
        slotB.push_back(got[0]);
        slotC.push_back(got[1]);
        CHECK_EQ(got[2], CMD_NONE);   // the fourth player is idle
    }
    CHECK(slice(slotB, 1, fromB.size()) == fromB);
    CHECK(slice(slotC, 1, fromC.size()) == fromC);
}

static void testAnnouncements()
{
    Link a(GameVariant::aw1, 0, 2);
    a.send(repeat(CMD_NONE, 50));
    CHECK_EQ(a.sent.size(), 1u);          // the idle state once
    a.ms += keepaliveMs;
    a.transfer(CMD_NONE);
    CHECK_EQ(a.sent.size(), 2u);          // and again to be heard
}

int main()
{
    testFraming();
    testPacket(GameVariant::aw1);
    testPacket(GameVariant::aw2);
    testLoneStartWord();
    testPacketGivenUpForBarrier();
    testBarrier();
    testKeys();
    testLongPress();
    testLatePartStaysHeld();
    testSkippedFrame();
    testKeysWaitAtBarrier();
    testFirstWordAfterKeys(GameVariant::aw1);
    testFirstWordAfterKeys(GameVariant::aw2);
    testFourPlayers();
    testAnnouncements();

    if (g_failures == 0)
    {
        std::printf("All awProtocol tests passed.\n");
        return 0;
    }
    std::printf("%d failure(s).\n", g_failures);
    return 1;
}
