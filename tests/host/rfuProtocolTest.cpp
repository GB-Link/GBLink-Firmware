// Host-side tests for the RFU protocol core (src/sections/rfuProtocol.hpp).
// Build & run:  make -C tests/host
//
// gpsp/rfu.c is the protocol oracle: expected word sequences below are its
// rfu_transfer() return stream shifted by the one transfer real hardware
// (and this port's consume/produce split) imposes.

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <deque>
#include <cstring>

#include "../../src/sections/rfuProtocol.hpp"

using namespace rfuproto;

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
            std::printf("FAIL %s:%d: %s == %s  (0x%lX vs 0x%lX)\n", __FILE__,\
                        __LINE__, #a, #b, (unsigned long)va,                 \
                        (unsigned long)vb);                                  \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

static uint16_t fixedRand(void*)
{
    static uint16_t seq = 0x1233;
    return ++seq;  // non-zero, deterministic
}

struct CapturedFrame
{
    std::vector<uint8_t> bytes;
};

// One GBA-master transfer: the GBA receives what was staged BEFORE the
// transfer, while sending `send`.
static uint32_t xfer(RfuCore& c, uint32_t send, uint32_t t)
{
    const uint32_t received = c.stagedTx;
    c.onSlaveTransfer(send, t);
    return received;
}

static uint32_t cmdHdr(uint8_t cmd, uint8_t len)
{
    return 0x99660000u | (static_cast<uint32_t>(len) << 8) | cmd;
}

// Faithful model of the GBA side of FRLG's AgbRFU_checkID, MASTER clock path
// (pokefirered src/librfu_sio32id.c Sio32IDIntr). step() consumes the word
// received from the adapter and returns the next word the GBA transmits.
struct GbaCheckId
{
    static constexpr uint16_t kNintendo[4] = { 0x494E, 0x544E, 0x4E45, 0x4F44 };
    uint16_t send_id = 0, recv_id = 0, count = 0;
    uint32_t lastId = 0;

    uint32_t step(uint32_t received)
    {
        const uint16_t lo = received & 0xFFFF;
        const uint16_t hi = (received >> 16) & 0xFFFF;
        if (lastId == 0)
        {
            if (lo == recv_id)
            {
                if (count < 4)
                {
                    if (recv_id == (uint16_t)~send_id && hi == (uint16_t)~recv_id)
                        ++count;
                }
                else
                    lastId = hi;
            }
            else
                count = 0;
        }
        send_id = (count < 4) ? kNintendo[count] : (uint16_t)0x8001;
        recv_id = (uint16_t)~hi;
        return ((uint32_t)recv_id << 16) | send_id;
    }
};

// Drive RfuCore through the real ID exchange so it sits in comWaitCmd, and
// assert the GBA's checkID converges to RFU_ID (0x8001). gbaFirstWord lets
// tests vary the GBA's initial SIODATA32 to prove prefix self-alignment.
static void login(RfuCore& c, uint32_t t = 0, uint32_t gbaFirstWord = 0)
{
    GbaCheckId gba;
    uint32_t G = gbaFirstWord;
    for (int n = 0; n < 40 && gba.lastId == 0; n++)
        G = gba.step(xfer(c, G, t));

    CHECK_EQ(gba.lastId, 0x8001u);    // adapter detected as the wireless adapter
    CHECK_EQ(c.comstate, comWaitCmd);
}

// Issue a command and collect the ACK plus response words by polling with
// BUSY_WORD (the GBA's dummy poll value).
struct CmdResult
{
    uint32_t ack;
    std::vector<uint32_t> words;
};

static CmdResult runCommand(RfuCore& c, uint8_t cmd, const std::vector<uint32_t>& payload,
                            uint32_t t)
{
    xfer(c, cmdHdr(cmd, static_cast<uint8_t>(payload.size())), t);
    for (uint32_t w : payload)
    {
        const uint32_t during = xfer(c, w, t);
        CHECK_EQ(during, BUSY_WORD);  // payload words are answered with busy
    }

    CmdResult r{};
    r.ack = xfer(c, BUSY_WORD, t);

    // The response length rides in the ACK (also true for the error frame
    // 0x996601EE, which is followed by exactly one error-code word). The
    // core's state may already have advanced past the delivery state — the
    // staged words are what matters.
    const uint8_t respLen =
        ((r.ack >> 16) == 0x9966) ? static_cast<uint8_t>(r.ack >> 8) : 0;
    for (uint8_t i = 0; i < respLen; i++)
        r.words.push_back(xfer(c, BUSY_WORD, t));
    return r;
}

//-//////////////////////////////////////////////////////////////////////////-//
// RFU1 framing
//-//////////////////////////////////////////////////////////////////////////-//

struct ParsedFrame
{
    uint32_t ptype;
    uint32_t hdata;
    std::vector<uint8_t> payload;
};

static void captureParsed(void* ctx, uint32_t ptype, uint32_t hdata,
                          const uint8_t* payload, uint16_t plen)
{
    auto* out = static_cast<std::vector<ParsedFrame>*>(ctx);
    out->push_back({ ptype, hdata, std::vector<uint8_t>(payload, payload + plen) });
}

static void testFraming()
{
    uint8_t stream[256];
    size_t len = 0;

    len += rfu1SerializeCmd(stream + len, NET_CONNECT_ACK, 0x00021234);
    std::memset(stream + len, 0, 12); len += 12;  // transport chunk padding

    const uint32_t bdata[6] = { 1, 2, 3, 0xDEADBEEF, 5, 6 };
    len += rfu1SerializeBcast(stream + len, 0xABCD, bdata);

    const uint32_t words[4] = { 0x44332211, 0x88776655, 0xCCBBAA99, 0x00FFEEDD };
    len += rfu1SerializeData(stream + len, NET_HOST_SEND, 14, words, 14);
    std::memset(stream + len, 0, 5); len += 5;

    for (size_t split = 0; split <= len; split++)
    {
        std::vector<ParsedFrame> frames;
        Rfu1StreamParser p;
        p.setCallback(&captureParsed, &frames);
        p.push(stream, split);
        p.push(stream + split, len - split);

        CHECK_EQ(frames.size(), 3u);
        if (frames.size() != 3) return;

        CHECK_EQ(frames[0].ptype, NET_CONNECT_ACK);
        CHECK_EQ(frames[0].hdata, 0x00021234u);
        CHECK_EQ(frames[0].payload.size(), 4u);  // pad word

        CHECK_EQ(frames[1].ptype, NET_BROADCAST);
        CHECK_EQ(frames[1].hdata, 0xABCDu);
        CHECK_EQ(unpack32be(&frames[1].payload[12]), 0xDEADBEEFu);

        CHECK_EQ(frames[2].ptype, NET_HOST_SEND);
        CHECK_EQ(frames[2].hdata, 14u);
        // little-endian byte flattening: first payload byte = low byte of word 0
        CHECK_EQ(frames[2].payload[0], 0x11u);
        CHECK_EQ(frames[2].payload[12], 0xDDu);  // word 3 low byte
        CHECK_EQ(frames[2].payload[13], 0xEEu);
    }
}

//-//////////////////////////////////////////////////////////////////////////-//
// Login + zero/with-payload commands + error path
//-//////////////////////////////////////////////////////////////////////////-//

static void testLoginAndCommands()
{
    RfuCore c;
    c.rand16 = &fixedRand;
    c.reset();
    login(c);

    // First command transfer still receives the sticky login echo
    const uint32_t sticky = c.stagedTx;
    CHECK_EQ(xfer(c, cmdHdr(CMD_SYSVER, 0), 10), sticky);
    CHECK_EQ(xfer(c, BUSY_WORD, 10), cmdHdr(CMD_SYSVER, 1) | 0x80);  // ACK 0x99660192
    CHECK_EQ(xfer(c, BUSY_WORD, 10), 0x00830117u);                   // version word
    CHECK_EQ(xfer(c, BUSY_WORD, 10), BUSY_WORD);                     // idle again
    CHECK_EQ(c.comstate, comWaitCmd);

    // INIT1: zero payload, zero response
    auto r = runCommand(c, CMD_INIT1, {}, 20);
    CHECK_EQ(r.ack, cmdHdr(CMD_INIT1, 0) | 0x80);
    CHECK_EQ(c.comstate, comWaitCmd);

    // BCST_DATA: six payload words accepted
    auto b = runCommand(c, CMD_BCST_DATA, { 1, 2, 3, 4, 5, 6 }, 30);
    CHECK_EQ(b.ack, cmdHdr(CMD_BCST_DATA, 0) | 0x80);

    // HOST_ACCEPT while idle is an error: 0x996601EE then the error code
    auto e = runCommand(c, CMD_HOST_ACCEPT, {}, 40);
    CHECK_EQ(e.ack, 0x996601EEu);
    CHECK_EQ(e.words.size(), 1u);
    CHECK_EQ(e.words[0], 1u);  // -(-1)
    CHECK_EQ(c.comstate, comWaitCmd);

    // SYSCFG reconfigures the wait timeout used later
    auto s = runCommand(c, CMD_SYSCFG, { (10u << 8) | 60u }, 50);  // 60 frames, rtx 10
    CHECK_EQ(s.ack, cmdHdr(CMD_SYSCFG, 0) | 0x80);
}

//-//////////////////////////////////////////////////////////////////////////-//
// WAIT → WAITEVENT response priorities (≙ gpsp rfu_update, rfu.c:891-928):
// idle → immediate disconnect notice; connected client with no data → the
// 533 ms timeout; host with no client data → the ~11 ms rtx "no response".
//-//////////////////////////////////////////////////////////////////////////-//

// The section clocks eventWord(0..count-1) into the GBA synchronously and
// then finishes; tests read the staged frame the same way.
static std::vector<uint32_t> takeDelivery(RfuCore& c)
{
    std::vector<uint32_t> words;
    for (uint8_t i = 0; i < c.eventWordCount(); i++)
        words.push_back(c.eventWord(i));
    c.finishDelivery();
    return words;
}

static void testWaitResponses()
{
    std::vector<CapturedFrame> sink;
    const auto sinkEmit = [](void* ctx, const uint8_t* f, size_t l) {
        static_cast<std::vector<CapturedFrame>*>(ctx)->push_back(
            { std::vector<uint8_t>(f, f + l) });
    };

    // A: WAIT while idle resolves immediately with a disconnect notice
    {
        RfuCore c;
        c.rand16 = &fixedRand;
        c.reset();
        login(c);
        runCommand(c, CMD_SYSVER, {}, 0);

        runCommand(c, CMD_WAIT, {}, 1000);
        CHECK_EQ(c.comstate, comWaitEvent);
        CHECK(c.pollWaitEvent(1001));
        const auto w = takeDelivery(c);
        CHECK_EQ(w.size(), 3u);
        CHECK_EQ(w[0], 0x99660129u);  // RESP_DISC, len 1
        CHECK_EQ(w[1], 0xFu);
        CHECK_EQ(w[2], BUSY_WORD);
        CHECK_EQ(c.comstate, comWaitCmd);
        CHECK_EQ(c.stagedTx, BUSY_WORD);
    }

    // B: connected client with no incoming data hits the wait timeout
    {
        RfuCore c;
        c.rand16 = &fixedRand;
        c.emit = sinkEmit;
        c.emitCtx = &sink;
        c.reset();
        login(c);
        runCommand(c, CMD_SYSVER, {}, 0);

        const uint8_t zeros[24] = {};
        c.applyNetPacket(NET_BROADCAST, 0xAAAA, zeros, 24, 0);
        runCommand(c, CMD_CONNECT, { 0xAAAA }, 0);
        c.applyNetPacket(NET_CONNECT_ACK, 0x00001234, zeros, 4, 0);
        CHECK_EQ(c.state, stClient);

        uint32_t t = 1000;
        runCommand(c, CMD_WAIT, {}, t);
        // The first poll arms the deadlines (produce() has no clock); a
        // connected client with no host payload rides the wait to the 533ms
        // TIMEO exactly like gpsp — data events come from real arrivals
        // only, no fake empty-data keepalive (it would unblock the pump
        // off the parent's pace and break the WAIT rendezvous).
        CHECK(!c.pollWaitEvent(t));
        CHECK(!c.pollWaitEvent(t + 500));
        CHECK(c.pollWaitEvent(t + 534));
        const auto w = takeDelivery(c);
        CHECK_EQ(w.size(), 2u);
        CHECK_EQ(w[0], 0x99660000u | CMD_RESP_TIMEO);
        CHECK_EQ(w[1], BUSY_WORD);
        CHECK_EQ(c.comstate, comWaitCmd);
    }

    // C: host without client responses hits the rtx window first (~11 ms)
    {
        RfuCore c;
        c.rand16 = &fixedRand;
        c.emit = sinkEmit;
        c.emitCtx = &sink;
        c.reset();
        login(c);
        runCommand(c, CMD_SYSVER, {}, 0);

        runCommand(c, CMD_HOST_START, {}, 0);
        const uint8_t zeros[4] = {};
        c.applyNetPacket(NET_CONNECT_REQ, 0, zeros, 4, 0);  // a client joins
        CHECK_EQ(runCommand(c, CMD_HOST_ACCEPT, {}, 0).words.size(), 1u);

        uint32_t t = 2000;
        runCommand(c, CMD_SEND_DATAW, { 4, 0x11223344 }, t);
        CHECK_EQ(c.comstate, comWaitEvent);
        CHECK(!c.pollWaitEvent(t + 5));
        CHECK(c.pollWaitEvent(t + 20));
        const auto w = takeDelivery(c);
        CHECK_EQ(w.size(), 3u);
        CHECK_EQ(w[0], 0x99660128u);  // RESP_DATA | len 1: no response
        CHECK_EQ(w[1], 0x00000F0Fu);
        CHECK_EQ(w[2], BUSY_WORD);
        CHECK_EQ(c.comstate, comWaitCmd);
    }
}

//-//////////////////////////////////////////////////////////////////////////-//
// Two cores cross-wired: broadcast → connect → data both ways → disconnect.
// Frames pass through the real parser, optionally delayed (latency queue).
//-//////////////////////////////////////////////////////////////////////////-//

// Frames are always queued and delivered on flush — like the real transport,
// where a frame emitted inside the PIO ISR can never re-enter the emitter's
// own call stack (the zero-latency alternative would deliver a CONNECT_ACK
// before the connecting core even sets stConnecting).
struct Pipe
{
    Rfu1StreamParser parser;
    RfuCore* dest = nullptr;
    std::deque<CapturedFrame> queue;
    uint32_t* clock = nullptr;

    static void onFrame(void* ctx, uint32_t ptype, uint32_t hdata,
                        const uint8_t* payload, uint16_t plen)
    {
        Pipe* self = static_cast<Pipe*>(ctx);
        self->dest->applyNetPacket(ptype, hdata, payload, plen, *self->clock);
    }

    static void onEmit(void* ctx, const uint8_t* frame, size_t len)
    {
        Pipe* self = static_cast<Pipe*>(ctx);
        self->queue.push_back({ std::vector<uint8_t>(frame, frame + len) });
    }

    void flush()
    {
        while (!queue.empty())
        {
            parser.push(queue.front().bytes.data(), queue.front().bytes.size());
            queue.pop_front();
        }
    }
};

static void testEndToEnd()
{
    uint32_t t = 0;

    RfuCore host, client;
    Pipe hostToClient, clientToHost;

    hostToClient.dest = &client;
    hostToClient.clock = &t;
    hostToClient.parser.setCallback(&Pipe::onFrame, &hostToClient);

    clientToHost.dest = &host;
    clientToHost.clock = &t;
    clientToHost.parser.setCallback(&Pipe::onFrame, &clientToHost);

    host.rand16 = &fixedRand;
    host.emit = &Pipe::onEmit;
    host.emitCtx = &hostToClient;
    host.reset();

    client.rand16 = &fixedRand;
    client.emit = &Pipe::onEmit;
    client.emitCtx = &clientToHost;
    client.reset();

    // Flushing one direction can enqueue replies in the other (CONNECT_REQ →
    // CONNECT_ACK, HOST_SEND → CLIENT_ACK); settle both.
    auto deliver = [&]() {
        for (int i = 0; i < 4 && (!hostToClient.queue.empty() ||
                                  !clientToHost.queue.empty()); i++)
        {
            clientToHost.flush();
            hostToClient.flush();
        }
    };

    login(host, t);
    login(client, t);
    runCommand(host, CMD_SYSVER, {}, t);
    runCommand(client, CMD_SYSVER, {}, t);

    // Host side: set broadcast data and start hosting
    const std::vector<uint32_t> bdata = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
    runCommand(host, CMD_BCST_DATA, bdata, t);
    auto hs = runCommand(host, CMD_HOST_START, {}, t);
    CHECK_EQ(hs.ack, cmdHdr(CMD_HOST_START, 0) | 0x80);
    CHECK_EQ(host.state, stHost);

    // Broadcast goes out on the next tick (HOST_START forces it immediately)
    host.tick(t);
    t += 200; deliver();

    // Client scans and sees the host
    runCommand(client, CMD_BCRD_START, {}, t);
    auto fetch = runCommand(client, CMD_BCRD_FETCH, {}, t);
    CHECK_EQ(fetch.words.size(), 7u);
    const uint16_t hostDevid = static_cast<uint16_t>(fetch.words[0]);
    CHECK(hostDevid != 0);
    CHECK_EQ(fetch.words[4], 0x44u);  // broadcast payload words follow the devid

    // Connect
    auto conn = runCommand(client, CMD_CONNECT, { hostDevid }, t);
    CHECK_EQ(conn.ack, cmdHdr(CMD_CONNECT, 0) | 0x80);
    CHECK_EQ(client.state, stConnecting);

    // Frames are still in flight: the connection is reported in progress
    auto inProg = runCommand(client, CMD_ISCONNECTED, {}, t);
    CHECK_EQ(inProg.words[0], CONN_INPROGRESS);

    t += 150; deliver();  // CONNECT_REQ reaches host, ACK returns
    CHECK_EQ(client.state, stClient);

    auto isconn = runCommand(client, CMD_ISCONNECTED, {}, t);
    const uint16_t clientDevid = isconn.words[0] & 0xFFFF;
    const unsigned slot = (isconn.words[0] >> 16) & 0x3;
    CHECK(clientDevid != 0);
    CHECK_EQ(slot, 0u);
    auto conComplete = runCommand(client, CMD_CONCOMPL, {}, t);
    CHECK_EQ(conComplete.words[0], isconn.words[0]);

    auto accept = runCommand(host, CMD_HOST_ACCEPT, {}, t);
    CHECK_EQ(accept.words.size(), 1u);
    CHECK_EQ(accept.words[0] & 0xFFFF, clientDevid);

    // Host → client data (SEND_DATAW puts the host into a wait)
    // Header: byte count in the low 7 bits; payload "abcdefgh"
    auto sendw = runCommand(host, CMD_SEND_DATAW, { 8, 0x64636261, 0x68676665 }, t);
    CHECK_EQ(sendw.ack, cmdHdr(CMD_SEND_DATAW, 0) | 0x80);
    CHECK_EQ(host.comstate, comWaitEvent);

    t += 100; deliver();  // HOST_SEND reaches the client (who ACKs)

    auto crecv = runCommand(client, CMD_RECV_DATA, {}, t);
    CHECK_EQ(crecv.words.size(), 3u);
    CHECK_EQ(crecv.words[0], 8u);            // byte count header
    CHECK_EQ(crecv.words[1], 0x64636261u);   // "abcd"
    CHECK_EQ(crecv.words[2], 0x68676665u);   // "efgh"

    t += 100; deliver();  // CLIENT_ACK reaches the host

    // Client → host data: byte count encoded at the slot-dependent position
    const uint32_t clientHdr = 4u << (8 + slot * 5);
    runCommand(client, CMD_SEND_DATA, { clientHdr, 0x31303039 }, t);
    t += 100; deliver();

    // The host's wait now resolves with "data available", adapter-master
    CHECK(host.pollWaitEvent(t));
    const auto ev = takeDelivery(host);
    CHECK_EQ(ev.size(), 2u);
    CHECK_EQ(ev[0], 0x99660000u | CMD_RESP_DATA);
    CHECK_EQ(ev[1], BUSY_WORD);
    CHECK_EQ(host.comstate, comWaitCmd);

    auto hrecv = runCommand(host, CMD_RECV_DATA, {}, t);
    CHECK_EQ(hrecv.words.size(), 2u);
    CHECK_EQ(hrecv.words[0], 4u << (8 + slot * 5));  // per-slot bitfield
    CHECK_EQ(hrecv.words[1], 0x31303039u);

    // SLOTSTAT / SYSSTAT sanity
    auto sysstat = runCommand(client, CMD_SYSSTAT, {}, t);
    CHECK_EQ(sysstat.words[0], (5u << 24) | ((1u << slot) << 16) | clientDevid);
    auto slotstat = runCommand(host, CMD_SLOTSTAT, {}, t);
    CHECK_EQ(slotstat.words[0], 1u);

    // Client disconnects; host learns and frees the slot
    runCommand(client, CMD_DISCONNECT, { 1 }, t);
    CHECK_EQ(client.state, stIdle);
    t += 100; deliver();
    auto accept2 = runCommand(host, CMD_HOST_ACCEPT, {}, t);
    CHECK_EQ(accept2.words.size(), 0u);
}

//-//////////////////////////////////////////////////////////////////////////-//
// Wait-event gating: the WAIT-class ACK is staged in-band, and the GBA clocks
// exactly ONE more master transfer to read it before dropping to slave
// (sio32intr_clock_master state 1 -> 3). The adapter may only take the bus
// after that transfer (a premature flip discards the staged ACK and fights
// the GBA for SC); afterwards the GBA clocks nothing, so events deliver
// freely. Deadlines anchor at the first poll after the ACK read.
//-//////////////////////////////////////////////////////////////////////////-//

static void testWaitArming()
{
    // Idle WAIT: gated until the ACK-read transfer, then resolves immediately
    // with a disconnect notice.
    RfuCore c;
    c.rand16 = &fixedRand;
    c.reset();
    login(c);
    runCommand(c, CMD_SYSVER, {}, 0);

    xfer(c, cmdHdr(CMD_WAIT, 0), 100);
    CHECK_EQ(c.comstate, comWaitEvent);
    CHECK_EQ(c.stagedTx, cmdHdr(CMD_WAIT, 0) | 0x80);  // ACK staged in-band
    CHECK(!c.pollWaitEvent(101));                      // ACK not read yet
    CHECK_EQ(xfer(c, BUSY_WORD, 102), cmdHdr(CMD_WAIT, 0) | 0x80);  // ACK read
    CHECK(c.pollWaitEvent(103));
    CHECK_EQ(c.eventWordCount(), 3u);
    CHECK_EQ(c.eventWord(0), 0x99660129u);  // idle → disconnect notice
    takeDelivery(c);
    CHECK_EQ(c.comstate, comWaitCmd);

    // Deadline anchoring: connected client, WAIT at t=1000, first poll at
    // t=1300 → the 533 ms timeout counts from 1300.
    RfuCore c2;
    c2.rand16 = &fixedRand;
    std::vector<CapturedFrame> sink;
    c2.emit = [](void* ctx, const uint8_t* f, size_t l) {
        static_cast<std::vector<CapturedFrame>*>(ctx)->push_back(
            { std::vector<uint8_t>(f, f + l) });
    };
    c2.emitCtx = &sink;
    c2.reset();
    login(c2);
    runCommand(c2, CMD_SYSVER, {}, 0);
    const uint8_t zeros[24] = {};
    c2.applyNetPacket(NET_BROADCAST, 0xAAAA, zeros, 24, 0);
    runCommand(c2, CMD_CONNECT, { 0xAAAA }, 0);
    c2.applyNetPacket(NET_CONNECT_ACK, 0x00001234, zeros, 4, 0);

    xfer(c2, cmdHdr(CMD_WAIT, 0), 1000);
    CHECK(!c2.pollWaitEvent(1250));         // gated: ACK not read yet
    xfer(c2, BUSY_WORD, 1290);              // GBA reads the ACK
    CHECK(!c2.pollWaitEvent(1300));         // arms here
    CHECK(!c2.pollWaitEvent(1820));         // still inside the 533ms window
    CHECK(c2.pollWaitEvent(1834));          // TIMEO anchored at 1300
    CHECK_EQ(c2.eventWord(0), 0x99660000u | CMD_RESP_TIMEO);
    takeDelivery(c2);

    // 0x35 is a wait-class alias: the ACK must reverse roles too.
    RfuCore c3;
    c3.rand16 = &fixedRand;
    c3.reset();
    login(c3);
    runCommand(c3, CMD_SYSVER, {}, 0);
    xfer(c3, cmdHdr(CMD_WAIT2, 0), 100);
    CHECK_EQ(c3.comstate, comWaitEvent);
}

//-//////////////////////////////////////////////////////////////////////////-//
// Connect retrying: a NACK (target mid-scan) keeps stConnecting and tick()
// re-emits CONNECT_REQ — the RF-retry analog — until the budget expires.
//-//////////////////////////////////////////////////////////////////////////-//

static void testConnectRetry()
{
    std::vector<CapturedFrame> sink;
    RfuCore c;
    c.rand16 = &fixedRand;
    c.emit = [](void* ctx, const uint8_t* f, size_t l) {
        static_cast<std::vector<CapturedFrame>*>(ctx)->push_back(
            { std::vector<uint8_t>(f, f + l) });
    };
    c.emitCtx = &sink;
    c.reset();
    login(c);
    const uint8_t z24[24] = {};
    c.applyNetPacket(NET_BROADCAST, 0xAAAA, z24, 24, 0);

    sink.clear();
    runCommand(c, CMD_CONNECT, { 0xAAAA }, 100);
    CHECK_EQ(sink.size(), 1u);
    CHECK_EQ(c.state, stConnecting);

    // NACK does NOT fail the attempt...
    c.applyNetPacket(NET_CONNECT_NACK, 0, z24, 4, 150);
    CHECK_EQ(c.state, stConnecting);

    // ...tick() re-emits after the retry cadence...
    c.tick(200);            // arms the retry clock
    c.tick(400);
    c.tick(600);
    CHECK(sink.size() >= 2);

    // ...an ACK arriving mid-retry connects normally...
    c.applyNetPacket(NET_CONNECT_ACK, 0x00001234, z24, 4, 700);
    CHECK_EQ(c.state, stClient);

    // ...and with no ACK ever, the budget expires to stIdle (ISCONNECTED
    // then reports CONN_FAILED = the in-game "busy").
    RfuCore c2;
    c2.rand16 = &fixedRand;
    c2.emit = c.emit;
    c2.emitCtx = &sink;
    c2.reset();
    login(c2);
    c2.applyNetPacket(NET_BROADCAST, 0xBBBB, z24, 24, 0);
    runCommand(c2, CMD_CONNECT, { 0xBBBB }, 1000);
    for (uint32_t t = 1000; t <= 6000; t += 100) c2.tick(t);
    CHECK_EQ(c2.state, stIdle);
    auto isc = runCommand(c2, CMD_ISCONNECTED, {}, 6100);
    CHECK_EQ(isc.words[0], CONN_FAILED);
}

//-//////////////////////////////////////////////////////////////////////////-//
// ID exchange restarts when the game re-runs checkID (soft reset without a
// GPIO pulse), including from the middle of a wait. Also proves the canned
// prefix self-aligns for any GBA first word.
//-//////////////////////////////////////////////////////////////////////////-//

static void testLoginRestart()
{
    RfuCore c;
    c.rand16 = &fixedRand;
    c.reset();
    login(c);
    runCommand(c, CMD_SYSVER, {}, 0);
    runCommand(c, CMD_WAIT, {}, 0);
    CHECK_EQ(c.comstate, comWaitEvent);

    // The game reset and restarts checkID while we were waiting: a NINTENDO
    // word (not a 0x9966 command) drops the adapter back into the ID-wait
    // phase, from which a full ID exchange reaches the command phase again.
    xfer(c, 0x7FFF494E, 10);
    CHECK_EQ(c.comstate, comIdWait);
    login(c, 10);
    CHECK_EQ(c.comstate, comWaitCmd);
}

static void testIdPrefixAlignment()
{
    // The canned dance starts only after the GBA's first 0x494E, so detection
    // succeeds regardless of the GBA's leftover initial SIODATA32.
    for (uint32_t first : { 0x00000000u, 0xFFFFFFFFu, 0x1234ABCDu, 0x0000494Eu })
    {
        RfuCore c;
        c.rand16 = &fixedRand;
        c.reset();
        login(c, 0, first);
        CHECK_EQ(c.comstate, comWaitCmd);
    }
}

//-//////////////////////////////////////////////////////////////////////////-//
// 5-player room: a host fills 4 client slots, NACKs the 5th, concatenates
// RECV data in slot order, emits exactly ONE HOST_SEND regardless of client
// count (the relay fans it out), and per-slot disconnects free slots without
// touching the others.
//-//////////////////////////////////////////////////////////////////////////-//

static void testMultiClient()
{
    std::vector<CapturedFrame> sink;
    RfuCore host;
    host.rand16 = &fixedRand;
    host.emit = [](void* ctx, const uint8_t* f, size_t l) {
        static_cast<std::vector<CapturedFrame>*>(ctx)->push_back(
            { std::vector<uint8_t>(f, f + l) });
    };
    host.emitCtx = &sink;
    host.reset();
    login(host);
    runCommand(host, CMD_SYSVER, {}, 0);
    runCommand(host, CMD_HOST_START, {}, 0);

    // Four joiners get slots 0-3 (ACK hdata = devid | slot<<16); a fifth is
    // NACK'd.
    const uint8_t zeros[4] = {};
    uint16_t devids[4] = {};
    for (unsigned i = 0; i < 4; i++)
    {
        sink.clear();
        host.applyNetPacket(NET_CONNECT_REQ, 0, zeros, 4, 0);
        CHECK_EQ(sink.size(), 1u);
        CHECK_EQ(unpack32be(&sink[0].bytes[4]), NET_CONNECT_ACK);
        const uint32_t hdata = unpack32be(&sink[0].bytes[8]);
        CHECK_EQ((hdata >> 16) & 0x3, i);
        devids[i] = hdata & 0xFFFF;
        CHECK(devids[i] != 0);
    }
    sink.clear();
    host.applyNetPacket(NET_CONNECT_REQ, 0, zeros, 4, 0);
    CHECK_EQ(sink.size(), 1u);
    CHECK_EQ(unpack32be(&sink[0].bytes[4]), NET_CONNECT_NACK);

    auto accept = runCommand(host, CMD_HOST_ACCEPT, {}, 0);
    CHECK_EQ(accept.words.size(), 4u);

    // One SEND_DATA = ONE frame on the wire, even with 4 clients connected.
    sink.clear();
    runCommand(host, CMD_SEND_DATA, { 4, 0xAABBCCDD }, 0);
    CHECK_EQ(sink.size(), 1u);
    CHECK_EQ(unpack32be(&sink[0].bytes[4]), NET_HOST_SEND);

    // Clients 1 and 3 send data; RECV concatenates in slot order with the
    // per-slot byte counts in the header bitfield.
    uint8_t p1[92] = { 0xDD, 0xEE };
    host.applyNetPacket(NET_CLIENT_SEND,
                        (2u << 24) | (1u << 16) | devids[1], p1, 92, 0);
    uint8_t p3[92] = { 0xAA, 0xBB, 0xCC };
    host.applyNetPacket(NET_CLIENT_SEND,
                        (3u << 24) | (3u << 16) | devids[3], p3, 92, 0);

    auto recv = runCommand(host, CMD_RECV_DATA, {}, 0);
    CHECK_EQ(recv.words.size(), 3u);
    CHECK_EQ(recv.words[0], (2u << (8 + 1 * 5)) | (3u << (8 + 3 * 5)));
    // Byte-level concatenation in slot order: DD EE AA BB CC, repacked LE.
    CHECK_EQ(recv.words[1], 0xBBAAEEDDu);
    CHECK_EQ(recv.words[2], 0x000000CCu);

    // Host disconnects slot 1 only; slots 0/2/3 stay.
    sink.clear();
    runCommand(host, CMD_DISCONNECT, { 1u << 1 }, 0);
    CHECK_EQ(sink.size(), 1u);
    CHECK_EQ(unpack32be(&sink[0].bytes[4]), NET_DISCONNECT);
    CHECK_EQ(unpack32be(&sink[0].bytes[8]), devids[1] | (1u << 16));
    auto accept2 = runCommand(host, CMD_HOST_ACCEPT, {}, 0);
    CHECK_EQ(accept2.words.size(), 3u);

    // A client-side DISCONNECT for someone ELSE must not tear us down.
    RfuCore cl;
    cl.rand16 = &fixedRand;
    std::vector<CapturedFrame> clSink;
    cl.emit = [](void* ctx, const uint8_t* f, size_t l) {
        static_cast<std::vector<CapturedFrame>*>(ctx)->push_back(
            { std::vector<uint8_t>(f, f + l) });
    };
    cl.emitCtx = &clSink;
    cl.reset();
    login(cl);
    const uint8_t z24[24] = {};
    cl.applyNetPacket(NET_BROADCAST, 0xAAAA, z24, 24, 0);
    runCommand(cl, CMD_CONNECT, { 0xAAAA }, 0);
    cl.applyNetPacket(NET_CONNECT_ACK, 0x00021234, z24, 4, 0);
    CHECK_EQ(cl.state, stClient);
    cl.applyNetPacket(NET_DISCONNECT, 0x0000BEEF, z24, 4, 0);  // someone else
    CHECK_EQ(cl.state, stClient);
    cl.applyNetPacket(NET_DISCONNECT, 0x00021234, z24, 4, 0);  // us
    CHECK_EQ(cl.state, stIdle);
}

static void testMaxPlayers()
{
    // SYSCFG bits 16-17 cap the joinable slots: 0b11 = 2 players = 1 client.
    std::vector<CapturedFrame> sink;
    RfuCore host;
    host.rand16 = &fixedRand;
    host.emit = [](void* ctx, const uint8_t* f, size_t l) {
        static_cast<std::vector<CapturedFrame>*>(ctx)->push_back(
            { std::vector<uint8_t>(f, f + l) });
    };
    host.emitCtx = &sink;
    host.reset();
    login(host);
    runCommand(host, CMD_SYSCFG, { 0x00030420 }, 0);  // maxPlayers bits = 3
    runCommand(host, CMD_HOST_START, {}, 0);

    const uint8_t zeros[4] = {};
    sink.clear();
    host.applyNetPacket(NET_CONNECT_REQ, 0, zeros, 4, 0);
    CHECK_EQ(unpack32be(&sink[0].bytes[4]), NET_CONNECT_ACK);
    sink.clear();
    host.applyNetPacket(NET_CONNECT_REQ, 0, zeros, 4, 0);
    CHECK_EQ(unpack32be(&sink[0].bytes[4]), NET_CONNECT_NACK);

    // CFGSTAT echoes the raw SYSCFG word (host shape: bdata + cfg + 0x101).
    auto cfg = runCommand(host, CMD_CFGSTAT, {}, 0);
    CHECK_EQ(cfg.words.size(), 8u);
    CHECK_EQ(cfg.words[6], 0x00030420u);
    CHECK_EQ(cfg.words[7], 0x101u);
}

static void testPeerTable()
{
    // Broadcasts from distinct devids fill distinct peer slots (mutual
    // visibility in a 5-member room); re-broadcasts update in place; the
    // scan response lists them all.
    RfuCore c;
    c.rand16 = &fixedRand;
    c.reset();
    login(c);
    runCommand(c, CMD_SYSVER, {}, 0);

    uint8_t bcast[24] = {};
    for (uint16_t id = 1; id <= 4; id++)
    {
        bcast[3] = static_cast<uint8_t>(id);  // word0 BE low byte = marker
        c.applyNetPacket(NET_BROADCAST, id, bcast, 24, 100 * id);
    }
    // Same devid again: update, not duplicate.
    c.applyNetPacket(NET_BROADCAST, 2, bcast, 24, 900);

    runCommand(c, CMD_BCRD_START, {}, 1000);
    auto fetch = runCommand(c, CMD_BCRD_FETCH, {}, 1000);
    CHECK_EQ(fetch.words.size(), 28u);  // 4 peers x (devid + 6 words)
    bool seen[5] = {};
    for (size_t o = 0; o < fetch.words.size(); o += 7)
    {
        const uint16_t id = fetch.words[o] & 0xFFFF;
        CHECK(id >= 1 && id <= 4);
        seen[id] = true;
    }
    CHECK(seen[1] && seen[2] && seen[3] && seen[4]);

    // A 5th distinct devid evicts the stalest entry (devid 1, last seen at
    // t=100; devid 2 was refreshed at t=900).
    c.applyNetPacket(NET_BROADCAST, 5, bcast, 24, 1100);
    auto fetch2 = runCommand(c, CMD_BCRD_FETCH, {}, 1200);
    CHECK_EQ(fetch2.words.size(), 28u);
    bool seen2[6] = {};
    for (size_t o = 0; o < fetch2.words.size(); o += 7)
        seen2[fetch2.words[o] & 0xFFFF] = true;
    CHECK(!seen2[1] && seen2[2] && seen2[3] && seen2[4] && seen2[5]);
}

static void testSendClamp()
{
    // A hostile/buggy plen up to 255 must not overrun the 23-word staging
    // buffer (gpsp has the same latent overflow; here it is clamped) — and
    // the core must keep functioning afterwards.
    std::vector<CapturedFrame> sink;
    RfuCore host;
    host.rand16 = &fixedRand;
    host.emit = [](void* ctx, const uint8_t* f, size_t l) {
        static_cast<std::vector<CapturedFrame>*>(ctx)->push_back(
            { std::vector<uint8_t>(f, f + l) });
    };
    host.emitCtx = &sink;
    host.reset();
    login(host);
    runCommand(host, CMD_HOST_START, {}, 0);
    const uint8_t zeros[4] = {};
    host.applyNetPacket(NET_CONNECT_REQ, 0, zeros, 4, 0);

    std::vector<uint32_t> huge(200, 0x41414141u);
    huge[0] = 90;  // header: 90 bytes claimed
    runCommand(host, CMD_SEND_DATA, huge, 10);

    // Still alive and consistent: the next command round-trips normally.
    auto r = runCommand(host, CMD_SYSVER, {}, 20);
    CHECK_EQ(r.words.size(), 1u);
    CHECK_EQ(r.words[0], 0x00830117u);
    CHECK_EQ(host.comstate, comWaitCmd);

    // Client-shape CFGSTAT: 7 words, six zeros + 0x101.
    RfuCore cl;
    cl.rand16 = &fixedRand;
    cl.reset();
    login(cl);
    const uint8_t z24[24] = {};
    cl.applyNetPacket(NET_BROADCAST, 0xAAAA, z24, 24, 0);
    runCommand(cl, CMD_CONNECT, { 0xAAAA }, 0);
    cl.applyNetPacket(NET_CONNECT_ACK, 0x00001234, z24, 4, 0);
    auto cfg = runCommand(cl, CMD_CFGSTAT, {}, 0);
    CHECK_EQ(cfg.words.size(), 7u);
    for (int j = 0; j < 6; j++) CHECK_EQ(cfg.words[j], 0u);
    CHECK_EQ(cfg.words[6], 0x101u);
}

//-//////////////////////////////////////////////////////////////////////////-//
// Inbound FIFO: two distinct frames landing between two RECV polls must BOTH
// be delivered, in order, exactly once (gpsp pkts[4] semantics). A latest-
// wins buffer skipped the first — fatal for the games' seq'd UNI command and
// block protocols (chat exit, trade data), which assume the lossless shared
// RF frame clock.
//-//////////////////////////////////////////////////////////////////////////-//

static void testInboundFifo()
{
    const uint8_t f1[4] = { 0x11, 0x22, 0x33, 0x44 };
    const uint8_t f2[4] = { 0x55, 0x66, 0x77, 0x88 };
    const uint8_t z24[24] = {};

    // Client side: two HOST_SENDs land before the game polls.
    RfuCore c;
    c.rand16 = &fixedRand;
    c.reset();
    login(c);
    c.applyNetPacket(NET_BROADCAST, 0xAAAA, z24, 24, 0);
    runCommand(c, CMD_CONNECT, { 0xAAAA }, 0);
    c.applyNetPacket(NET_CONNECT_ACK, 0x00001234, z24, 4, 0);
    CHECK_EQ(c.state, stClient);

    c.applyNetPacket(NET_HOST_SEND, 4, f1, 4, 10);
    c.applyNetPacket(NET_HOST_SEND, 4, f2, 4, 20);

    auto r1 = runCommand(c, CMD_RECV_DATA, {}, 30);
    CHECK_EQ(r1.words[0], 4u);
    CHECK_EQ(r1.words[1], 0x44332211u);
    auto r2 = runCommand(c, CMD_RECV_DATA, {}, 40);
    CHECK_EQ(r2.words[0], 4u);
    CHECK_EQ(r2.words[1], 0x88776655u);
    auto r3 = runCommand(c, CMD_RECV_DATA, {}, 50);  // drained: zero count
    CHECK_EQ(r3.words[0], 0u);

    // Host side: two CLIENT_SENDs from the same slot queue and pop in order.
    RfuCore h;
    h.rand16 = &fixedRand;
    h.reset();
    login(h);
    runCommand(h, CMD_HOST_START, {}, 0);
    h.applyNetPacket(NET_CONNECT_REQ, 0, z24, 4, 0);
    auto acc = runCommand(h, CMD_HOST_ACCEPT, {}, 0);
    CHECK_EQ(acc.words.size(), 1u);
    const uint16_t devid = acc.words[0] & 0xFFFF;

    const uint32_t hd = (4u << 24) | devid;  // blen=4, slot 0
    h.applyNetPacket(NET_CLIENT_SEND, hd, f1, 4, 10);
    h.applyNetPacket(NET_CLIENT_SEND, hd, f2, 4, 20);

    auto hr1 = runCommand(h, CMD_RECV_DATA, {}, 30);
    CHECK_EQ(hr1.words[0], 4u << 8);         // per-slot byte-count bitfield
    CHECK_EQ(hr1.words[1], 0x44332211u);
    auto hr2 = runCommand(h, CMD_RECV_DATA, {}, 40);
    CHECK_EQ(hr2.words[0], 4u << 8);
    CHECK_EQ(hr2.words[1], 0x88776655u);
    auto hr3 = runCommand(h, CMD_RECV_DATA, {}, 50);
    CHECK_EQ(hr3.words[0], 0u);

    // Zero frames never stack on the client: they only tick the pump, so
    // one is enqueued only when the queue is empty — a save-frozen game
    // must not have its FIFO pegged by the peer's 60Hz carrier while the
    // peer's REAL end-sequence frames tail-drop behind the zeros.
    {
        const uint8_t zf[4] = { 0, 0, 0, 0 };
        const uint8_t rf[4] = { 0x20, 0xBE, 0x11, 7 };
        for (int i = 0; i < 5; i++)
            c.applyNetPacket(NET_HOST_SEND, 4, zf, 4, 60 + i);   // 1 queues
        c.applyNetPacket(NET_HOST_SEND, 4, rf, 4, 70);           // real queues
        for (int i = 0; i < 3; i++)
            c.applyNetPacket(NET_HOST_SEND, 4, zf, 4, 80 + i);   // all skipped
        auto q1 = runCommand(c, CMD_RECV_DATA, {}, 90);
        CHECK_EQ(q1.words[0], 4u);
        CHECK_EQ(q1.words[1], 0u);                               // the one zero
        auto q2 = runCommand(c, CMD_RECV_DATA, {}, 91);
        CHECK_EQ(q2.words[1] & 0xFFFF, 0xBE20u);                 // then the real
        auto q3 = runCommand(c, CMD_RECV_DATA, {}, 92);
        CHECK_EQ(q3.words[0], 0u);                               // drained
    }

    // Overflow: gpsp tail-drops — depth 32, the 33rd pending frame is shed
    // and counted, the queued 32 survive intact.
    for (uint8_t i = 0; i < 33; i++)
    {
        const uint8_t fx[4] = { i, 0xAA, 0xBB, 0xCC };
        c.applyNetPacket(NET_HOST_SEND, 4, fx, 4, 100 + i);
    }
    CHECK_EQ(c.counters.rxDropQueueFull, 1u);
    for (uint8_t i = 0; i < 32; i++)
    {
        auto rr = runCommand(c, CMD_RECV_DATA, {}, 200 + i);
        CHECK_EQ(rr.words[0], 4u);
        CHECK_EQ(rr.words[1] & 0xFF, i);
    }
}

//-//////////////////////////////////////////////////////////////////////////-//
// Backpressure: a deep child FIFO emits NET_FLOWCTL(1) toward the host; the
// host paces its wait-data events to ~33ms while held (production dips below
// the child's consumption so the standing backlog — hidden inter-game lag —
// drains); the child clears at low water; the hold auto-expires.
//-//////////////////////////////////////////////////////////////////////////-//

static void testFlowControl()
{
    std::vector<CapturedFrame> sink;
    const auto countPtype = [&](uint32_t pt) {
        size_t n = 0;
        for (const auto& fr : sink)
            if (unpack32be(&fr.bytes[4]) == pt) n++;
        return n;
    };
    const auto lastHdata = [&](uint32_t pt) -> uint32_t {
        uint32_t h = 0xFFFFFFFF;
        for (const auto& fr : sink)
            if (unpack32be(&fr.bytes[4]) == pt) h = unpack32be(&fr.bytes[8]);
        return h;
    };

    // Child side: hint at high water, re-hint while deep, clear at low water.
    RfuCore c;
    c.rand16 = &fixedRand;
    c.emit = [](void* ctx, const uint8_t* f, size_t l) {
        static_cast<std::vector<CapturedFrame>*>(ctx)->push_back(
            { std::vector<uint8_t>(f, f + l) });
    };
    c.emitCtx = &sink;
    c.reset();
    login(c);
    const uint8_t z24[24] = {};
    c.applyNetPacket(NET_BROADCAST, 0xAAAA, z24, 24, 0);
    runCommand(c, CMD_CONNECT, { 0xAAAA }, 0);
    c.applyNetPacket(NET_CONNECT_ACK, 0x00001234, z24, 4, 0);

    sink.clear();
    for (uint8_t i = 0; i < 8; i++)
    {
        const uint8_t fx[4] = { i, 1, 2, 3 };
        c.applyNetPacket(NET_HOST_SEND, 4, fx, 4, 100 + i);
    }
    c.tick(110);
    // Gentle symmetric backpressure: standing depth >= 6 hints the peer.
    CHECK_EQ(countPtype(NET_FLOWCTL), 1u);
    CHECK_EQ(lastHdata(NET_FLOWCTL) & 1u, 1u);
    c.tick(150);                                  // inside the re-hint window
    CHECK_EQ(countPtype(NET_FLOWCTL), 1u);
    c.tick(300);                                  // re-hint while still deep
    CHECK_EQ(countPtype(NET_FLOWCTL), 2u);
    for (int i = 0; i < 6; i++) runCommand(c, CMD_RECV_DATA, {}, 400 + i);
    c.tick(410);                                  // depth 2 -> clear
    CHECK_EQ(countPtype(NET_FLOWCTL), 3u);
    CHECK_EQ(lastHdata(NET_FLOWCTL) & 1u, 0u);
    for (int i = 0; i < 6; i++) runCommand(c, CMD_RECV_DATA, {}, 420 + i);

    // Host side: data events paced at ~33ms while held; hold expires.
    RfuCore h;
    h.rand16 = &fixedRand;
    h.emit = c.emit;
    h.emitCtx = &sink;
    h.reset();
    login(h);
    runCommand(h, CMD_HOST_START, {}, 0);
    const uint8_t z4[4] = {};
    h.applyNetPacket(NET_CONNECT_REQ, 0, z4, 4, 0);
    auto acc = runCommand(h, CMD_HOST_ACCEPT, {}, 0);
    const uint16_t devid = acc.words[0] & 0xFFFF;
    const uint32_t hd = (4u << 24) | devid;

    // Non-zero frames: zero-cmd child frames are no-ops and never queue.
    const uint8_t k1[4] = { 0x00, 0xBE, 0x11, 1 };
    const uint8_t k2[4] = { 0x20, 0xBE, 0x11, 2 };
    const uint8_t k3[4] = { 0x40, 0xBE, 0x11, 3 };
    // An inbound FLOWCTL applies the GENTLE trim: data events pace at 18ms
    // (~9% slower) — never the old half-rate brake — and the hold expires.
    h.applyNetPacket(NET_FLOWCTL, 1, z4, 4, 1000);
    h.applyNetPacket(NET_CLIENT_SEND, hd, k1, 4, 1000);
    h.applyNetPacket(NET_CLIENT_SEND, hd, k2, 4, 1001);

    uint32_t t = 1000;
    runCommand(h, CMD_SEND_DATAW, { 4, 0x1 }, t);
    CHECK(h.pollWaitEvent(t));                    // first event immediate
    CHECK_EQ(takeDelivery(h)[0], 0x99660000u | CMD_RESP_DATA);
    runCommand(h, CMD_RECV_DATA, {}, t);
    CHECK_EQ(h.dbgFlowHolds, 1u);

    runCommand(h, CMD_SEND_DATAW, { 4, 0x2 }, t + 2);
    CHECK(!h.pollWaitEvent(t + 2));               // trimmed: not at once...
    CHECK(!h.pollWaitEvent(t + 13));              // ...rtx must NOT preempt
    CHECK(h.pollWaitEvent(t + 18));               // the gentle gap
    CHECK_EQ(takeDelivery(h)[0], 0x99660000u | CMD_RESP_DATA);
    runCommand(h, CMD_RECV_DATA, {}, t + 18);

    // Hold expired (no re-hint): back to immediate host events.
    h.applyNetPacket(NET_CLIENT_SEND, hd, k3, 4, 1400);
    runCommand(h, CMD_SEND_DATAW, { 4, 0x3 }, 1400);
    CHECK(h.pollWaitEvent(1400));
    CHECK_EQ(takeDelivery(h)[0], 0x99660000u | CMD_RESP_DATA);
}

//-//////////////////////////////////////////////////////////////////////////-//
// Host stale-tail shed: a deep child slot sheds the oldest 8 SEQ-CARRYING
// frames (one full mod-8 wrap — invisible to the parent game's +1 check);
// zero-cmd frames are dropped alongside without counting toward the 8, and
// the surviving head carries the same 3-bit seq the old head did.
//-//////////////////////////////////////////////////////////////////////////-//

static void testHostStaleShed()
{
    RfuCore h;
    h.rand16 = &fixedRand;
    h.reset();
    login(h);
    runCommand(h, CMD_HOST_START, {}, 0);
    const uint8_t z4[4] = {};
    h.applyNetPacket(NET_CONNECT_REQ, 0, z4, 4, 0);
    auto acc = runCommand(h, CMD_HOST_ACCEPT, {}, 0);
    const uint16_t devid = acc.words[0] & 0xFFFF;
    const uint32_t hd = (4u << 24) | devid;

    // Held-keys frames: {seq<<5, 0xBE, key=0x11 (idle), marker}. 9 seq
    // STEPS with zero-cmd frames interleaved AND a duplicate of step 4
    // (same seq — the backup-queue re-send shape): below threshold, all
    // queue; the dup must not be miscounted as a step.
    for (uint8_t s = 0; s < 9; s++)
    {
        const uint8_t fr[4] = { static_cast<uint8_t>((s & 7) << 5), 0xBE, 0x11, s };
        h.applyNetPacket(NET_CLIENT_SEND, hd, fr, 4, 10 + s);
        if (s == 4)
            h.applyNetPacket(NET_CLIENT_SEND, hd, fr, 4, 10 + s);  // dup, same seq
        if (s % 3 == 0)
        {
            const uint8_t zf[4] = { 0, 0, 0, 0 };
            h.applyNetPacket(NET_CLIENT_SEND, hd, zf, 4, 10 + s);
        }
    }
    CHECK_EQ(h.dbgSlotOccupancy(), 1u);
    CHECK_EQ(h.dbgSheds, 0u);

    // The 10th seq STEP crosses hostShedAtSeqFrames: steps 1-8 (incl. the
    // dup and interleaved zeros) shed in one unit.
    const uint8_t fr9[4] = { static_cast<uint8_t>((9 & 7) << 5), 0xBE, 0x11, 9 };
    h.applyNetPacket(NET_CLIENT_SEND, hd, fr9, 4, 30);
    CHECK_EQ(h.dbgSheds, 1u);

    // Survivors: steps 8 and 9 — and step 8's 3-bit seq (8&7 = 0) equals
    // the shed head's seq (0): the wrap keeps the stream continuous.
    auto r1 = runCommand(h, CMD_RECV_DATA, {}, 40);
    CHECK_EQ(r1.words[0], 4u << 8);
    CHECK_EQ(r1.words[1] & 0xFF, (8u & 7) << 5);   // seq 0 again
    CHECK_EQ((r1.words[1] >> 24) & 0xFF, 8u);      // marker byte3
    auto r2 = runCommand(h, CMD_RECV_DATA, {}, 41);
    CHECK_EQ((r2.words[1] >> 24) & 0xFF, 9u);
    auto r3 = runCommand(h, CMD_RECV_DATA, {}, 42);
    CHECK_EQ(r3.words[0], 0u);                     // drained

    // NEGATIVE: block chunks (0x89) are data the parent's game WAITS on —
    // a deep queue of them must never shed, however deep.
    for (uint8_t s = 0; s < 14; s++)
    {
        const uint8_t fr[4] = { static_cast<uint8_t>((s & 7) << 5), 0x89, s, 0 };
        h.applyNetPacket(NET_CLIENT_SEND, hd, fr, 4, 100 + s);
    }
    CHECK_EQ(h.dbgSheds, 1u);                      // unchanged
    auto rb = runCommand(h, CMD_RECV_DATA, {}, 200);
    CHECK_EQ(rb.words[1] & 0xFF, 0u);              // oldest chunk survived
    for (int i = 0; i < 13; i++) runCommand(h, CMD_RECV_DATA, {}, 210 + i);

    // Zero-drop boundary: a FULLY-zero frame never queues; frames with only
    // byte0 set must queue — the LL comm=0 close (80 00) and zero-cmd
    // frames carrying an ack nibble both live there.
    const uint8_t zz[4] = { 0, 0, 0, 0 };
    const uint8_t llClose[2] = { 0x80, 0x00 };
    const uint8_t ackOnly[4] = { 0x03, 0x00, 0, 0 };
    h.applyNetPacket(NET_CLIENT_SEND, (4u << 24) | devid, zz, 4, 300);
    auto rz = runCommand(h, CMD_RECV_DATA, {}, 301);
    CHECK_EQ(rz.words[0], 0u);                     // dropped
    h.applyNetPacket(NET_CLIENT_SEND, (2u << 24) | devid, llClose, 2, 310);
    auto rc = runCommand(h, CMD_RECV_DATA, {}, 311);
    CHECK_EQ(rc.words[0], 2u << 8);
    CHECK_EQ(rc.words[1] & 0xFFFF, 0x0080u);       // 80 00 delivered
    h.applyNetPacket(NET_CLIENT_SEND, (4u << 24) | devid, ackOnly, 4, 320);
    auto ra = runCommand(h, CMD_RECV_DATA, {}, 321);
    CHECK_EQ(ra.words[0], 4u << 8);
    CHECK_EQ(ra.words[1] & 0xFF, 0x03u);           // ack nibble delivered
}

//-//////////////////////////////////////////////////////////////////////////-//
// Idle re-broadcast: a connected adapter whose game goes quiet (flash save)
// keeps re-emitting its last ALL-ZERO UNI frame at RF cadence — the real
// radio's carrier, which the peer games' ~5s link supervision is calibrated
// against. Non-zero buffers are never replayed (a replayed child seq would
// trip the parent's duplicate detection).
//-//////////////////////////////////////////////////////////////////////////-//

static void testIdleRetransmit()
{
    std::vector<CapturedFrame> sink;
    RfuCore c;
    c.rand16 = &fixedRand;
    c.emit = [](void* ctx, const uint8_t* f, size_t l) {
        static_cast<std::vector<CapturedFrame>*>(ctx)->push_back(
            { std::vector<uint8_t>(f, f + l) });
    };
    c.emitCtx = &sink;
    c.reset();
    login(c);
    const uint8_t z24[24] = {};
    c.applyNetPacket(NET_BROADCAST, 0xAAAA, z24, 24, 0);
    runCommand(c, CMD_CONNECT, { 0xAAAA }, 0);
    c.applyNetPacket(NET_CONNECT_ACK, 0x00001234, z24, 4, 0);
    CHECK_EQ(c.state, stClient);

    const auto countSends = [&]() {
        size_t n = 0;
        for (const auto& fr : sink)
            if (unpack32be(&fr.bytes[4]) == NET_CLIENT_SEND) n++;
        return n;
    };

    // Game stages an all-zero 14-byte UNI frame (idle), then goes silent.
    const uint32_t hdr = 14u << 8;  // byte count at the slot-0 position
    runCommand(c, CMD_SEND_DATA, { hdr, 0, 0, 0, 0 }, 100);
    c.tick(100);                      // stamps the send
    const size_t base = countSends();
    CHECK_EQ(base, 1u);

    // The carrier engages only after a genuine quiet period (60ms) — at one
    // frame period it would race the live 59.7Hz stream and halve real
    // throughput — then runs at RF cadence.
    c.tick(117);                      // one frame gap: active-stream range
    CHECK_EQ(countSends(), base);
    c.tick(140);                      // still inside the engage threshold
    CHECK_EQ(countSends(), base);
    c.tick(160);                      // 60ms quiet: carrier engages
    CHECK_EQ(countSends(), base + 1);
    c.tick(170);
    CHECK_EQ(countSends(), base + 1);
    c.tick(177);                      // now at RF cadence
    CHECK_EQ(countSends(), base + 2);
    CHECK(c.dbgIdleRetx >= 2);

    // A NON-zero buffer is never repeated: the carrier synthesizes an
    // ALL-ZERO frame of the same shape instead (cadence without content —
    // repeated non-zero frames enqueue into the peer game's 20-slot queue
    // and, mid-block, ratchet it into the latched-full reboot fatal; a
    // repeated child seq additionally trips the parent's dup detection).
    runCommand(c, CMD_SEND_DATA, { hdr, 0x0000BE20, 0x2C11, 0, 0 }, 300);
    c.tick(300);
    const size_t afterKeys = countSends();
    c.tick(360);
    CHECK_EQ(countSends(), afterKeys + 1);
    // The carrier frame's payload must be all-zero (RFU1 data payload
    // starts at byte 12), while the real send before it had content.
    const auto& carrier = sink.back();
    CHECK_EQ(unpack32be(&carrier.bytes[4]), NET_CLIENT_SEND);
    for (int i = 0; i < 14; i++)
        CHECK_EQ(carrier.bytes[12 + i], 0u);

    // Host side: same synthesis for a quiet parent mid-content.
    RfuCore h;
    h.rand16 = &fixedRand;
    h.emit = c.emit;
    h.emitCtx = &sink;
    h.reset();
    login(h);
    runCommand(h, CMD_HOST_START, {}, 0);
    const uint8_t z4[4] = {};
    h.applyNetPacket(NET_CONNECT_REQ, 0, z4, 4, 0);
    runCommand(h, CMD_HOST_ACCEPT, {}, 0);

    const auto countHostSends = [&]() {
        size_t n = 0;
        for (const auto& fr : sink)
            if (unpack32be(&fr.bytes[4]) == NET_HOST_SEND) n++;
        return n;
    };
    runCommand(h, CMD_SEND_DATA, { 14, 0x00008920, 0x1234, 0, 0 }, 400);
    h.tick(400);
    const size_t hostBase = countHostSends();
    CHECK_EQ(hostBase, 1u);
    h.tick(430);                       // inside the engage threshold
    CHECK_EQ(countHostSends(), hostBase);
    h.tick(460);                       // 60ms quiet: zero carrier, same shape
    CHECK_EQ(countHostSends(), hostBase + 1);
    const auto& hc = sink.back();
    CHECK_EQ(unpack32be(&hc.bytes[4]), NET_HOST_SEND);
    for (int i = 0; i < 14; i++)
        CHECK_EQ(hc.bytes[12 + i], 0u);
    h.tick(477);                       // RF cadence once engaged
    CHECK_EQ(countHostSends(), hostBase + 2);

    // A ZERO-LENGTH send ("nothing to say" — the win/save screens issue
    // these) must NOT disarm the carrier: it falls back to the natural
    // frame size. This exact gap dropped the peer to the 533ms crawl and
    // fired its ~6s giveup at battle end.
    runCommand(h, CMD_SEND_DATA, { 0, 0 }, 600);
    h.tick(600);
    const size_t afterEmpty = countHostSends();
    h.tick(660);                       // engages despite blen=0
    CHECK_EQ(countHostSends(), afterEmpty + 1);
    const auto& nc = sink.back();
    CHECK_EQ(unpack32be(&nc.bytes[4]), NET_HOST_SEND);
    CHECK_EQ(unpack32be(&nc.bytes[8]) & 0x7F, 70u);  // natural host size
    for (int i = 0; i < 70; i++)
        CHECK_EQ(nc.bytes[12 + i], 0u);
}

//-//////////////////////////////////////////////////////////////////////////-//
// Client data events are paced to one per RF frame (~16ms): a queued backlog
// must NOT resolve waits back-to-back — unpaced delivery ratchets the game's
// 20-slot recvQueue to its latched-full FATAL (the reboot screen).
//-//////////////////////////////////////////////////////////////////////////-//

static void testClientEventPacing()
{
    RfuCore c;
    c.rand16 = &fixedRand;
    c.reset();
    login(c);
    const uint8_t z24[24] = {};
    c.applyNetPacket(NET_BROADCAST, 0xAAAA, z24, 24, 0);
    runCommand(c, CMD_CONNECT, { 0xAAAA }, 0);
    c.applyNetPacket(NET_CONNECT_ACK, 0x00001234, z24, 4, 0);
    CHECK_EQ(c.state, stClient);

    const uint8_t f1[4] = { 0x11, 0x22, 0x33, 0x44 };
    c.applyNetPacket(NET_HOST_SEND, 4, f1, 4, 10);
    c.applyNetPacket(NET_HOST_SEND, 4, f1, 4, 12);

    // First wait: resolves immediately (pacer starts disarmed).
    uint32_t t = 1000;
    runCommand(c, CMD_WAIT, {}, t);
    CHECK(c.pollWaitEvent(t));
    CHECK_EQ(takeDelivery(c)[0], 0x99660000u | CMD_RESP_DATA);
    runCommand(c, CMD_RECV_DATA, {}, t);

    // Second frame is already queued, but the next data event must hold
    // until one RF frame has elapsed since the previous one.
    runCommand(c, CMD_WAIT, {}, t + 2);
    CHECK(!c.pollWaitEvent(t + 2));
    CHECK(!c.pollWaitEvent(t + 15));
    CHECK(c.pollWaitEvent(t + 16));
    CHECK_EQ(takeDelivery(c)[0], 0x99660000u | CMD_RESP_DATA);
}

int main()
{
    testFraming();
    testLoginAndCommands();
    testWaitResponses();
    testWaitArming();
    testEndToEnd();
    testLoginRestart();
    testIdPrefixAlignment();
    testMultiClient();
    testMaxPlayers();
    testPeerTable();
    testSendClamp();
    testInboundFifo();
    testClientEventPacing();
    testFlowControl();
    testHostStaleShed();
    testIdleRetransmit();
    testConnectRetry();

    if (g_failures == 0)
    {
        std::printf("All rfuProtocol tests passed.\n");
        return 0;
    }
    std::printf("%d failure(s).\n", g_failures);
    return 1;
}
