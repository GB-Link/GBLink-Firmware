// Host test for the link mode's handshake: the real PacketLayer and UsbSection,
// a modelled GBA on the other end of the cable, and the host's StartHandshake
// and ConnectLink sent at different distances from each other.
//
// The link layer (the PIO programs) is replaced by a thread that runs one
// transfer at a time at the packet layer's own timing, calling the callbacks
// exactly as layers/link/gbaLinkLayer.cpp does: transmit, receive, transive
// done. Zephyr's kernel calls are host stand-ins (stubs/).
//
//   make handshakeTest && ./handshakeTest

#include "../../src/sections/usbSection.hpp"
#include "../../src/layers/link/multiMode/multiMode.hpp"
#include "../../src/layers/link/gbaLinkLayer.hpp"
#include "../../src/layers/transport.hpp"
#include "../../src/linkStatus.hpp"
#include "../../src/link_defines.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

// ---- the kernel calls the stubs leave to us ---------------------------------

static std::recursive_mutex g_irq;

extern "C" unsigned int irq_lock(void) { g_irq.lock(); return 0; }
extern "C" void irq_unlock(unsigned int) { g_irq.unlock(); }
extern "C" int32_t k_sleep(k_timeout_t t)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(t.ms));
    return 0;
}

// ---- Transport: the statuses the host would see ------------------------------

static std::mutex g_statusLock;
static std::vector<uint16_t> g_statuses;

namespace Transport
{
    void setActive(Id) {}
    Id active() { return Id::Usb; }
    void registerCommandHandler(ReceiveHandler, void*) {}
    void registerDataHandler(ReceiveHandler, void*) {}

    bool sendStatus(std::span<const uint8_t, 2> data)
    {
        std::lock_guard<std::mutex> lock(g_statusLock);
        g_statuses.push_back(static_cast<uint16_t>(data[0] | data[1] << 8));
        return true;
    }

    bool sendData(std::span<const uint8_t>) { return true; }
    bool trySendData(std::span<const uint8_t>) { return true; }
}

static bool sawStatus(LinkStatus status)
{
    std::lock_guard<std::mutex> lock(g_statusLock);
    for (uint16_t s : g_statuses)
        if (s == static_cast<uint16_t>(status)) return true;
    return false;
}

// ---- the link layer: one transfer at a time ----------------------------------

static TransmitHandler g_tx;
static void* g_txData;
static ReceiveHandler g_rx;
static void* g_rxData;
static TransiveDoneHandler g_done;
static void* g_doneData;
static std::atomic<int> g_mode{DISABLED};

void link_setTransmitCallback(TransmitHandler cb, void* userData)
{
    std::lock_guard<std::recursive_mutex> lock(g_irq);
    g_tx = cb;
    g_txData = userData;
}

void link_setReceiveCallback(ReceiveHandler cb, void* userData)
{
    std::lock_guard<std::recursive_mutex> lock(g_irq);
    g_rx = cb;
    g_rxData = userData;
}

void link_setTransiveDoneCallback(TransiveDoneHandler cb, void* userData)
{
    std::lock_guard<std::recursive_mutex> lock(g_irq);
    g_done = cb;
    g_doneData = userData;
}

void multiMode_selectMode(enum MultiMode mode) { g_mode = mode; }

// The GBA. As a child (the adapter is the parent, MASTER) it answers 0xB9A0
// until it has seen the parent's 0x8FFF, then idles. As the parent (the
// adapter answers as the children, SLAVE) it sends 0xB9A0 until every child
// has answered 0xB9A0, then 0x8FFF, then idles.
struct Gba
{
    bool connected = false;
    int b9a0Seen = 0;              // as a child: 0xB9A0 the parent sent before 0x8FFF
    int connectAfter = 0;          // ...counted when 0x8FFF arrived

    uint16_t childAnswer(TxValues parentAndOthers)
    {
        if (connected) return 0x0000;
        if (parentAndOthers[0] == LINK_MASTER_HANDSHAKE)
        {
            connected = true;
            connectAfter = b9a0Seen;
            return LINK_SLAVE_HANDSHAKE;
        }
        if (parentAndOthers[0] == LINK_SLAVE_HANDSHAKE) b9a0Seen++;
        return LINK_SLAVE_HANDSHAKE;
    }

    uint16_t parentWord(bool childrenReady)
    {
        if (connected) return 0x0000;
        if (childrenReady)
        {
            connected = true;
            return LINK_MASTER_HANDSHAKE;
        }
        return LINK_SLAVE_HANDSHAKE;
    }
};

static Gba g_gba;
static std::atomic<bool> g_wireRun{true};
static std::atomic<int> g_transfers{0};

static void wire()
{
    bool childrenReady = false;
    while (g_wireRun)
    {
        uint32_t cycles = 29868;   // a handshake round, until the layer says otherwise
        {
            std::lock_guard<std::recursive_mutex> lock(g_irq);
            if (g_mode != DISABLED && g_tx && g_rx && g_done)
            {
                NextTransmit next = g_tx(g_txData);
                uint16_t rx;
                if (g_mode == MASTER)
                {
                    rx = g_gba.childAnswer(next.txValues);
                }
                else
                {
                    rx = g_gba.parentWord(childrenReady);
                    // The parent sends 0x8FFF once every child has answered
                    // 0xB9A0 in a round.
                    childrenReady = true;
                    for (uint16_t w : next.txValues)
                        if (w != LINK_SLAVE_HANDSHAKE) childrenReady = false;
                }
                g_rx(rx, g_rxData);
                g_done(rx, next.txValues, g_doneData);
                g_transfers++;
                if (next.timingUs) cycles = next.timingUs;
            }
        }
        // One PIO cycle is ~0.5425 us.
        std::this_thread::sleep_for(std::chrono::microseconds(cycles * 5425 / 10000));
    }
}

// ---- the cases -----------------------------------------------------------------

static int g_failed = 0;

static void check(const char* what, bool ok)
{
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) g_failed++;
}

static bool waitFor(LinkStatus status, int ms)
{
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end)
    {
        if (sawStatus(status)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return sawStatus(status);
}

// One session: the adapter in seat `seat` of `players`, the host sending
// StartHandshake and then ConnectLink `connectDelayMs` later.
static void session(uint8_t seat, uint8_t players, int connectDelayMs)
{
    {
        std::lock_guard<std::mutex> lock(g_statusLock);
        g_statuses.clear();
    }
    g_gba = Gba{};
    g_transfers = 0;

    char what[160];
    std::snprintf(what, sizeof what, "seat %u of %u, ConnectLink %d ms after StartHandshake",
                  seat, players, connectDelayMs);
    std::printf("%s\n", what);

    auto* section = new UsbSection(seat, players);
    std::thread wireThread(wire);
    std::thread proc([section] { section->process(); });

    // The host waits for the GBA's handshake, as Celio and the netplay bridge do.
    bool received = waitFor(LinkStatus::HandshakeReceived, 2000);
    check("HandshakeReceived", received);

    section->startHandshake();
    if (connectDelayMs) std::this_thread::sleep_for(std::chrono::milliseconds(connectDelayMs));
    section->connectLink();

    bool connected = waitFor(LinkStatus::LinkConnected, 3000);
    check("LinkConnected within 3 s", connected);
    if (seat != 0)
    {
        std::snprintf(what, sizeof what, "the GBA saw 0xB9A0 before 0x8FFF (%d rounds)", g_gba.connectAfter);
        check(what, g_gba.connected && g_gba.connectAfter >= 1);
    }
    else
    {
        check("the GBA, as parent, connected", g_gba.connected);
    }

    section->cancel();
    proc.join();
    delete section;            // the packet layer turns the link off on the way
    g_wireRun = false;
    wireThread.join();
    g_wireRun = true;
}

int main()
{
    // The adapter as the parent, the GBA a child: ConnectLink matters.
    for (int delay : {0, 5, 30, 60, 200})
        session(1, 2, delay);
    session(2, 4, 0);
    // The adapter as the children, the GBA the parent: ConnectLink is ignored.
    session(0, 2, 0);

    std::printf("%s\n", g_failed ? "FAILED" : "all checks passed");
    return g_failed ? 1 : 0;
}
