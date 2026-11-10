#include "gb.hpp"
#include "../linkStatus.hpp"
#include "../hardware.hpp"
#include "../persist.hpp"
#include "../layers/usbLayer.hpp"
#include "../layers/transport.hpp"
#include "../layers/link/gbaLinkLayer.hpp"
#include "../sections/gbProtocol.hpp"
#include "../sections/gbPrinter.hpp"

#include <algorithm>
#include <cstring>

extern "C"
{
    #include "../layers/link/gbLinkLayer.h"
    #include "hardware/timer.h"
}

// Max bytes per single USB transfer
static constexpr uint8_t MAX_TRANSFER_BYTES = 0x40;

// Data packets and the link's commands from the host, in order, so the host can send several without
// waiting for the answers, and a command acts between the packets around it. When all are taken, newer
// ones are dropped.
struct HostPacket
{
    bool command;
    uint8_t length;
    uint8_t data[MAX_TRANSFER_BYTES * 2];
};
K_MSGQ_DEFINE(g_hostPackets, sizeof(HostPacket), 24, 1);

// Answers for the host, in order, each sent once the host has taken the one before, so a host that
// reads late never holds up the link. When this is full, newer answers are dropped.
struct HostReply
{
    uint8_t length;
    uint8_t data[MAX_TRANSFER_BYTES];
};
K_MSGQ_DEFINE(g_hostReplies, sizeof(HostReply), 24, 1);

// Wakes the mode's loop: a host packet, bytes for the host from the Game Boy, a sub-mode change.
K_SEM_DEFINE(g_wake, 0, 1);

struct IrqGuard
{
    unsigned int key = irq_lock();
    ~IrqGuard() { irq_unlock(key); }
};

static gbproto::Link<IrqGuard> g_link;
static gbprinter::Printer g_printer;

// Passive mode also checks this often for answers to send and for a stuck transfer.
static constexpr k_timeout_t passivePoll = K_MSEC(10);

// A byte the Game Boy started but did not finish within this time leaves the passive program out of step.
static constexpr uint32_t passiveStallMs = 1000;

// The printer gets back in step sooner: a game that keeps asking for the printer leaves only short gaps.
static constexpr uint32_t printerStallMs = 20;

// A packet the Game Boy stopped sending for this long is dropped.
static constexpr int64_t printerAbortMs = 100;

// While the host has not taken earlier data yet, check this often whether it has.
static constexpr k_timeout_t hostPoll = K_MSEC(1);

static uint8_t passiveTransfer(uint8_t received)
{
    const size_t forHost = g_link.bytesForHost();
    const uint8_t next = g_link.onPassiveTransfer(received);
    if (g_link.bytesForHost() != forHost) k_sem_give(&g_wake);
    return next;
}

static int hybridTransfer(uint8_t received, bool clocked)
{
    const size_t forHost = g_link.bytesForHost();
    const int next = g_link.onHybridTransfer(received, clocked);
    if (g_link.bytesForHost() != forHost) k_sem_give(&g_wake);
    return next;
}

// Hybrid: the adapter's turn may have been waiting for the bytes just queued.
static void resumeHybrid()
{
    IrqGuard guard;
    const int next = g_link.resumeHybrid();
    if (next >= 0) gb_link_hybrid_clock(static_cast<uint8_t>(next));
}

static uint8_t printerTransfer(uint8_t received)
{
    const size_t forHost = g_printer.bytesForHost();
    const uint8_t next = g_printer.onByte(received);
    if (g_printer.bytesForHost() != forHost) k_sem_give(&g_wake);
    return next;
}

static void queueReply(const uint8_t* data, size_t length)
{
    HostReply reply;
    for (size_t sent = 0; sent < length; sent += reply.length)
    {
        reply.length = static_cast<uint8_t>(std::min(length - sent, sizeof(reply.data)));
        std::memcpy(reply.data, data + sent, reply.length);
        k_msgq_put(&g_hostReplies, &reply, K_NO_WAIT);
    }
}

// The answers first, then in regular passive mode the Game Boy's bytes. False while the host has not
// taken earlier data yet.
static bool sendToHost()
{
    HostReply reply;
    while (k_msgq_peek(&g_hostReplies, &reply) == 0)
    {
        if (!Transport::trySendData(std::span<const uint8_t>(reply.data, reply.length))) return false;
        k_msgq_get(&g_hostReplies, &reply, K_NO_WAIT);
    }

    uint8_t bytes[MAX_TRANSFER_BYTES];
    size_t count;
    while ((count = g_link.peekStream(bytes, sizeof(bytes))) > 0)
    {
        if (!Transport::trySendData(std::span<const uint8_t>(bytes, count))) return false;
        g_link.dropStream(count);
    }
    return true;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// USB Data Handler
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

// Runs on a USB thread or the bridge UART's receive thread, neither of which may wait here.
static void queueHostPacket(std::span<const uint8_t> data, bool command)
{
    HostPacket packet;
    packet.command = command;
    packet.length = static_cast<uint8_t>(std::min(data.size(), sizeof(packet.data)));
    std::memcpy(packet.data, data.data(), packet.length);
    if (k_msgq_put(&g_hostPackets, &packet, K_NO_WAIT) == 0) k_sem_give(&g_wake);
}

void GBModule::usbDataHandler(std::span<const uint8_t> data, void*)
{
    queueHostPacket(data, false);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// Module Interface
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

bool GBModule::canHandle(uint8_t command)
{
    return (command & 0xF0) == 0x30;
}

void GBModule::receiveCommand(std::span<const uint8_t> command)
{
    switch (static_cast<GBCommand>(command[0]))
    {
        case GBCommand::EnterPrinterMode:
            m_subMode = SubMode::printer;
            // Signal the normal mode loop to exit so we can switch
            k_sem_give(&g_wake);
            break;

        case GBCommand::ExitPrinterMode:
            m_subMode = SubMode::normal;
            break;

        default:
            // The link's own commands take their turn with the data packets (gbproto::Command)
            queueHostPacket(command, true);
            break;
    }
}

void GBModule::cancel()
{
    m_cancel = true;
    m_subMode = SubMode::normal;
    k_sem_give(&g_wake);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// Execute — Main Entry Point
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void GBModule::execute()
{
    m_cancel = false;
    k_sem_reset(&g_wake);
    k_msgq_purge(&g_hostPackets);
    k_msgq_purge(&g_hostReplies);
    g_link.reset();

    // Disable GBA PIO link before initializing GB SPI
    link_disable();

    // Initialize GB 8-bit SPI PIO on pio0
    gb_link_init();
    gb_link_set_passive_handler(passiveTransfer);
    gb_link_set_hybrid_handler(hybridTransfer);

    applyLedForSlot(LED_SLOT_GBC); // GB/GBC mode

    Transport::registerDataHandler(usbDataHandler, this);

    sendLinkStatus(LinkStatus::GBModeActive);

    while (!m_cancel)
    {
        switch (m_subMode)
        {
            case SubMode::normal:
                normalModeLoop();
                break;

            case SubMode::printer:
                sendLinkStatus(LinkStatus::GBPrinterModeActive);
                applyLedForSlot(LED_SLOT_PRINTER); // printer sub-mode

                printerModeLoop();

                gb_link_set_passive_handler(passiveTransfer);
                gb_link_set_passive(false, 0);
                g_link.returnToActive();

                applyLedForSlot(LED_SLOT_GBC); // back to GB mode
                m_subMode = SubMode::normal;
                sendLinkStatus(LinkStatus::GBModeActive);
                break;
        }
    }

    // Cleanup: deinit GB PIO so GBA can reclaim pio0
    gb_link_deinit();

    Transport::registerDataHandler(nullptr, nullptr);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// Execute Printer Mode — Direct entry point for printer-only operation
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void GBModule::executePrinterMode()
{
    m_cancel = false;
    m_subMode = SubMode::printer;

    k_sem_reset(&g_wake);

    // Disable GBA PIO link (free pio0 resources)
    link_disable();
    gb_link_init();

    applyLedForSlot(LED_SLOT_PRINTER); // printer mode

    sendLinkStatus(LinkStatus::GBPrinterModeActive);

    printerModeLoop();

    gb_link_deinit();

    applyLedForSlot(LED_SLOT_IDLE); // connected, no active mode
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// The Game Boy link protocol (sections/gbProtocol.hpp)
// Active: the adapter clocks each data packet out. Passive: the Game Boy clocks, the adapter answers
// from the bytes the host queued and sends the Game Boy's answers back as they arrive. Hybrid: the
// Game Boy and the adapter take turns clocking, one queued byte each.
// Either way the host never has to wait for an answer before sending more, nor read one right away.
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void GBModule::normalModeLoop()
{
    k_timeout_t wait = K_NO_WAIT;
    while (!m_cancel && m_subMode == SubMode::normal)
    {
        k_sem_take(&g_wake, wait);

        HostPacket packet;
        while (!m_cancel && m_subMode == SubMode::normal
               && k_msgq_get(&g_hostPackets, &packet, K_NO_WAIT) == 0)
        {
            if (packet.command) handleCommand(packet.data, packet.length);
            else handleHostData(packet.data, packet.length);
            sendToHost();
        }

        if (!g_link.active()) gb_link_check_passive_stall(passiveStallMs);

        if (!sendToHost()) wait = hostPoll;
        else wait = g_link.active() ? K_FOREVER : passivePoll;
    }
}

void GBModule::handleHostData(const uint8_t* data, size_t count)
{
    if (count == 0) return;
    if (g_link.active())
    {
        transferActive(data, count);
        return;
    }
    g_link.queueHostData(data, count);
    if (g_link.hybrid()) resumeHybrid();
}

void GBModule::handleCommand(const uint8_t* data, size_t count)
{
    gbproto::Reply reply;
    switch (g_link.command(data, count, reply))
    {
        case gbproto::Action::switchToActive:
            gb_link_set_passive(false, 0);
            break;

        case gbproto::Action::switchToPassive:
            gb_link_set_passive(true, g_link.idleByte());
            break;

        case gbproto::Action::restartPassive:
            gb_link_restart_passive(g_link.idleByte());
            break;

        case gbproto::Action::startHybrid:
        {
            IrqGuard guard;
            gb_link_start_hybrid(g_link.hybridGapUs(), g_link.hybridGameBoyFirst(), g_link.idleByte());
            g_link.startHybrid();
            break;
        }

        case gbproto::Action::reply:
        case gbproto::Action::none:
            break;
    }

    if (reply.length) queueReply(reply.data, reply.length);
}

void GBModule::transferActive(const uint8_t* data, size_t count)
{
    // Perform SPI transfer: send data to Game Boy, receive response
    uint8_t rxBuf[MAX_TRANSFER_BYTES * 2];

    // Transfer in chunks matching bytes_per_transfer setting
    // Match old firmware: delay AFTER EVERY chunk including the last
    const uint32_t bytesPerTransfer = g_link.bytesPerTransfer();
    const uint32_t usBetweenTransfer = g_link.usBetweenTransfer();
    uint32_t totalProcessed = 0;
    while (totalProcessed < count) {
        uint32_t transferable = bytesPerTransfer;
        if (count - totalProcessed < transferable) {
            transferable = count - totalProcessed;
        }

        gb_link_transfer(data + totalProcessed, rxBuf + totalProcessed,
                       transferable, 0);
        totalProcessed += transferable;

        // Delay after EVERY chunk (matches old firmware exactly)
        if (usBetweenTransfer > 0) {
            busy_wait_us(usBetweenTransfer);
        }
    }

    queueReply(rxBuf, totalProcessed);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// Printer Mode: the Game Boy prints through the passive program, and the printer (sections/gbPrinter.hpp)
// answers in its interrupt. The host gets the forwarded packets as they arrive.
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

// False while the host has not taken earlier data yet.
static bool sendPrinterData()
{
    uint8_t bytes[MAX_TRANSFER_BYTES];
    size_t count;
    while ((count = g_printer.peekForHost(bytes, sizeof(bytes))) > 0)
    {
        if (!Transport::trySendData(std::span<const uint8_t>(bytes, count))) return false;
        g_printer.dropForHost(count);
    }
    return true;
}

void GBModule::printerModeLoop()
{
    {
        IrqGuard guard;
        g_printer.start();
    }
    gb_link_set_passive_handler(printerTransfer);
    gb_link_restart_passive(0x00);

    uint32_t lastCount = g_printer.bytesSeen();
    int64_t lastByteAt = k_uptime_get();
    k_timeout_t wait = K_NO_WAIT;
    while (!m_cancel && m_subMode == SubMode::printer)
    {
        k_sem_take(&g_wake, wait);

        const uint32_t count = g_printer.bytesSeen();
        const int64_t now = k_uptime_get();
        if (count != lastCount)
        {
            lastCount = count;
            lastByteAt = now;
        }
        else if (g_printer.midPacket() && now - lastByteAt >= printerAbortMs)
        {
            {
                IrqGuard guard;
                g_printer.abort();
            }
            gb_link_restart_passive(0x00);
            lastByteAt = now;
        }
        gb_link_check_passive_stall(printerStallMs);

        wait = sendPrinterData() ? passivePoll : hostPoll;
    }
    sendPrinterData();
}
