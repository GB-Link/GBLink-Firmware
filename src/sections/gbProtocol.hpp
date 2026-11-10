#include <cstdint>
#include <cstring>
#include <cstddef>

#pragma once

// Game Boy link protocol of the GB Link mode.
//
// Active (the default): the adapter clocks the Game Boy. Each data packet from
// the host is clocked out, and the bytes received in return go back as one
// packet.
//
// Passive: the Game Boy clocks the adapter. Each transfer is answered with the
// next byte the host queued, or with the idle byte while nothing is queued. The
// Game Boy's answers to queued bytes go back to the host as they arrive, so the
// host never has to answer within a transfer.
//
// Sections (passive): the host loads a whole block first. The adapter answers
// the waiting byte until the Game Boy sends the preamble byte, answers the
// preamble while the Game Boy repeats it, then exchanges the block. The host
// polls for the end and reads the Game Boy's block back in pieces. Pokémon
// Gold, Silver and Crystal exchange their blocks this way.
//
// Hybrid: the Game Boy and the adapter take turns clocking, one byte each, for
// games where both sides drive the clock. The queued bytes are used in order,
// whoever clocks: the adapter clocks the next one on its turn, and waits while
// there is none, and answers the Game Boy's turn with the next one, or with the
// idle byte. The Game Boy's bytes go back as in passive mode.
//
// Data packets are link data. The host sets the mode and the rest with
// commands (Command below), which the module takes in turn with the data
// packets, so each one acts between the packets around it.
//
// Zephyr-free so it can be tested on the host. Concurrency contract (see
// module/gb.cpp): onPassiveTransfer() and onHybridTransfer() run in the PIO
// interrupt; they are the only writers of the buffer towards the host. The
// buffer towards the Game Boy is read there and, inside a Guard, by
// resumeHybrid(). Everything else runs in the module thread, and changes to
// state the interrupt also uses happen inside a Guard, which must keep the
// interrupt out (irq_lock).

namespace gbproto
{

// Largest chunk clocked out at once in active mode, and largest reply.
constexpr size_t maxTransferBytes = 0x40;

constexpr uint32_t defaultUsBetweenTransfer = 1000;
constexpr uint8_t defaultBytesPerTransfer = 1;

constexpr size_t bufferBytes = 0x400;

// The GB Link mode's commands (0x31 and 0x32, the printer, are the module's).
// A command shorter than its payload is ignored. The queries answer with the
// command byte first.
enum class Command : uint8_t
{
    setTiming = 0x30,           // µs between transfers (3 bytes), bytes per transfer
    setActiveMode = 0x33,
    setPassiveMode = 0x34,
    setHybridMode = 0x35,       // u32 µs the adapter waits before clocking its turn; 1: the Game Boy clocks first
    setIdleByte = 0x36,         // the byte answered while nothing is queued
    clearQueues = 0x37,
    restart = 0x38,             // passive or hybrid starts over, after a transfer went out of step
    loadSection = 0x39,         // preamble byte, the byte answered until the preamble
    getSectionLoaded = 0x3A,    // -> u32 checksum, u32 length of the block loaded so far
    startSection = 0x3B,
    getSectionStatus = 0x3C,    // -> done, u32 bytes of the Game Boy's block, u32 bytes left to send
    readSection = 0x3D,         // how many bytes of the Game Boy's block -> how many, the bytes
    endSection = 0x3E,          // back to regular passive mode
};

// What the module does after command().
enum class Action : uint8_t
{
    none,
    reply,              // send the reply
    switchToActive,     // stop the passive program, start the active one
    switchToPassive,    // stop the active program, start the passive one
    restartPassive,     // restart the passive program
    startHybrid,        // inside one Guard: start the hybrid programs, Link::startHybrid()
};

enum class Mode : uint8_t
{
    active,             // the adapter clocks
    passive,            // the Game Boy clocks
    hybrid,             // they take turns
};

struct Reply
{
    uint8_t data[maxTransferBytes];
    size_t length = 0;
};

inline uint32_t readU32(const uint8_t* bytes)
{
    return static_cast<uint32_t>(bytes[0])
        | (static_cast<uint32_t>(bytes[1]) << 8)
        | (static_cast<uint32_t>(bytes[2]) << 16)
        | (static_cast<uint32_t>(bytes[3]) << 24);
}

inline void writeU32(uint8_t* bytes, uint32_t value)
{
    bytes[0] = static_cast<uint8_t>(value);
    bytes[1] = static_cast<uint8_t>(value >> 8);
    bytes[2] = static_cast<uint8_t>(value >> 16);
    bytes[3] = static_cast<uint8_t>(value >> 24);
}

// One writer and one reader; one byte stays free to tell full from empty.
template <size_t Size = bufferBytes>
class ByteRing
{
public:
    size_t size() const { return (m_head + Size - m_tail) % Size; }

    size_t push(const uint8_t* bytes, size_t count)
    {
        size_t pushed = 0;
        for (; pushed < count; pushed++)
        {
            const uint32_t next = (m_head + 1) % Size;
            if (next == m_tail) break;
            m_data[m_head] = bytes[pushed];
            m_head = next;
        }
        return pushed;
    }

    bool pop(uint8_t& value)
    {
        if (m_tail == m_head) return false;
        value = m_data[m_tail];
        m_tail = (m_tail + 1) % Size;
        return true;
    }

    size_t peek(uint8_t* bytes, size_t max) const
    {
        size_t count = 0;
        for (uint32_t i = m_tail; i != m_head && count < max; i = (i + 1) % Size)
            bytes[count++] = m_data[i];
        return count;
    }

    void drop(size_t count)
    {
        if (count > size()) count = size();
        m_tail = (m_tail + count) % Size;
    }

    // Sum of the bytes, each weighted by its place in a little-endian u32.
    uint32_t checksum() const
    {
        uint32_t sum = 0;
        unsigned shift = 0;
        for (uint32_t i = m_tail; i != m_head; i = (i + 1) % Size)
        {
            sum += static_cast<uint32_t>(m_data[i]) << shift;
            shift = (shift + 8) % 32;
        }
        return sum;
    }

    void clear() { m_tail = m_head; }

private:
    volatile uint8_t m_data[Size] = {};
    volatile uint32_t m_head = 0;
    volatile uint32_t m_tail = 0;
};

template <class Guard>
class Link
{
public:
    // The GB Link mode starts: active, default timing, nothing queued.
    void reset()
    {
        Guard guard;
        m_mode = Mode::active;
        m_usBetweenTransfer = defaultUsBetweenTransfer;
        m_bytesPerTransfer = defaultBytesPerTransfer;
        m_idle = 0;
        m_hybridGapUs = 0;
        m_hybridGameBoyFirst = false;
        m_turnWaiting = false;
        clearPassive();
    }

    // Another sub-mode had the pins: active again, the timing stays.
    void returnToActive()
    {
        Guard guard;
        m_mode = Mode::active;
        m_turnWaiting = false;
        clearPassive();
    }

    bool active() const { return m_mode == Mode::active; }
    bool hybrid() const { return m_mode == Mode::hybrid; }
    uint32_t usBetweenTransfer() const { return m_usBetweenTransfer; }
    uint8_t bytesPerTransfer() const { return m_bytesPerTransfer; }

    // Hybrid: how long the adapter waits after the Game Boy's byte before it
    // clocks its own, and who clocks the first byte.
    uint32_t hybridGapUs() const { return m_hybridGapUs; }
    bool hybridGameBoyFirst() const { return m_hybridGameBoyFirst; }

    void setTiming(uint32_t usBetweenTransfer, uint8_t bytesPerTransfer)
    {
        if (bytesPerTransfer == 0) bytesPerTransfer = 1;
        if (bytesPerTransfer > maxTransferBytes) bytesPerTransfer = maxTransferBytes;
        m_usBetweenTransfer = usBetweenTransfer;
        m_bytesPerTransfer = bytesPerTransfer;
    }

    // The byte answered while nothing is queued, also the first one after the
    // passive program (re)starts.
    uint8_t idleByte() const { return m_idle; }

    // bytes: the command byte, then its payload.
    Action command(const uint8_t* bytes, size_t count, Reply& reply)
    {
        reply.length = 0;
        if (count == 0) return Action::none;
        const uint8_t id = bytes[0];
        const uint8_t* payload = bytes + 1;
        const size_t payloadLength = count - 1;

        switch (static_cast<Command>(id))
        {
            case Command::setTiming:
                if (payloadLength < 4) break;
                setTiming(static_cast<uint32_t>(payload[0])
                              | (static_cast<uint32_t>(payload[1]) << 8)
                              | (static_cast<uint32_t>(payload[2]) << 16),
                          payload[3]);
                break;

            case Command::setActiveMode:
                if (m_mode == Mode::active) break;
                {
                    Guard guard;
                    m_mode = Mode::active;
                    m_turnWaiting = false;
                }
                return Action::switchToActive;

            case Command::setPassiveMode:
                if (m_mode == Mode::passive) break;
                {
                    Guard guard;
                    clearPassive();
                    m_mode = Mode::passive;
                    m_turnWaiting = false;
                }
                return Action::switchToPassive;

            case Command::setHybridMode:
                if (payloadLength < sizeof(uint32_t) + 1) break;
                m_hybridGapUs = readU32(payload);
                m_hybridGameBoyFirst = payload[sizeof(uint32_t)] != 0;
                return Action::startHybrid;

            case Command::setIdleByte:
                if (payloadLength < 1) break;
                m_idle = payload[0];
                break;

            case Command::clearQueues:
            {
                Guard guard;
                m_toHost.clear();
                m_toGameBoy.clear();
                break;
            }

            case Command::restart:
                if (m_mode == Mode::active) break;
                if (m_mode == Mode::hybrid) return Action::startHybrid;
                {
                    Guard guard;
                    clearPassive();
                }
                return Action::restartPassive;

            case Command::loadSection:
            {
                if (payloadLength < 2) break;
                Guard guard;
                clearPassive();
                m_sectionSetup = true;
                m_sectionRequested = true;
                m_preamble = payload[0];
                m_waiting = payload[1];
                break;
            }

            case Command::getSectionLoaded:
                reply.data[0] = id;
                writeU32(reply.data + 1, m_toGameBoy.checksum());
                writeU32(reply.data + 1 + sizeof(uint32_t), static_cast<uint32_t>(m_toGameBoy.size()));
                reply.length = 1 + 2 * sizeof(uint32_t);
                return Action::reply;

            case Command::startSection:
                m_sectionSetup = false;
                break;

            case Command::getSectionStatus:
            {
                const size_t toGameBoy = m_toGameBoy.size();
                reply.data[0] = id;
                reply.data[1] = (toGameBoy == 0 && !m_scheduled) ? 1 : 0;
                writeU32(reply.data + 2, static_cast<uint32_t>(m_toHost.size()));
                writeU32(reply.data + 2 + sizeof(uint32_t), static_cast<uint32_t>(toGameBoy));
                reply.length = 2 + 2 * sizeof(uint32_t);
                return Action::reply;
            }

            case Command::readSection:
            {
                if (payloadLength < 1) break;
                size_t length = 0;
                if (m_sectionRequested)
                {
                    length = payload[0];
                    if (length > maxTransferBytes - 2) length = maxTransferBytes - 2;
                    length = m_toHost.peek(reply.data + 2, length);
                    m_toHost.drop(length);
                }
                reply.data[0] = id;
                reply.data[1] = static_cast<uint8_t>(length);
                reply.length = 2 + length;
                return Action::reply;
            }

            case Command::endSection:
            {
                Guard guard;
                clearPassive();
                break;
            }
        }
        return Action::none;
    }

    // Passive and hybrid: bytes for the Game Boy. What does not fit is dropped;
    // the section checksum and length show it.
    void queueHostData(const uint8_t* data, size_t count) { m_toGameBoy.push(data, count); }

    // Regular passive mode and hybrid: the Game Boy's answers for the host.
    // Sections are read with readSection instead.
    size_t peekStream(uint8_t* bytes, size_t max) const
    {
        if (m_mode == Mode::active || m_sectionRequested) return 0;
        return m_toHost.peek(bytes, max);
    }

    void dropStream(size_t count) { m_toHost.drop(count); }

    size_t bytesForHost() const { return m_toHost.size(); }

    // One transfer finished: `received` is the Game Boy's byte. Returns the
    // byte for the next transfer.
    uint8_t onPassiveTransfer(uint8_t received)
    {
        if (m_scheduled) m_toHost.push(&received, 1);
        m_scheduled = false;

        uint8_t next = m_idle;

        if (m_sectionSetup)
        {
            // The host is still loading the block.
        }
        else if (m_sectionRequested)
        {
            if (!m_preambleStarted && received == m_preamble)
                m_preambleStarted = true;
            if (!m_pastPreamble && m_preambleStarted && received != m_preamble)
            {
                // The Game Boy's first byte after the preamble starts its block.
                m_pastPreamble = true;
                m_toHost.push(&received, 1);
            }

            if (!m_preambleStarted)
                next = m_waiting;
            else if (!m_pastPreamble)
                next = m_preamble;
            else if (m_toGameBoy.pop(next))
            {
                // The Game Boy's answer to the block's last byte is not part of its block.
                if (m_toGameBoy.size() > 0) m_scheduled = true;
            }
        }
        else if (m_toGameBoy.pop(next))
        {
            m_scheduled = true;
        }

        return next;
    }

    // Hybrid: one transfer finished; `clocked` when the adapter clocked it.
    // Returns the byte for the next transfer, the other side's turn, or -1
    // when it is the adapter's turn and nothing is queued (resumeHybrid()
    // gives it once something is).
    int onHybridTransfer(uint8_t received, bool clocked)
    {
        // A byte that ends after the host left hybrid mode changes nothing.
        if (m_mode != Mode::hybrid) return clocked ? m_idle : -1;

        // A byte the adapter clocks always comes from the host.
        if (clocked || m_scheduled) m_toHost.push(&received, 1);
        m_scheduled = false;

        uint8_t next;
        if (clocked)
        {
            if (!m_toGameBoy.pop(next)) return m_idle;
            m_scheduled = true;
            return next;
        }

        if (m_toGameBoy.pop(next)) return next;
        m_turnWaiting = true;
        return -1;
    }

    // Hybrid: after host data was queued, the byte to clock now if the
    // adapter's turn was waiting for one, else -1.
    int resumeHybrid()
    {
        Guard guard;
        uint8_t next;
        if (!m_turnWaiting || !m_toGameBoy.pop(next)) return -1;
        m_turnWaiting = false;
        return next;
    }

    // Hybrid mode starts over: nothing queued, and the first turn is the
    // adapter's, waiting for the host, or the Game Boy's, answered with the
    // idle byte. The module calls it with the start of the hybrid programs
    // inside one Guard, so no byte of the program it stops gets in.
    void startHybrid()
    {
        Guard guard;
        clearPassive();
        m_mode = Mode::hybrid;
        m_turnWaiting = !m_hybridGameBoyFirst;
    }

private:
    void clearPassive()
    {
        m_scheduled = false;
        m_toHost.clear();
        m_toGameBoy.clear();
        m_sectionSetup = false;
        m_sectionRequested = false;
        m_preambleStarted = false;
        m_pastPreamble = false;
    }

    volatile Mode m_mode = Mode::active;
    volatile uint32_t m_usBetweenTransfer = defaultUsBetweenTransfer;
    volatile uint8_t m_bytesPerTransfer = defaultBytesPerTransfer;

    volatile uint8_t m_idle = 0;
    volatile bool m_scheduled = false;      // the next transfer answers with host data

    volatile uint32_t m_hybridGapUs = 0;
    volatile bool m_hybridGameBoyFirst = false;
    volatile bool m_turnWaiting = false;    // the adapter's turn, with nothing to clock yet

    volatile uint8_t m_preamble = 0;
    volatile uint8_t m_waiting = 0;
    volatile bool m_sectionSetup = false;
    volatile bool m_sectionRequested = false;
    volatile bool m_preambleStarted = false;
    volatile bool m_pastPreamble = false;

    ByteRing<> m_toGameBoy;
    ByteRing<> m_toHost;
};

}
