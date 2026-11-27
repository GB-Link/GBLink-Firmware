#include <cstdint>
#include <cstddef>

#pragma once

#include "gbProtocol.hpp"

// Game Boy Printer emulation. The Game Boy clocks each packet in:
//
//     0x88 0x33 | command | compression | length (2 bytes LE) | data | checksum (2 bytes LE) | 2 bytes
//
// The printer answers 0x00 to every byte except the last two: 0x81 (a printer
// is connected), then its status (0x00: ready, no errors).
//
// For the host it forwards each packet's command, compression, length and
// data, and adds markers: 0xFF when it starts, 0xFE after a PRINT packet, and
// "ABORTPRINT" when the Game Boy stopped halfway through a packet.
//
// Zephyr-free so it can be tested on the host. onByte() runs in the PIO
// interrupt; start() and abort() must keep that interrupt out.

namespace gbprinter
{

constexpr uint8_t packetStart1 = 0x88;
constexpr uint8_t packetStart2 = 0x33;
constexpr uint8_t commandPrint = 0x02;

constexpr uint8_t deviceId = 0x81;
constexpr uint8_t statusReady = 0x00;

constexpr uint8_t startMarker = 0xFF;
constexpr uint8_t printMarker = 0xFE;
constexpr uint8_t abortMarker[] = { 'A', 'B', 'O', 'R', 'T', 'P', 'R', 'I', 'N', 'T' };

// A whole picture (nine bands of 640 bytes) with its packet headers.
constexpr size_t hostBufferBytes = 0x4000;

class Printer
{
public:
    // Waiting for a packet; the host gets the start marker.
    void start()
    {
        m_toHost.clear();
        m_state = State::start1;
        forward(startMarker);
    }

    // Drops the packet in progress and tells the host.
    void abort()
    {
        m_state = State::start1;
        m_toHost.push(abortMarker, sizeof(abortMarker));
    }

    bool midPacket() const { return m_state != State::start1 && m_state != State::start2; }

    // Counts the Game Boy's bytes, so the host side can tell when it stopped.
    uint32_t bytesSeen() const { return m_bytesSeen; }

    // One transfer finished: `received` is the Game Boy's byte. Returns the
    // printer's answer for the next transfer.
    uint8_t onByte(uint8_t received)
    {
        m_bytesSeen = m_bytesSeen + 1;

        switch (m_state)
        {
            case State::start1:
                if (received == packetStart1) m_state = State::start2;
                break;

            case State::start2:
                if (received == packetStart2) m_state = State::command;
                else if (received != packetStart1) m_state = State::start1;
                break;

            case State::command:
                m_command = received;
                forward(received);
                m_state = State::compression;
                break;

            case State::compression:
                forward(received);
                m_state = State::lengthLow;
                break;

            case State::lengthLow:
                m_left = received;
                m_state = State::lengthHigh;
                break;

            case State::lengthHigh:
                m_left = static_cast<uint16_t>(m_left | (received << 8));
                forward(static_cast<uint8_t>(m_left));
                forward(static_cast<uint8_t>(m_left >> 8));
                m_state = (m_left > 0) ? State::data : State::checksumLow;
                break;

            case State::data:
                forward(received);
                m_left = static_cast<uint16_t>(m_left - 1);
                if (m_left == 0) m_state = State::checksumLow;
                break;

            case State::checksumLow:
                m_state = State::checksumHigh;
                break;

            case State::checksumHigh:
                m_state = State::idByte;
                return deviceId;

            case State::idByte:
                m_state = State::statusByte;
                return statusReady;

            case State::statusByte:
                if (m_command == commandPrint) forward(printMarker);
                m_state = State::start1;
                break;
        }
        return 0x00;
    }

    size_t peekForHost(uint8_t* bytes, size_t max) const { return m_toHost.peek(bytes, max); }
    void dropForHost(size_t count) { m_toHost.drop(count); }
    size_t bytesForHost() const { return m_toHost.size(); }

private:
    enum class State : uint8_t
    {
        start1, start2, command, compression, lengthLow, lengthHigh, data,
        checksumLow, checksumHigh, idByte, statusByte,
    };

    // What does not fit is dropped.
    void forward(uint8_t byte) { m_toHost.push(&byte, 1); }

    volatile State m_state = State::start1;
    volatile uint32_t m_bytesSeen = 0;
    uint8_t m_command = 0;
    uint16_t m_left = 0;
    gbproto::ByteRing<hostBufferBytes> m_toHost;
};

}
