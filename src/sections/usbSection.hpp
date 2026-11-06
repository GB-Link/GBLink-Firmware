
#include "../layers/packet/packetLayer.hpp"
#include "../layers/transport.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>

#pragma once

class UsbSection
{
public:
    UsbSection(uint8_t playerSeat, uint8_t playerCount) : m_playerSeat(playerSeat), m_playerCount(playerCount)
    {
        m_packetLayer.sendCommand(usbLinkCommand(playerSeat, playerCount));
        m_packetLayer.configurePlayer(playerSeat, playerCount);
    }

    bool process();

    void startHandshake() { m_packetLayer.enableHandshake(); }

    void cancel() 
    {
        m_cancel = true;
        m_packetLayer.cancel(); 
    }

    void connectLink() 
    { 
        if (m_playerSeat != 0) m_packetLayer.connectHandshake();
    }

private:

    void bufferReceivedPackets(const std::span<const uint16_t>& recievedPacket)
    {
        auto packetsBuffer = std::span(m_packets).subspan(
            m_numberOfWaitingPackets * packetSize,
            packetSize
        );

        const auto bytes = std::as_bytes(recievedPacket);
        std::memcpy(packetsBuffer.data() + 1, bytes.data(), bytes.size());

        packetsBuffer[0] = m_playerSeat;
        
        // skip directional input of case "no button was pressed" as
        // this will just strain the usb/network connection
        
        if ((recievedPacket[0] == 0xCAFE && recievedPacket[1] == 0x11))
        {
            m_noDirectionCommandStreak++;
            if (m_noDirectionCommandStreak > 1)
            {
                packetsBuffer[1] = 0x00;
                packetsBuffer[2] = 0x00;
                packetsBuffer[3] = 0x00;
                packetsBuffer[4] = 0x00;
            }
        }
        else
        {
            m_noDirectionCommandStreak = 0;
        }
        

        m_numberOfWaitingPackets++;

        if (m_numberOfWaitingPackets == maxBufferedPackets)
        {
            flush();
        }
    }

    bool flush()
    {
        // case for end of session
        if (m_numberOfWaitingPackets == 0) return true;
        
        // skip empty commands, only the payload counts since the seat byte is always set
        bool emptyCommands = true;
        for (size_t i = 0; i < m_numberOfWaitingPackets; i++)
        {
            const auto payload = std::span(m_packets).subspan(i * packetSize + 1, packetSize - 1);
            if (!std::ranges::all_of(payload, [](uint8_t byte) { return byte == 0x00; }))
            {
                emptyCommands = false;
                break;
            }
        }

        if (!emptyCommands)
        {
            Transport::sendData(std::span(m_packets));
        }

        m_packets.fill(0x00);
        m_numberOfWaitingPackets = 0;

        return true;
    }

    static constexpr uint8_t maxBufferedPackets = 3;
    static constexpr uint8_t packetSize = 17;

    bool m_cancel = false;
    void establishConncection();
    
    std::array<uint8_t, packetSize * maxBufferedPackets> m_packets = {};
    size_t m_numberOfWaitingPackets = 0;
    size_t m_noDirectionCommandStreak = 0;

    PacketLayer m_packetLayer;
    uint8_t m_playerSeat;
    uint8_t m_playerCount;
};