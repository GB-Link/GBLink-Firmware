
#include "usbSection.hpp"

#include "../linkStatus.hpp"

void UsbSection::establishConncection()
{
    while (m_packetLayer.getReceivedHandshake() != LINK_SLAVE_HANDSHAKE) 
    {
        if (m_cancel) return; 
    }

    sendLinkStatus(LinkStatus::HandshakeReceived);

    while (!m_packetLayer.isHandshakeEnabled())
    { 
        k_sleep(K_MSEC(50));
        if (m_cancel) return; 
    }

    if (m_playerSeat != 0)
    {
        while (m_packetLayer.getTransmittedHandshake()[0] != LINK_MASTER_HANDSHAKE) 
        { 
            if (m_cancel) return; 
        }
    }
    else
    {
        while (m_packetLayer.getReceivedHandshake() != LINK_MASTER_HANDSHAKE) 
        { 
            if (m_cancel) return; 
        }
    }

    sendLinkStatus(LinkStatus::LinkConnected);
}

bool UsbSection::process()
{
    bool keepAlive = true;
    bool partnerReadyCloseLink[3] = {false, false, false};
    bool readyCloseLink = false;
    establishConncection();

    while(!m_cancel)
    {
        PacketLayer::TransiveResult result = m_packetLayer.awaitTransiveResults();

        bufferReceivedPackets(result.received);

        if ((result.received[0] == LINKCMD_SEND_HELD_KEYS) && (result.received[1] == LINK_KEY_CODE_EXIT_ROOM))
        {
            keepAlive = false;
        } 

        if (result.transmitted[0][0] == LINKCMD_SEND_HELD_KEYS && result.transmitted[1][0] == LINK_KEY_CODE_EXIT_ROOM)
        {
            keepAlive = false;
        }

        if (result.received[0] == LINKCMD_READY_CLOSE_LINK)
        {
            readyCloseLink = true;
        }

        for (int i = 0; i < m_playerCount - 1; i++)
        {
            if (result.transmitted[0][i] == LINKCMD_READY_CLOSE_LINK)
            {
                partnerReadyCloseLink[i] = true;
            }
        }
            
        // only the partners actually in the session have to be ready
        const auto partners = std::span(partnerReadyCloseLink).first(m_playerCount - 1);
        const bool closeLink = readyCloseLink && std::ranges::all_of(partners, [](bool ready) { return ready; });

        if (closeLink) break;
    }

    flush();

    return keepAlive;
}