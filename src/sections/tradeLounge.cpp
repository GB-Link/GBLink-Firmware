#include "tradeLounge.hpp"
extern "C"
{
    #include "../payloads/trainerCard.h"
    #include "../payloads/linkPlayer.h"
}

#include "../layers/packet/packetCommands/commands.hpp"

NextSection TradeLounge::process()
{
    connectAsMaster();
    m_packetLayer.sendCommand(sendLinkTypeCommand(LINKTYPE_TRADE));
    NextSection nextSection = NextSection::connection;

    while (!m_cancel)
    {
        PacketLayer::TransiveResult result = m_packetLayer.awaitTransiveResults();
        std::span<const uint16_t> command = result.received;

        switch(command[0])
        {
            case LINKCMD_INIT_BLOCK:
            {
                const struct LinkPlayerBlock* linkPlayerBlock = linkPLayer(LINKTYPE_TRADE);
                m_packetLayer.sendCommand(blockCommand(linkPlayerBlock, sizeof(*linkPlayerBlock), sizeof(*linkPlayerBlock)));
                break;
            }
            
            case LINKCMD_SEND_HELD_KEYS:
            {
                if (command[1] == LINK_KEY_CODE_EXIT_ROOM)
                {
                    m_packetLayer.sendCommand(moveCommand(LINK_KEY_CODE_EXIT_ROOM));
                    nextSection = NextSection::exit;
                } 
                if (command[1] == LINK_KEY_CODE_READY)
                {   
                    m_packetLayer.sendCommand(moveCommand(LINK_KEY_CODE_READY));
                    nextSection = NextSection::connection;
                }
                break;
            }
            
            case LINKCMD_READY_CLOSE_LINK:
            {
                m_packetLayer.sendCommand(readyCloseLinkCommand());
                k_sleep(K_MSEC(200));
                return nextSection;
            }
            
            default: break;
        }

        k_sleep(K_MSEC(5));
    }
    return NextSection::cancel; // user canceled from web interface
}