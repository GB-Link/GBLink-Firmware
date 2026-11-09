#include "tradeDisconnected.hpp"

extern "C"
{
    #include "../payloads/linkPlayer.h"
}

#include "../layers/packet/packetCommands/commands.hpp"


void TradeDisconnect::exchangeTrainerData()
{
    connectAsMaster();
    m_packetLayer.sendCommand(sendLinkTypeCommand(LINKTYPE_TRADE_DISCONNECTED));

    while(!m_cancel)
    {
        PacketLayer::TransiveResult result = m_packetLayer.awaitTransiveResults();
        std::span<const uint16_t> command = result.received;

        if ((command[0] == LINKCMD_INIT_BLOCK) && (m_blockState == BlockCommandState::None))
        {
            
            const struct LinkPlayerBlock* linkPlayerBlock = linkPLayer(LINKTYPE_TRADE_DISCONNECTED);
            m_packetLayer.sendCommand(blockCommand(linkPlayerBlock, sizeof(*linkPlayerBlock), sizeof(*linkPlayerBlock)));
            m_blockState = BlockCommandState::LinkPlayer;        
        }

        if (m_blockState == BlockCommandState::LinkPlayer && m_packetLayer.idle())
        {
            return;
        }

        k_sleep(K_MSEC(5));
    }
}

NextSection TradeDisconnect::handleDisconnect()
{
    while(!m_cancel)
    {
        PacketLayer::TransiveResult result = m_packetLayer.awaitTransiveResults();
        std::span<const uint16_t> command = result.received;

        if (m_blockState == BlockCommandState::FinishTrade)
        {
            std::array<uint16_t, 2> command = {LINKCMD_CONFIRM_FINISH_TRADE};
            m_packetLayer.sendCommand(blockCommand(command.data(), command.size(), 20));

            m_blockState = BlockCommandState::None;

            k_sleep(K_MSEC(5));
            continue;
        }

        switch (command[0])
        {
            case LINKCMD_READY_EXIT_STANDBY:
                m_packetLayer.sendCommand(readyExitStandbyCommand());
                break;
            
            case LINKCMD_CONT_BLOCK:
            {
                if (command[1] == LINKCMD_READY_FINISH_TRADE)
                {
                    std::array<uint16_t, 2> command = {LINKCMD_READY_FINISH_TRADE};
                    m_packetLayer.sendCommand(blockCommand(command.data(), command.size(), 20));
                    m_blockState = BlockCommandState::FinishTrade;
                    
                }
                break;
            }

            case LINKCMD_READY_CLOSE_LINK:
            {
                m_packetLayer.sendCommand(readyCloseLinkCommand());
                k_sleep(K_MSEC(400));
                return NextSection::connection;
            }
        }

        k_sleep(K_MSEC(5));
    }
    return NextSection::cancel; // user canceled from web interface
}


NextSection TradeDisconnect::process()
{
    exchangeTrainerData();
    return handleDisconnect();   
}