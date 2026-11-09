#include "tradeSetup.hpp"

extern "C"
{
    #include "../payloads/trainerCard.h"
    #include "../payloads/linkPlayer.h"
}

#include "../layers/packet/packetCommands/commands.hpp"

#include <zephyr/drivers/gpio.h>

NextSection TradeSetup::process()
{
    #ifdef CONFIG_SECTIONS_USE_MASTER_MODE
    connectAsMaster();
    #else
    connectAsSlave();
    #endif

    #ifdef CONFIG_SECTIONS_USE_MASTER_MODE
    m_packetLayer.sendCommand(sendLinkTypeCommand(m_linkType));
    #endif
    NextSection nextSection = NextSection::connection;

    while (!m_cancel)
    {
        PacketLayer::TransiveResult result = m_packetLayer.awaitTransiveResults();
        std::span<const uint16_t> command = result.received;
        
        #ifdef CONFIG_SECTIONS_USE_MASTER_MODE
        if (m_blockState == BlockCommandState::RequestTrainerCard && m_packetLayer.idle())
        {
            m_packetLayer.sendCommand(sendBlockCommandRequestCommand(2));
            m_blockState = BlockCommandState::TrainerCard;
            continue;
        }
        #endif
        
        switch(command[0])
        {

            case LINKCMD_INIT_BLOCK:
            {
                switch(m_blockState)
                {
                    case BlockCommandState::LinkPlayer:
                    {
                        const struct LinkPlayerBlock* linkPlayerBlock = linkPLayer(m_linkType);

                        #ifdef CONFIG_SECTIONS_USE_MASTER_MODE
                        m_blockState = BlockCommandState::RequestTrainerCard;
                        #else
                        m_blockState = BlockCommandState::TrainerCard;
                        #endif

                        m_packetLayer.sendCommand(blockCommand(linkPlayerBlock, sizeof(*linkPlayerBlock), sizeof(*linkPlayerBlock)));
                        break;
                    }
                    
                    case BlockCommandState::TrainerCard:
                    {
                        const struct TrainerCard* trainerCard = trainerCardPlaceholder();
                        m_packetLayer.sendCommand(blockCommand(trainerCard, sizeof(*trainerCard), 0x64));
                        break;
                    }
                    default: continue;
                } 
                break;
            }
            
            case LINKCMD_READY_EXIT_STANDBY:
                m_packetLayer.sendCommand(readyExitStandbyCommand());
                break;
            
            case LINKCMD_SEND_HELD_KEYS:
            {
                if (command[1] == LINK_KEY_CODE_EXIT_ROOM)
                {
                    m_packetLayer.sendCommand(moveCommand(LINK_KEY_CODE_EXIT_ROOM));
                    nextSection = NextSection::exit;
                    break;
                }

                if (!m_packetLayer.idle()) break;

                if (m_movementDataIndex >= m_movementData.size()) break;
                
                m_packetLayer.sendCommand(moveCommand(m_movementData[m_movementDataIndex]));
                m_movementDataIndex++;
                break;
            }
            
            case LINKCMD_READY_CLOSE_LINK:
                m_packetLayer.sendCommand(readyCloseLinkCommand());
                k_sleep(K_MSEC(300));
                return nextSection;
            
            default: break;
        }
        k_sleep(K_MSEC(5));
    }
    return NextSection::cancel; // user canceled from web interface
}