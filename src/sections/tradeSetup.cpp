#include "tradeSetup.hpp"

extern "C"
{
    #include "../payloads/trainerCard.h"
    #include "../payloads/linkPlayer.h"
}

#include "../callbacks/commands.hpp"

#include <zephyr/drivers/gpio.h>

using namespace legacy;

NextSection TradeSetup::process()
{
    #ifdef CONFIG_SECTIONS_USE_MASTER_MODE
    connectAsMaster();
    #else
    connectAsSlave();
    #endif

    #ifdef CONFIG_SECTIONS_USE_MASTER_MODE
    m_packetLayer.setTransiveHandler(sendLinkTypeCommand(m_linkType));
    #endif
    NextSection nextSection = NextSection::connection;

    while (!m_cancel)
    {
        auto result = m_packetLayer.awaitTransiveResults();
        std::span<const uint16_t> command = result.received;
        
        #ifdef CONFIG_SECTIONS_USE_MASTER_MODE
        if (m_blockState == BlockCommandState::RequestTrainerCard && m_packetLayer.idle())
        {
            m_packetLayer.setTransiveHandler(sendBlockCommandRequestCommand(2));
            m_blockState = BlockCommandState::TrainerCard;
            continue;
        }
        #endif
        
        switch(command[0])
        {

            case LINKCMD_INIT_BLOCK:
            {
                auto transive = blockCommand();

                switch(m_blockState)
                {
                    case BlockCommandState::LinkPlayer:
                    {
                        const struct LinkPlayerBlock* linkPlayerBlock = linkPLayer(m_linkType);
                        blockCommandSetup(linkPlayerBlock, sizeof(*linkPlayerBlock), sizeof(*linkPlayerBlock));

                        #ifdef CONFIG_SECTIONS_USE_MASTER_MODE
                        m_blockState = BlockCommandState::RequestTrainerCard;
                        #else
                        m_blockState = BlockCommandState::TrainerCard;
                        #endif
                        break;
                    }
                    
                    case BlockCommandState::TrainerCard:
                    {
                        const struct TrainerCard* trainerCard = trainerCardPlaceholder();
                        blockCommandSetup(trainerCard, sizeof(*trainerCard), 0x64);
                        break;
                    }
                    default: continue;
                } 
                m_packetLayer.setTransiveHandler(transive);
                break;
            }
            
            case LINKCMD_READY_EXIT_STANDBY:
                m_packetLayer.setTransiveHandler(readyExitStandbyCommand());
                break;
            
            case LINKCMD_SEND_HELD_KEYS:
            {
                if (command[1] == LINK_KEY_CODE_EXIT_ROOM)
                {
                    moveCommandInit(LINK_KEY_CODE_EXIT_ROOM);
                    m_packetLayer.setTransiveHandler(moveCommand());
                    nextSection = NextSection::exit;
                    break;
                }

                if (!m_packetLayer.idle()) break;

                if (m_movementDataIndex >= m_movementData.size()) break;
                
                moveCommandInit(m_movementData[m_movementDataIndex]);
                m_packetLayer.setTransiveHandler(moveCommand());
                m_movementDataIndex++;
                break;
            }
            
            case LINKCMD_READY_CLOSE_LINK:
                m_packetLayer.setTransiveHandler(readyCloseLinkCommand());
                k_sleep(K_MSEC(300));
                return nextSection;
            
            default: break;
        }
        k_sleep(K_MSEC(5));
    }
    return NextSection::cancel; // user canceled from web interface
}