#include "tradeConnection.hpp"

#include "../payloads/pokemon.hpp"
#include <bit>
#include <cstdint>
#include <sys/types.h>
extern "C"
{
    #include "../payloads/linkPlayer.h"
}
#include "../callbacks/commands.hpp"

using namespace legacy;

void TradeConnection::handleInitialDataExchange()
{
    connectAsMaster();
    m_packetLayer.setTransiveHandler(sendLinkTypeCommand(LINKTYPE_TRADE_CONNECTING));

    while (!m_cancel)
    {
        auto result = m_packetLayer.awaitTransiveResults();
        std::span<const uint16_t> command = result.received;

        if (m_requestBlock)
        {
            m_requestBlock = false;
            m_packetLayer.setTransiveHandler(sendBlockCommandRequestCommand(m_requestBlockSize));

            k_sleep(K_MSEC(5));
            continue;
        }


        switch(command[0])
        {
            case LINKCMD_INIT_BLOCK:
            {
                auto transive = blockCommand();

                switch(m_blockState)
                {
                    case TradeConnectionState::LinkPlayer:
                    {
                        const struct LinkPlayerBlock* linkPlayerBlock = linkPLayer(LINKTYPE_TRADE_CONNECTING);
                        blockCommandSetup(linkPlayerBlock, sizeof(*linkPlayerBlock), sizeof(*linkPlayerBlock));
                        m_blockState = TradeConnectionState::PartyPart0;
                        m_requestBlockSize = 1;
                        party::partnerPartyInit();
                        k_timer_start(&m_commandRequestTimer, K_MSEC(2000), K_NO_WAIT);
                        break;
                    }

                    case TradeConnectionState::PartyPart0:
                    {
                        const auto party = std::as_bytes(party::getParty().subspan<0, 200>());
                        blockCommandSetup(party.data(), party.size(), 200);
                        m_blockState = TradeConnectionState::PartyPart1;
                        m_requestBlockSize = 1;
                        k_timer_start(&m_commandRequestTimer, K_MSEC(2000), K_NO_WAIT);
                        break;
                    }

                    case TradeConnectionState::PartyPart1:
                    {
                        const auto party = std::as_bytes(party::getParty().subspan<200, 200>());
                        blockCommandSetup(party.data(), party.size(), 200);
                        m_blockState = TradeConnectionState::PartyPart2;
                        m_requestBlockSize = 1;
                        k_timer_start(&m_commandRequestTimer, K_MSEC(2000), K_NO_WAIT);
                        break;
                    }

                    case TradeConnectionState::PartyPart2:
                    {
                        const auto party = std::as_bytes(party::getParty().subspan<400, 200>());
                        blockCommandSetup(party.data(), party.size(), 200);
                        m_blockState = TradeConnectionState::Mail;
                        m_requestBlockSize = 3;
                        k_timer_start(&m_commandRequestTimer, K_MSEC(2000), K_NO_WAIT);
                        break;
                    }
                    
                    case TradeConnectionState::Mail:
                    {
                        const auto mail = getEmptyMailPayload();
                        blockCommandSetup(0, 0, 220);
                        m_blockState = TradeConnectionState::Ribbons;
                        m_requestBlockSize = 4;
                        k_timer_start(&m_commandRequestTimer, K_MSEC(2000), K_NO_WAIT);
                        break;
                    }

                    case TradeConnectionState::Ribbons:
                    {
                        blockCommandSetup(nullptr, 0, 40); 
                        m_blockState = TradeConnectionState::LinkCMD;
                        break;
                    }

                    default: break;

                }
                m_packetLayer.setTransiveHandler(transive);
                break;
            }

            case LINKCMD_CONT_BLOCK:
            {
                if (m_blockState == TradeConnectionState::PartyPart1 || 
                    m_blockState == TradeConnectionState::PartyPart2 || 
                    m_blockState == TradeConnectionState::Mail
                ) 
                {
                    party::partnerPartyConstruct(std::span(std::bit_cast<const uint8_t*>(command.data()), 16).subspan(2));
                }
                break;
            }
            
            default:
            {
                if (m_blockState == TradeConnectionState::LinkCMD && m_packetLayer.idle())
                {
                    return;
                }
            }
        }

        k_sleep(K_MSEC(5));
    }
}

NextSection TradeConnection::handleTradeNegotiations()
{
    NextSection nextSection = NextSection::disconnect;

    bool followupCmd = false;
    uint16_t cmd = 0x00;

    while(!m_cancel)
    {
        auto result = m_packetLayer.awaitTransiveResults();
        std::span<const uint16_t> command = result.received;

        if (followupCmd && m_packetLayer.idle())
        {
            followupCmd = false;
            sendLinkCommand(cmd);
            k_sleep(K_MSEC(5));
            continue;
        }

        switch (command[0])
        {
            case LINKCMD_CONT_BLOCK:
            {
                switch (command[1])
                {
                    case LINKCMD_INIT_BLOCK: //WTF were they thinking?
                    {
                        sendLinkCommand(LINKCMD_INIT_BLOCK);
                        followupCmd = true;
                        cmd = LINKCMD_START_TRADE;
                        break;
                    }

                    case LINKCMD_READY_TO_TRADE:
                    {
                        sendLinkCommand(LINKCMD_SET_MONS_TO_TRADE, command[2]);
                        party::tradePkmnAtIndex(command[2]);
                        break;
                    }

                    case LINKCMD_REQUEST_CANCEL:
                    {
                        sendLinkCommand(LINKCMD_REQUEST_CANCEL);
                        followupCmd = true;
                        cmd = LINKCMD_BOTH_CANCEL_TRADE;

                        nextSection = NextSection::lounge;
                        break;
                    }

                    case LINKCMD_READY_CANCEL_TRADE:
                    {
                        sendLinkCommand(LINKCMD_INIT_BLOCK);
                        followupCmd = true;
                        cmd = LINKCMD_PLAYER_CANCEL_TRADE;
                        break;
                    }
                }
                break;
            }

            case LINKCMD_READY_CLOSE_LINK:
            {
                m_packetLayer.setTransiveHandler(readyCloseLinkCommand());
                k_sleep(K_MSEC(400));
                return nextSection;
            }
        }

        k_sleep(K_MSEC(5));
    }
    return NextSection::cancel; // user canceled from web interface
}

NextSection TradeConnection::process()
{
   handleInitialDataExchange();
   return handleTradeNegotiations();
}

void TradeConnection::sendLinkCommand(uint16_t cmd, uint16_t arg)
{
    static std::array<uint16_t, 2> command;
    command[0] = cmd;
    command[1] = arg;
    blockCommandSetup(command.data(), command.size() * sizeof(uint16_t), 20);
    m_packetLayer.setTransiveHandler(blockCommand());
}