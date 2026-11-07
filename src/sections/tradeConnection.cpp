#include "tradeConnection.hpp"

#include "../payloads/pokemon.hpp"
#include <bit>
#include <cstdint>
#include <sys/types.h>
extern "C"
{
    #include "../payloads/linkPlayer.h"
}
#include "../layers/packet/packetCommands/commands.hpp"

void TradeConnection::handleInitialDataExchange()
{
    connectAsMaster();
    m_packetLayer.sendCommand(sendLinkTypeCommand(LINKTYPE_TRADE_CONNECTING));

    while (!m_cancel)
    {
        PacketLayer::TransiveResult result = m_packetLayer.awaitTransiveResults();
        std::span<const uint16_t> command = result.received;

        if (m_requestBlock)
        {
            m_requestBlock = false;
            m_packetLayer.sendCommand(sendBlockCommandRequestCommand(m_requestBlockSize));

            k_sleep(K_MSEC(5));
            continue;
        }


        switch(command[0])
        {
            case LINKCMD_INIT_BLOCK:
            {
                switch(m_blockState)
                {
                    case TradeConnectionState::LinkPlayer:
                    {
                        const struct LinkPlayerBlock* linkPlayerBlock = linkPLayer(LINKTYPE_TRADE_CONNECTING);
                        m_blockState = TradeConnectionState::PartyPart0;
                        m_requestBlockSize = 1;
                        party::partnerPartyInit();
                        k_timer_start(&m_commandRequestTimer, K_MSEC(2000), K_NO_WAIT);
                        m_packetLayer.sendCommand(blockCommand(linkPlayerBlock, sizeof(*linkPlayerBlock), sizeof(*linkPlayerBlock)));
                        break;
                    }

                    case TradeConnectionState::PartyPart0:
                    {
                        const auto party = std::as_bytes(party::getParty().subspan<0, 200>());
                        m_blockState = TradeConnectionState::PartyPart1;
                        m_requestBlockSize = 1;
                        k_timer_start(&m_commandRequestTimer, K_MSEC(2000), K_NO_WAIT);
                        m_packetLayer.sendCommand(blockCommand(party.data(), party.size(), 200));
                        break;
                    }

                    case TradeConnectionState::PartyPart1:
                    {
                        const auto party = std::as_bytes(party::getParty().subspan<200, 200>());
                        m_blockState = TradeConnectionState::PartyPart2;
                        m_requestBlockSize = 1;
                        k_timer_start(&m_commandRequestTimer, K_MSEC(2000), K_NO_WAIT);
                        m_packetLayer.sendCommand(blockCommand(party.data(), party.size(), 200));
                        break;
                    }

                    case TradeConnectionState::PartyPart2:
                    {
                        const auto party = std::as_bytes(party::getParty().subspan<400, 200>());
                        m_blockState = TradeConnectionState::Mail;
                        m_requestBlockSize = 3;
                        m_packetLayer.sendCommand(blockCommand(party.data(), party.size(), 200));
                        k_timer_start(&m_commandRequestTimer, K_MSEC(2000), K_NO_WAIT);
                        break;
                    }
                    
                    case TradeConnectionState::Mail:
                    {
                        //const auto mail = getEmptyMailPayload();
                        m_blockState = TradeConnectionState::Ribbons;
                        m_requestBlockSize = 4;
                        m_packetLayer.sendCommand(blockCommand(0, 0, 220));
                        k_timer_start(&m_commandRequestTimer, K_MSEC(2000), K_NO_WAIT);
                        break;
                    }

                    case TradeConnectionState::Ribbons:
                    {
                        m_packetLayer.sendCommand(blockCommand(nullptr, 0, 40));
                        m_blockState = TradeConnectionState::LinkCMD;
                        break;
                    }

                    default: break;

                }
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
                m_packetLayer.sendCommand(readyCloseLinkCommand());
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
    m_packetLayer.sendCommand(blockCommand(command.data(), command.size() * sizeof(uint16_t), 20));
}