#include <span>

#include "../layers/legacyPacketLayer.hpp"

#include "nextSectionState.hpp"
#include "section.hpp"

#pragma once

class TradeDisconnect : public Section
{
    enum class BlockCommandState : uint8_t
    {
        None = 0x00,
        LinkPlayer = 0x01,
        FinishTrade = 0x02
    };

    void exchangeTrainerData();

    NextSection handleDisconnect();

public:
    TradeDisconnect() {}

    ~TradeDisconnect()
    {
        while(!m_packetLayer.idle()) {};
    }

    NextSection process() override;

private:
    BlockCommandState m_blockState = BlockCommandState::None;
};