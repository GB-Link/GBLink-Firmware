#pragma once

#include <cstdint>
#include <span>

#include <zephyr/kernel.h>

#include "battleChipGateProtocol.hpp"

class BattleChipGateSection
{
public:
    explicit BattleChipGateSection(bcgproto::GateType initialGateType);
    ~BattleChipGateSection();

    void armLink();
    void process();
    void cancel() { m_cancel = true; }

private:
    bool m_cancel = false;
};

void bcgProto_receiveHandler(std::span<const uint8_t> data, void*);
void bcgProto_requestRearm();
