#pragma once

#include <zephyr/kernel.h>

#include "../sections/battleChipGateSection.hpp"
#include "moduleInterface.hpp"

class BattleChipGateModule : public IModule
{
public:
    explicit BattleChipGateModule(bcgproto::GateType gateType)
        : m_gateType(gateType)
    {
    }

    void execute();

    void cancel() override
    {
        m_cancel = true;
        if (m_currentSection) m_currentSection->cancel();
    }

    bool canHandle(uint8_t) override { return false; }

    void receiveCommand(std::span<const uint8_t>) override {}

private:
    bool m_cancel = false;
    BattleChipGateSection* m_currentSection = nullptr;
    bcgproto::GateType m_gateType;
};
