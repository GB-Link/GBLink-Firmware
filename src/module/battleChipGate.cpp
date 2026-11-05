#include "battleChipGate.hpp"
#include "../linkStatus.hpp"

extern "C"
{
    #include "../layers/linkLayer.h"
}

void BattleChipGateModule::execute()
{
    m_cancel = false;

    {
        BattleChipGateSection section(m_gateType);
        m_currentSection = &section;
        if (m_cancel)
        {
            m_currentSection = nullptr;
            return;
        }

        section.armLink();
        sendLinkStatus(LinkStatus::LinkConnected);
        section.process();
        link_changeMode(DISABLED);
        link_releasePartnerPins();
    }
    m_currentSection = nullptr;
}
