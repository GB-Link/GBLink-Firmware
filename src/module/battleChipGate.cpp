#include "battleChipGate.hpp"
#include "../linkStatus.hpp"

#include "../layers/link/multiMode/multiMode.hpp"
#include "../layers/link/ereader/ereader.hpp"


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
        multiMode_selectMode(DISABLED);
        ereader_releasePartnerPins();
    }
    m_currentSection = nullptr;
}
