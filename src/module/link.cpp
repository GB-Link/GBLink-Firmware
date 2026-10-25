#include "link.hpp"
#include "../linkStatus.hpp"
#include "../layers/packet/packetCommands/commands.hpp"

#include "syscalls/kernel.h"
#include "zephyr/kernel.h"
#include <zephyr/irq.h>


void LinkModule::execute()
{
    m_cancel = false;
    sendLinkStatus(LinkStatus::AwaitPlayerSettings);
    k_sem_take(&m_waitForLinkModeCommand, K_FOREVER);
    bool keepAlive = true;

    while (keepAlive && !m_cancel)
    {
        {
            UsbSection section(m_seatNumber, m_playerCount);
            m_currentSection = &section;
            keepAlive = section.process();
        }
        m_currentSection = nullptr; //kinda sketch but commands should only arrive when we have a current section

        if (keepAlive)
        {
            sendLinkStatus(LinkStatus::LinkReconnecting);
            k_sleep(K_MSEC(400));
        }
    }
}

void LinkModule::receiveCommand(std::span<const uint8_t> command)
{
    switch (static_cast<LinkModeCommand>(command[0]))
    {
        case LinkModeCommand::SetPlayerSettings:
            m_playerCount = command[1];
            m_seatNumber = command[2];
            k_sem_give(&m_waitForLinkModeCommand);            
            break;
        case LinkModeCommand::StartHandshake: return m_currentSection->startHandshake();
        case LinkModeCommand::ConnectLink: return m_currentSection->connectLink();
        default: break;
    }
}

