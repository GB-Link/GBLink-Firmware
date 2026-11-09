
#include "advanceWars.hpp"
#include "../linkStatus.hpp"

void AdvanceWarsModule::execute()
{
    m_cancel = false;

    // Step 1: Emit AwaitMode — server will assign master/slave
    sendLinkStatus(LinkStatus::AwaitPlayerSettings);
    k_sem_take(&m_waitForLinkModeCommand, K_FOREVER);
    if (m_cancel) return;

    // Step 2: Mode stored in receiveCommand(), emit HandshakeReceived
    // Server waits for both clients to report HandshakeReceived, then sends StartHandshake
    sendLinkStatus(LinkStatus::HandshakeReceived);
    k_sem_take(&m_waitForStart, K_FOREVER);
    if (m_cancel) return;

    // Step 3: Run the protocol proxy in the assigned role.
    // SLAVE = attached GBA is bus master (P1), MASTER = GBA is bus slave (P2).
    {
        AwProtocolSection section(m_variant, m_multiMode);
        m_currentSection = &section;
        // A cancel between the check above and this assignment would miss the
        // section; re-check now that cancel() can reach it.
        if (m_cancel)
        {
            m_currentSection = nullptr;
            return;
        }
        multiMode_selectMode(m_multiMode);
        sendLinkStatus(LinkStatus::LinkConnected);
        section.process();
        // Stop the PIO before the section destructor deregisters the link
        // callbacks — a master-mode PIO free-runs and its ISR must not fire
        // mid-deregistration. Also stops the GBA from being fed the last
        // staged word after the session ends.
        multiMode_selectMode(DISABLED);
    }
    m_currentSection = nullptr;
}

void AdvanceWarsModule::receiveCommand(std::span<const uint8_t> command)
{
    switch (static_cast<LinkModeCommand>(command[0]))
    {
        case LinkModeCommand::SetModeMaster:
            // The 4-player server sends [0x10, player count, seat]: seat 0's GBA is player 1, the bus master.
            m_multiMode = (command.size() >= 3 && command[2] == 0) ? SLAVE : MASTER;
            k_sem_give(&m_waitForLinkModeCommand);
            break;

        case LinkModeCommand::SetModeSlave:
            m_multiMode = SLAVE;
            k_sem_give(&m_waitForLinkModeCommand);
            break;

        case LinkModeCommand::StartHandshake:
            k_sem_give(&m_waitForStart);
            break;

        case LinkModeCommand::ConnectLink:
            // Ignored in AW mode — the protocol proxy handles link establishment
            break;

        default: break;
    }
}
