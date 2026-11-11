
#include "advanceWars.hpp"
#include "../linkStatus.hpp"

void AdvanceWarsModule::execute()
{
    m_cancel = false;

    // Step 1: Ask for the player settings — the server assigns the seat
    sendLinkStatus(LinkStatus::AwaitPlayerSettings);
    k_sem_take(&m_waitForLinkModeCommand, K_FOREVER);
    if (m_cancel) return;

    // Step 2: Seat stored in receiveCommand(), emit HandshakeReceived
    // Server waits for every client to report HandshakeReceived, then sends StartHandshake
    sendLinkStatus(LinkStatus::HandshakeReceived);
    k_sem_take(&m_waitForStart, K_FOREVER);
    if (m_cancel) return;

    // Step 3: Run the protocol proxy in the assigned seat.
    // Seat 0's GBA is the bus master (P1): the adapter is in SLAVE link mode. The other seats' GBAs are children.
    {
        const enum MultiMode multiMode = (m_seat == 0) ? SLAVE : MASTER;
        AwProtocolSection section(m_variant, m_seat, m_playerCount);
        m_currentSection = &section;
        // A cancel between the check above and this assignment would miss the
        // section; re-check now that cancel() can reach it.
        if (m_cancel)
        {
            m_currentSection = nullptr;
            return;
        }
        multiMode_selectMode(multiMode);
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
        case LinkModeCommand::SetPlayerSettings:
            // [0x10, player count 2-4, seat]
            if (command.size() < 3 || command[1] < 2 || command[1] > 4 || command[2] >= command[1]) return;
            m_playerCount = command[1];
            m_seat = command[2];
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
