
#include "rfuWireless.hpp"
#include "../linkStatus.hpp"

void RfuWirelessModule::execute()
{
    m_cancel = false;

    // Unlike the cable modes, the wireless adapter must be LIVE on the GBA
    // link immediately: the game runs AgbRFU_checkID (adapter detection) and
    // enters the Union Room before any network partner exists, so the link
    // cannot wait for the server's pairing handshake. Enable it now and run
    // the section right away; the network relay attaches asynchronously as a
    // partner joins the session (handled in receiveCommand, which keeps the
    // server's AwaitMode→…→LinkConnected flow progressing without gating the
    // GBA-side adapter). The master/slave role is irrelevant — the protocol
    // core is symmetric and who hosts is decided in-game.
    sendLinkStatus(LinkStatus::AwaitMode);

    {
        RfuProtocolSection section(m_role);
        m_currentSection = &section;
        if (m_cancel)
        {
            m_currentSection = nullptr;
            return;
        }
        rfuLink_enable();
        section.process();
        // Stop the PIO before the section destructor deregisters the done
        // callback — the adapter-master state machine free-runs and its ISR
        // must not fire mid-deregistration.
        rfuLink_disable();
    }
    m_currentSection = nullptr;
}

void RfuWirelessModule::receiveCommand(std::span<const uint8_t> command)
{
    // Progress the server's pairing handshake without blocking the (already
    // running) GBA link, so deviceData relay is established once a partner is
    // present while detection/Union Room work from the moment the mode starts.
    switch (static_cast<LinkModeCommand>(command[0]))
    {
        case LinkModeCommand::SetModeMaster:
        case LinkModeCommand::SetModeSlave:
            sendLinkStatus(LinkStatus::HandshakeReceived);
            break;

        case LinkModeCommand::StartHandshake:
            sendLinkStatus(LinkStatus::LinkConnected);
            break;

        case LinkModeCommand::ConnectLink:
            // Ignored — the RFU protocol core handles link establishment
            break;

        default: break;
    }
}
