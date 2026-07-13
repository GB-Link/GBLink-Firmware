#include "multiLink.hpp"

#include "../linkStatus.hpp"

void MultiLinkModule::execute()
{
    m_cancel = false;

    // Clamp the untrusted SetMode bytes: count 2-4, seat 0..count-1. The
    // link layer does unsigned N-1 arithmetic on these, so out-of-range
    // values would underflow into buffer overruns.
    const uint8_t players = (m_players >= 2 && m_players <= 4) ? m_players : 2;
    const uint8_t seat = (m_seat < players) ? m_seat : 0;

    {
        MultiLinkSection section(seat, players);
        m_currentSection = &section;
        if (m_cancel)
        {
            m_currentSection = nullptr;
            return;
        }

        if (seat == 0)
        {
            // Host: the attached cart is the bus parent; present N-1 children.
            link_setChildSlots((uint8_t)(players - 1));
            link_changeMode(SLAVE);
        }
        else
        {
            // Joiner: the attached cart is a child at seat `seat`; the adapter
            // is the bus master driving the other seats.
            link_setMasterMulti(players, seat);
            link_changeMode(MASTER);
        }

        section.process();

        // Stop the PIO before the section destructor deregisters the callbacks.
        link_changeMode(DISABLED);
    }

    m_currentSection = nullptr;
}
