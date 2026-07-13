#pragma once

#include <span>
#include <zephyr/kernel.h>

#include "moduleInterface.hpp"
#include "../sections/multiLinkSection.hpp"

// Multiplayer gen-3 link relay. The server assigns the seat (0 = host) and
// player count via the SetMode payload; the section runs immediately in the
// right role with no per-session mode handshake. The payload is untrusted
// host input, so execute() clamps it before it reaches the link layer.
class MultiLinkModule : public IModule
{
public:
    MultiLinkModule(uint8_t seat, uint8_t playerCount)
        : m_seat(seat), m_players(playerCount) {}

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
    uint8_t m_seat;
    uint8_t m_players;
    MultiLinkSection* m_currentSection = nullptr;
};
