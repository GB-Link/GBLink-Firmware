
#include "../layers/packet/packetLayer.hpp"
#include "nextSectionState.hpp"
#include "syscalls/kernel.h"

#pragma once 

class Section 
{
protected:
    Section() = default;
    ~Section() = default;

public:
    void cancel() 
    {
        m_packetLayer.cancel();
        m_cancel = true;
    }

protected:
    virtual NextSection process() = 0;

    inline void connectAsMaster()
    {
        m_packetLayer.configurePlayer(0, 2);

        while(m_packetLayer.getReceivedHandshake() != LINK_SLAVE_HANDSHAKE) 
        {
            k_sleep(K_MSEC(50));
            if (m_cancel) return;
        }

        m_packetLayer.enableHandshake();
        k_sleep(K_MSEC(500));
        m_packetLayer.connectHandshake();
    }

    inline void connectAsSlave()
    {
        m_packetLayer.configurePlayer(1, 2);

        while(m_packetLayer.getReceivedHandshake() != LINK_SLAVE_HANDSHAKE) 
        {
            k_sleep(K_MSEC(50));
            if (m_cancel) return;
        }

        m_packetLayer.enableHandshake();
    }

    bool m_cancel = false;
    PacketLayer m_packetLayer;
};

