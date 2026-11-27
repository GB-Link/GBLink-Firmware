#include "packetLayer.hpp"
#include "../link/multiMode/multiMode.hpp"

#include "syscalls/kernel.h"
#include <cerrno>

void PacketLayer::onTransiveDone(uint16_t rxBytes, TxValues txValues)
{
    switch (m_state)
    {
        case TransiveState::handshake:
            m_transmitedHandShake = txValues;
            m_receivedHandshake = rxBytes;
            k_sem_give(&m_handshakeSemaphore);
            m_timingUs = timingHandshake[m_playerCount-2];

            if (rxBytes == LINK_MASTER_HANDSHAKE 
                || txValues[0] == LINK_MASTER_HANDSHAKE)
            {
                m_state = TransiveState::crc;
                m_timingUs = timingCommandBytes[m_playerCount-2];
            }
            break;

        case TransiveState::crc:
            m_state = TransiveState::command;
            m_crc = 0x00;
            break;

        case TransiveState::command:
            
            if (m_commandIndex == 7)
            {
                m_timingUs = timingBetweenCommands[m_playerCount-2];
            }

            if (m_commandIndex != 8) return;

            m_commandIndex = 0;
            m_transmitCommandIndex = 0;
            m_state = TransiveState::crc;
            m_timingUs = timingCommandBytes[m_playerCount-2];
            k_sem_give(&m_commandTransiveSemaphore);
            if (m_waitForDisable >= 1)
            {
                m_waitForDisable = 0;
                k_sem_give(&m_saveToDisableSemaphore);
            }
            // Runs in the PIO ISR: only flag completion. Replacing m_handler here would
            // free the command's heap context, and free() takes a k_mutex (asserts in ISR).
            // The finished command stays alive until the thread's next sendCommand().
            if (!m_idle && m_handler.transmitDone() == CommandState::done)
            {
                m_idle = true;
            }
            break;
        
        default: break;
    }
}

bool PacketLayer::awaitDisable()
{
    // As slave, just disable since we can't gurantee that the command will be completed
    // As master, let's shutdown gracefully and wait for last command to complete
    if (m_seatNumber == 0)
    {
        m_waitForDisable = 1;
        if (k_sem_take(&m_saveToDisableSemaphore, K_MSEC(100)) == -EAGAIN)
        {
            // Stopped on a timeout too: the link callbacks still point at this object.
            multiMode_selectMode(DISABLED);
            return false;
        }
    }
    
    multiMode_selectMode(DISABLED);
    return true;
}