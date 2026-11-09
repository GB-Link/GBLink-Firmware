#include "../link/multiMode/multiMode.hpp"
#include "../link/gbaLinkLayer.hpp"

#include "./packetCommands/commands.hpp"
#include "../../link_defines.h"

#include <cstdint>
#include <span>
#include <optional>
#include <utility>
#include <zephyr/drivers/gpio.h>

#pragma once

class PacketLayer
{
public:
    struct TransiveResult
    {
        std::span<const uint16_t> received;
        std::span<TxValues> transmitted;
    };

private:

    enum class TransiveState
    {
        crc,
        command,
        handshake
    };

    enum class HandShakeState
    {
        disabled,
        enabled,
        connect
    };

    //cycle timings, each cycle is 0.5425 µs
    static constexpr std::array<uint32_t, 3> timingHandshake = {29868, 29551, 29282};
    static constexpr std::array<uint32_t, 3> timingCommandBytes = {1098, 1166, 1178};
    static constexpr std::array<uint32_t, 3> timingBetweenCommands = {12673, 9758, 7253};

public:
    PacketLayer()
    {
        link_setTransmitCallback(&transmitCallback, this);
        link_setReceiveCallback(&receiveCallback, this);
        link_setTransiveDoneCallback(&transiveDoneCallback, this);
        
        k_sem_init(&m_commandTransiveSemaphore, 0, 1);
        k_sem_init(&m_handshakeSemaphore, 0, 1);
        k_sem_init(&m_saveToDisableSemaphore, 0, 1);
        #ifdef CONFIG_STM32F0
        k_timer_init(&m_timeoutTimer, &packetTimeout, nullptr);
        k_timer_user_data_set(&m_timeoutTimer, this);
        #endif
    }

    ~PacketLayer()
    {
        awaitDisable();
        link_setTransmitCallback(nullptr, nullptr);
        link_setReceiveCallback(nullptr, nullptr);
        link_setTransiveDoneCallback(nullptr, nullptr);

    }

    void sendCommand(TransmitBehaviour handler)
    {
        // Swap under irq_lock so the ISR never sees a half-moved handler; the previous
        // command ends up in `handler` and is destroyed on return, in thread context.
        const unsigned int key = irq_lock();
        std::swap(m_handler, handler);
        m_idle = false;
        irq_unlock(key);

    }
    
    TransiveResult awaitTransiveResults()
    {
        return *awaitTransiveResults(K_FOREVER);
    }

    std::optional<TransiveResult> awaitTransiveResults(k_timeout_t timeout)
    {
        if (k_sem_take(&m_commandTransiveSemaphore, timeout) != 0) return std::nullopt;
        return TransiveResult
        {
            std::span(m_receivedCommand),
            std::span(m_transmittedCommand)
        };
    }

    uint16_t getReceivedHandshake()
    {
        return *getReceivedHandshake(K_FOREVER);
    }

    std::optional<uint16_t> getReceivedHandshake(k_timeout_t timeout)
    {
        if (k_sem_take(&m_handshakeSemaphore, timeout) != 0) return std::nullopt;
        return m_receivedHandshake;
    }

    TxValues getTransmittedHandshake()
    {
        k_sem_take(&m_handshakeSemaphore, K_FOREVER);
        return m_transmitedHandShake;
    }

    void cancel() 
    {  
        // just give to every semaphore so we can make sure we don't softlock,
        // object will be destroyed shorty anway so no concern about state
        k_sem_give(&m_handshakeSemaphore); 
        k_sem_give(&m_commandTransiveSemaphore);
        k_sem_give(&m_saveToDisableSemaphore);
    }

    bool idle() { return m_idle; }

    void enableHandshake() { m_handshakeState = HandShakeState::enabled; }

    void connectHandshake() { m_handshakeState = HandShakeState::connect; }

    bool isHandshakeEnabled() { return m_transmitedHandShake[0] == LINK_SLAVE_HANDSHAKE; }

    void configurePlayer(uint8_t seatNumber, uint8_t playerCount)
    {   
        m_playerCount = playerCount;
        m_seatNumber = seatNumber;

        if (seatNumber == 0)
        {
             
            multiMode_selectMode(SLAVE);
        }
        else
        {
            multiMode_selectMode(MASTER);
        }

       
    }

private:

    bool awaitDisable();

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    void onReceive(uint16_t rxBytes)
    {
        switch(m_state)
        {
            case TransiveState::crc: return receiveCrc(rxBytes);
            case TransiveState::command: return receiveCommand(rxBytes);
            case TransiveState::handshake: return receiveHandshake(rxBytes);
            default: return;
        };
    }


    struct NextTransmit onTransmit()
    {
        switch(m_state)
        {
            case TransiveState::crc: return {m_seatNumber, transmitCrc(), m_timingUs};
            case TransiveState::command: return {m_seatNumber, transmitCommand(), m_timingUs};
            case TransiveState::handshake: return {m_seatNumber, transmitHandshake(), m_timingUs};
            default: return {0, {}, m_timingUs};
        };
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    void receiveHandshake(uint16_t rxBytes)
    {
        m_receivedHandshake = rxBytes;
    }

    TxValues transmitHandshake()
    {
        TxValues handshakes;

        uint16_t handshakeValue = 0;
        switch(m_handshakeState)
        {
            case HandShakeState::disabled:
                handshakeValue = LINK_HANDSHAKE_DISABLE;
                break;
            case HandShakeState::enabled:
                handshakeValue = LINK_SLAVE_HANDSHAKE;
                break;
            case HandShakeState::connect:
                handshakeValue = LINK_SLAVE_HANDSHAKE;
                break;
        }

        for (uint8_t i = 0; i < m_playerCount - 1; i++)
        {
            if (m_handshakeState == HandShakeState::connect && i == 0)
            {
                handshakes.push(LINK_MASTER_HANDSHAKE);
            }
            else 
            {
                handshakes.push(handshakeValue);
            }
            
        }

        return handshakes;
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    void receiveCrc(uint16_t rxBytes)
    {
        (void)rxBytes;
        // don't care
    }

    TxValues transmitCrc()
    {
        TxValues crcValues;
        for (uint8_t i = 0; i < m_playerCount - 1; i++)
        {
            crcValues.push(m_crc);
        }
        
        return crcValues;
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    void receiveCommand(uint16_t rxBytes)
    {
        m_receivedCommand[m_commandIndex] = rxBytes;
        m_crc += rxBytes;
        m_commandIndex++;
    }

    TxValues transmitCommand()
    {
        TxValues txValues;
        if (m_idle)
        {
            // Finished command is still parked in m_handler; send idle zeros instead.
            for (uint8_t i = 0; i < m_playerCount - 1; i++) txValues.push(0x00);
        }
        else
        {
            txValues = m_handler.transmit();
        }

        m_transmittedCommand[m_transmitCommandIndex] = txValues;
        for (uint8_t i = 0; i < m_playerCount - 1; i++)
        {
            m_crc += txValues[i];
        }
        m_transmitCommandIndex++;
        return txValues;
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    void onTransiveDone(uint16_t rxBytes, TxValues txValues);

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

private:
    atomic_t m_waitForDisable = 0;

    volatile bool m_idle = true;

    uint8_t m_seatNumber = 1;
    uint8_t m_playerCount = 2;

    uint16_t m_receivedHandshake = LINK_HANDSHAKE_DISABLE;
    TxValues m_transmitedHandShake;
    uint16_t m_crc = LINK_SLAVE_HANDSHAKE; //first crc is always handshake

    uint32_t m_timingUs = 0;

    struct k_sem m_commandTransiveSemaphore;
    struct k_sem m_handshakeSemaphore;
    struct k_sem m_saveToDisableSemaphore;

    int m_commandIndex = 0;
    std::array<uint16_t, 8> m_receivedCommand = {};
    int m_transmitCommandIndex = 0;
    std::array<TxValues, 8> m_transmittedCommand = {};

    TransmitBehaviour m_handler = emptyCommand();
    TransiveState m_state = TransiveState::handshake;
    HandShakeState m_handshakeState = HandShakeState::disabled;

    struct k_timer m_timeoutTimer;

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
    // Callbacks
    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    static void receiveCallback(uint16_t rxBytes, void* userData)
    {
        PacketLayer* self = static_cast<PacketLayer*>(userData);
        return self->onReceive(rxBytes);
    }


    static struct NextTransmit transmitCallback(void* userData)
    {
        PacketLayer* self = static_cast<PacketLayer*>(userData);
        return self->onTransmit();
    }

    static void transiveDoneCallback(uint16_t rxBytes, TxValues txValues, void* userData)
    {
        PacketLayer* self = static_cast<PacketLayer*>(userData);
        self->onTransiveDone(rxBytes, txValues);
    }

};
