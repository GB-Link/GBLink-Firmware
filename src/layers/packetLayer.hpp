
#include <cstdint>
extern "C"
{
    #include "linkLayer.h"
}

#include "../callbacks/commands.hpp"
#include "../link_defines.h"

#ifdef CONFIG_STM32F0
#include "masterClock.hpp"
#endif

#include <span>
#include <zephyr/drivers/gpio.h>

#pragma once

class PacketLayer
{
public:
    enum class Mode
    {
        master,
        slave
    };

    struct TransiveResult
    {
        std::span<const uint16_t> received;
        std::span<const uint16_t> transmitted;
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

    static constexpr uint32_t timingHandshake = 30097;
    static constexpr uint32_t timingCommandBytes = 1378;
    static constexpr uint32_t timingBetweenCommands = 12953;

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
    }

    void setTransiveHandler(struct TransiveStruct handler)
    {
        m_handler = handler;
        m_idle = false;
        if (handler.init != nullptr) handler.init(handler.userData);
    }
    
    TransiveResult awaitTransiveResults() 
    { 
        k_sem_take(&m_commandTransiveSemaphore, K_FOREVER);
        return TransiveResult
        {
            std::span(m_receivedCommand),
            std::span(m_transmittedCommand)
        };
    }

    bool tryAwaitTransiveResults(TransiveResult& result, k_timeout_t timeout)
    {
        if (k_sem_take(&m_commandTransiveSemaphore, timeout) != 0) return false;
        result = TransiveResult{std::span(m_receivedCommand), std::span(m_transmittedCommand)};
        return true;
    }

    uint16_t getReceivedHandshake()
    {
        k_sem_take(&m_handshakeSemaphore, K_FOREVER);
        return m_receivedHandshake;
    }

    bool takeHandshake(uint16_t& rx, k_timeout_t timeout)
    {
        if (k_sem_take(&m_handshakeSemaphore, timeout) != 0) return false;
        rx = m_receivedHandshake;
        return true;
    }

    uint16_t getTransmittedHandshake()
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

    bool isHandshakeEnabled() { return m_transmitedHandShake == LINK_SLAVE_HANDSHAKE; }

    void setMode(Mode mode) 
    {   
        switch (mode)
        {
            case Mode::master:
                link_changeMode(MASTER);
                #ifdef CONFIG_STM32F0
                m_masterClock.enableSync();
                #endif
                break;
            
            case Mode::slave:
                link_changeMode(SLAVE);
                #ifdef CONFIG_STM32F0
                m_masterClock.disableSync();
                #endif
                break;
        }

        m_mode = mode;
    }

    Mode getMode() { return m_mode; }

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
            case TransiveState::crc: return {transmitCrc(), m_timingUs};
            case TransiveState::command: return {transmitCommand(), m_timingUs};
            case TransiveState::handshake: return {transmitHandshake(), m_timingUs};
            default: return {0x00, m_timingUs};
        };
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    void receiveHandshake(uint16_t rxBytes)
    {
        m_receivedHandshake = rxBytes;
    }

    uint16_t transmitHandshake()
    {
        // Don't set m_transmitedHandShake from outside to advance handshake state
        // Setting m_transmitedHandShake will create race conditions bad things happen
        switch(m_handshakeState)
        {
            case HandShakeState::disabled: return LINK_HANDSHAKE_DISABLE;
            case HandShakeState::enabled: return LINK_SLAVE_HANDSHAKE;
            case HandShakeState::connect: return LINK_MASTER_HANDSHAKE;
        }

        return 0xDEAD;
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    void receiveCrc(uint16_t rxBytes)
    {
        // don't care
    }

    uint16_t transmitCrc()
    {
        return m_crc;
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    void receiveCommand(uint16_t rxBytes)
    {
        m_receivedCommand[m_commandIndex] = rxBytes;
        m_crc += rxBytes;
        m_commandIndex++;
    }

    uint16_t transmitCommand()
    {
        uint16_t txBytes = (m_handler.transive != nullptr) ? m_handler.transive() : 0x00;
        m_crc += txBytes;
        m_transmittedCommand[m_transmitCommandIndex] = txBytes;
        m_transmitCommandIndex++;
        return txBytes;
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    void onTransiveDone(uint16_t rxBytes, uint16_t txBytes);

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

private:
    atomic_t m_waitForDisable = 0;

    bool m_idle = true;
    Mode m_mode = Mode::slave;

    uint16_t m_receivedHandshake = LINK_HANDSHAKE_DISABLE;
    uint16_t m_transmitedHandShake = LINK_HANDSHAKE_DISABLE;
    uint16_t m_crc = LINK_SLAVE_HANDSHAKE; //first crc is always handshake

    uint32_t m_timingUs = 0;

    struct k_sem m_commandTransiveSemaphore;
    struct k_sem m_handshakeSemaphore;
    struct k_sem m_saveToDisableSemaphore;

    int m_commandIndex = 0;
    std::array<uint16_t, 8> m_receivedCommand = {};
    int m_transmitCommandIndex = 0;
    std::array<uint16_t, 8> m_transmittedCommand = {};

    TransiveStruct m_handler = emptyCommand();
    TransiveState m_state = TransiveState::handshake;
    HandShakeState m_handshakeState = HandShakeState::disabled;

    struct k_timer m_timeoutTimer;
    #ifdef CONFIG_STM32F0
    MasterClock m_masterClock;
    #endif
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

    static void transiveDoneCallback(uint16_t rxBytes, uint16_t txBytes, void* userData)
    {
        PacketLayer* self = static_cast<PacketLayer*>(userData);
        self->onTransiveDone(rxBytes, txBytes);
    }

    #ifdef CONFIG_STM32F0
    static void packetTimeout(struct k_timer *timer)
    {
        void* userData = k_timer_user_data_get(timer);
        PacketLayer* self = static_cast<PacketLayer*>(userData);
        self->reset();
    }
    #endif
};