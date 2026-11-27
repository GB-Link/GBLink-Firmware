#include <zephyr/kernel.h>

#include "../sections//usbSection.hpp"
#include "moduleInterface.hpp"
#include "syscalls/kernel.h"

class LinkModule : public IModule
{
private:
    enum class LinkModeCommand : uint8_t
    {
        SetPlayerSettings = 0x10,
        StartHandshake = 0x12,
        ConnectLink = 0x13
    };

public:
    LinkModule()
    {
        k_sem_init(&m_waitForLinkModeCommand, 0, 1);
    }

    void execute();

    void cancel() override
    { 
        m_cancel = true;
        k_sem_give(&m_waitForLinkModeCommand);
        if (m_currentSection) m_currentSection->cancel();
    }

    bool canHandle(uint8_t command) override { return (command & 0xF0) == 0x10; }

    void receiveCommand(std::span<const uint8_t> command) override;

private:

    bool m_cancel = false;
    struct k_sem m_waitForLinkModeCommand;

    UsbSection* m_currentSection = nullptr;

    uint8_t m_playerCount;
    uint8_t m_seatNumber;

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
    // CALLS
    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
};

