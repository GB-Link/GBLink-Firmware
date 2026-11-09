
#include <zephyr/kernel.h>

#include "../sections/awProtocolSection.hpp"
#include "moduleInterface.hpp"

class AdvanceWarsModule : public IModule
{
private:
    enum class LinkModeCommand : uint8_t
    {
        SetModeMaster = 0x10,
        SetModeSlave = 0x11,
        StartHandshake = 0x12,
        ConnectLink = 0x13
    };

public:
    AdvanceWarsModule(awproto::GameVariant variant)
        : m_variant(variant)
    {
        k_sem_init(&m_waitForLinkModeCommand, 0, 1);
        k_sem_init(&m_waitForStart, 0, 1);
    }

    void execute();

    void cancel() override
    {
        m_cancel = true;
        k_sem_give(&m_waitForLinkModeCommand);
        k_sem_give(&m_waitForStart);
        if (m_currentSection) m_currentSection->cancel();
    }

    bool canHandle(uint8_t command) override { return (command & 0xF0) == 0x10; }

    void receiveCommand(std::span<const uint8_t> command) override;

private:

    bool m_cancel = false;
    struct k_sem m_waitForLinkModeCommand;
    struct k_sem m_waitForStart;

    AwProtocolSection* m_currentSection = nullptr;
    enum MultiMode m_multiMode = SLAVE;
    awproto::GameVariant m_variant;
};
