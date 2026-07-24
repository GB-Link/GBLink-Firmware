#pragma once

#include <zephyr/kernel.h>

#include "../sections/eReaderProtocolSection.hpp"
#include "moduleInterface.hpp"

class EReaderModule : public IModule
{
private:
    enum class LinkModeCommand : uint8_t
    {
        SetModeSlave = 0x11,
        StartHandshake = 0x12,
        ConnectLink = 0x13
    };

public:
    EReaderModule(erproto::Profile profile);

    void execute();

    void cancel() override;

    bool canHandle(uint8_t command) override;

    void receiveCommand(std::span<const uint8_t> command) override;

private:
    bool m_cancel = false;
    struct k_sem m_waitForLinkModeCommand;
    struct k_sem m_waitForStart;

    EReaderProtocolSection* m_currentSection = nullptr;
    erproto::Profile m_profile;
};
