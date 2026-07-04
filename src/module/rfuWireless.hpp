
#include <zephyr/kernel.h>

#include "../sections/rfuProtocolSection.hpp"
#include "moduleInterface.hpp"

class RfuWirelessModule : public IModule
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
    // role: Union-Room mesh-break lock — 0 symmetric / 1 host-lock / 2 client-lock.
    explicit RfuWirelessModule(uint8_t role = 0) : m_role(role) {}

    void execute();

    void cancel() override
    {
        m_cancel = true;
        if (m_currentSection) m_currentSection->cancel();
    }

    bool canHandle(uint8_t command) override { return (command & 0xF0) == 0x10; }

    void receiveCommand(std::span<const uint8_t> command) override;

private:

    bool m_cancel = false;
    uint8_t m_role = 0;  // 0 symmetric / 1 host-lock / 2 client-lock
    RfuProtocolSection* m_currentSection = nullptr;
};
