#pragma once

#include <span>
#include <zephyr/kernel.h>

#include "moduleInterface.hpp"

class GBModule : public IModule
{
public:
    void execute();
    void executePrinterMode();
    void cancel() override;
    bool canHandle(uint8_t command) override;
    void receiveCommand(std::span<const uint8_t> command) override;

private:
    // The printer sub-mode; the link's own commands are gbproto::Command (sections/gbProtocol.hpp)
    enum class GBCommand : uint8_t
    {
        EnterPrinterMode = 0x31,
        ExitPrinterMode = 0x32,
    };

    enum class SubMode { normal, printer };

    bool m_cancel = false;
    SubMode m_subMode = SubMode::normal;

    // --- Normal mode: the Game Boy link protocol (sections/gbProtocol.hpp) ---
    void normalModeLoop();
    void handleHostData(const uint8_t* data, size_t count);
    void handleCommand(const uint8_t* data, size_t count);
    void transferActive(const uint8_t* data, size_t count);

    // --- Printer mode: the Game Boy prints through the passive program (sections/gbPrinter.hpp) ---
    void printerModeLoop();

    // --- USB data handler ---
    static void usbDataHandler(std::span<const uint8_t> data, void* userData);
};
