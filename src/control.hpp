#include "layers/usbLayer.hpp"
#include "layers/serialLayer.hpp"
#include "layers/transport.hpp"
#include "layers/packetLayer.hpp"
#include "module/link.hpp"
#include "module/emu.hpp"
#include "module/gb.hpp"
#include "module/advanceWars.hpp"
#include "module/eReader.hpp"
#include "sections/eReaderProtocolSection.hpp"
#include "linkStatus.hpp"
#include "callbacks/commands.hpp"
#include "payloads/pokemon.hpp"
#include "module/moduleInterface.hpp"
#include "hardware.hpp"
#include "persist.hpp"
#include "firmware_version.hpp"

class Control
{
    enum class ControlCommand
    {
        SetMode = 0x00,
        Cancel = 0x01,
        EnterGBPrinter = 0x03,
        GetFirmwareInfo = 0x0F
    };

    enum class Mode
    {
        gbaTradeEmu = 0x00,
        gbaLink = 0x01,
        gbLink = 0x02,
        gbPrinter = 0x03,
        advanceWars = 0x04,
        gbaEreader = 0x05,
    };

    static constexpr uint8_t callSetModeId = 0x01;

public:
    Control()
    {
        Transport::registerCommandHandler(receiveCommandHandler, this);

        k_sem_init(&m_waitForModeSemaphore, 0, 1);
    }

    void executeMode()
    {
        k_sem_take(&m_waitForModeSemaphore, K_FOREVER);
        // Capture the requested mode; callSetMode always sets m_mode before
        // giving the semaphore (including when preempting a running mode), so we
        // must not clobber it below. The variant travels with the mode.
        const Mode mode = m_mode;
        const uint8_t modeVariant = m_modeVariant;
        sendLinkStatus(LinkStatus::DeviceReady);

        switch (mode)
        {
            case Mode::gbaTradeEmu:
            {
                applyLedForSlot(LED_SLOT_GBA);
                link_detectCableType();

                party::partyInit();
                Transport::registerDataHandler(party::usbReceivePkmFile, nullptr);

                EmuModule emuModule;
                m_currentModule = &emuModule;
                emuModule.execute();

                sendLinkStatus(LinkStatus::EmuTradeSessionFinished);
                break;
            }

            case Mode::gbaLink:
            {
                applyLedForSlot(LED_SLOT_GBA);
                link_detectCableType();

                Transport::registerDataHandler(usbLink_receiveHandler, nullptr);

                LinkModule linkModule;
                m_currentModule = &linkModule;
                linkModule.execute();

                sendLinkStatus(LinkStatus::LinkClosed);
                break;
            }

            case Mode::gbLink:
            {
                GBModule gbModule;
                m_currentModule = &gbModule;
                gbModule.execute();  // Sets blue LED internally

                sendLinkStatus(LinkStatus::GBSessionFinished);
                break;
            }

            case Mode::gbPrinter:
            {
                GBModule gbModule;
                m_currentModule = &gbModule;
                gbModule.executePrinterMode();  // Sets purple LED internally

                sendLinkStatus(LinkStatus::GBSessionFinished);
                break;
            }

            case Mode::advanceWars:
            {
                applyLedForSlot(LED_SLOT_ADVANCE_WARS);
                link_detectCableType();

                Transport::registerDataHandler(awProto_receiveHandler, nullptr);

                AdvanceWarsModule advanceWarsModule(
                    modeVariant == 2 ? awproto::GameVariant::aw2
                                     : awproto::GameVariant::aw1);
                m_currentModule = &advanceWarsModule;
                advanceWarsModule.execute();

                sendLinkStatus(LinkStatus::LinkClosed);
                break;
            }

            case Mode::gbaEreader:
            {
                applyLedForSlot(LED_SLOT_EREADER);
                link_detectCableType();

                Transport::registerDataHandler(erProto_receiveHandler, nullptr);

                const erproto::Profile profile =
                    modeVariant == 2 ? erproto::Profile::pokemonRuby :
                    modeVariant == 3 ? erproto::Profile::sma4JPN :
                    modeVariant == 4 ? erproto::Profile::pokemonRubyJPN :
                    modeVariant == 5 ? erproto::Profile::sma4EUR :
                    modeVariant == 6 ? erproto::Profile::sma4EURFra :
                    modeVariant == 7 ? erproto::Profile::sma4EURGer :
                    modeVariant == 8 ? erproto::Profile::sma4EUREsp :
                    modeVariant == 9 ? erproto::Profile::sma4EURIta
                                   : erproto::Profile::sma4;
                EReaderModule ereaderModule(profile);
                m_currentModule = &ereaderModule;
                ereaderModule.execute();

                sendLinkStatus(LinkStatus::LinkClosed);
                break;
            }
        }

        m_currentModule = nullptr;
        applyLedForSlot(LED_SLOT_IDLE); // green = connected, no active mode
    }

private:
    struct k_sem m_waitForModeSemaphore;
    Mode m_mode;
    uint8_t m_modeVariant = 0;

    IModule* m_currentModule = nullptr;

    bool canHandle(uint8_t command) { return (command & 0xF0) == 0x00; }

    // Hardware commands (0x40-0x4F) are device-level, handled regardless of active module
    bool isHardwareCommand(uint8_t command) { return (command & 0xF0) == 0x40; }

    enum class HardwareCommand : uint8_t
    {
        SetVoltage3V3 = 0x40,
        SetVoltage5V = 0x41,
        SetLEDColor = 0x42,
        RebootBootloader = 0x43,
        SetWebUsbLanding = 0x44,
        GetLedConfig = 0x45,
        SetModeLedColor = 0x46,
        ResetLedColors = 0x47,
        Reboot = 0x48,
        SetCableOverride = 0x49,
        GetCableType = 0x4a,
    };

    void receiveCommand(std::span<const uint8_t> data)
    {
        if (data.empty()) return;

        // Handle hardware commands first (device-level, always available)
        if (isHardwareCommand(data[0]))
        {
            return handleHardwareCommand(data);
        }

        if (m_currentModule != nullptr && m_currentModule->canHandle(data[0]))
        {
            return m_currentModule->receiveCommand(data);
        } 
        
        if (!canHandle(data[0])) return;
        switch (static_cast<ControlCommand>(data[0]))
        {
            // Optional third byte selects a game variant within the mode
            // (Advance Wars: 1 = AW1, 2 = AW2; e-reader: 1 = SMA4 US,
            // 2 = Pokemon RS US, 3 = SMA4 JPN, 4 = Pokemon RS JPN,
            // 5-9 = SMA4 EUR En/Fr/De/Es/It; defaults to the first variant
            // for older clients that send only [command, mode]).
            case ControlCommand::SetMode:
                if (data.size() < 2) return;
                return callSetMode(static_cast<Mode>(data[1]),
                                   data.size() >= 3 ? data[2] : 0);
            case ControlCommand::Cancel: return callCancel();
            case ControlCommand::EnterGBPrinter: return callSetMode(Mode::gbPrinter);
            case ControlCommand::GetFirmwareInfo: return callGetFirmwareInfo();
            default: return;
        }
    }

    void handleHardwareCommand(std::span<const uint8_t> data)
    {
        switch (static_cast<HardwareCommand>(data[0]))
        {
            case HardwareCommand::SetVoltage3V3:
                Hardware::getInstance().setVoltage3V3();
                break;
            case HardwareCommand::SetVoltage5V:
                Hardware::getInstance().setVoltage5V();
                break;
            case HardwareCommand::SetLEDColor:
                if (data.size() >= 5) {
                    Hardware::getInstance().setLED(data[1], data[2], data[3], data[4] != 0);
                }
                break;
            case HardwareCommand::RebootBootloader:
                // Resets into the USB bootloader; does not return.
                Hardware::getInstance().rebootToBootloader();
                break;
            case HardwareCommand::SetWebUsbLanding:
                // Persist the WebUSB landing-page toggle; applies on next reconnect.
                if (data.size() >= 2) {
                    setLandingPageEnabled(data[1] != 0);
                }
                break;
            case HardwareCommand::GetLedConfig:
                return callGetLedConfig();
            case HardwareCommand::SetModeLedColor:
                // [0x46, slot, r, g, b] — persist a mode's LED colour.
                if (data.size() >= 5) {
                    setLedColor(data[1], data[2], data[3], data[4]);
                }
                break;
            case HardwareCommand::ResetLedColors:
                // Restore all per-mode LED colours to the built-in defaults.
                resetLedColors();
                break;
            case HardwareCommand::Reboot:
                // Warm-reboot into the app so persisted settings apply now; no return.
                Hardware::getInstance().reboot();
                break;
            case HardwareCommand::SetCableOverride:
                if (data.size() >= 2) {
                    link_setCableOverride(data[1]);
                }
                break;
            case HardwareCommand::GetCableType:
            {
                // Report-only: the cable is sampled once at mode entry (a GBA
                // already in link mode drives SO low, which would poison a
                // re-sample); the override command changes the path instead.
                uint8_t resp[2] = { 0x4a, link_getDetectedCableType() };
                Transport::sendData(std::span<const uint8_t>(resp, sizeof(resp)));
                break;
            }
            default: break;
        }
    }

    void callGetLedConfig()
    {
        // [0x45, count, r0,g0,b0, ... r5,g5,b5]
        uint8_t info[2 + LED_SLOT_COUNT * 3];
        info[0] = static_cast<uint8_t>(HardwareCommand::GetLedConfig);
        info[1] = LED_SLOT_COUNT;
        for (uint8_t slot = 0; slot < LED_SLOT_COUNT; slot++) {
            getLedColor(slot, &info[2 + slot * 3]);
        }
        Transport::sendData(std::span<const uint8_t>(info, sizeof(info)));
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
    // CALLS
    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    void callSetMode(Mode mode, uint8_t variant = 0)
    {
        m_mode = mode;
        m_modeVariant = variant;
        // If a mode is already running, cancel it so executeMode unblocks and
        // switches to the new mode (otherwise SetMode would queue behind a mode
        // that never exits). Same cancel path the web "cancel" command uses.
        if (m_currentModule != nullptr)
        {
            m_currentModule->cancel();
        }
        k_sem_give(&m_waitForModeSemaphore);
    }

    void callCancel()
    {
        if (m_currentModule != nullptr) m_currentModule->cancel();
    }

    void callGetFirmwareInfo()
    {
        const uint8_t info[] = {
            0x0F, // Echo back the command ID so web app knows this is a firmware info response
            fw::versionMajor,
            fw::versionMinor,
            fw::versionPatch,
            // Byte 4: WebUSB landing-page enabled (1) / disabled (0). Older web
            // apps simply ignore the extra byte.
            static_cast<uint8_t>(landingPageEnabled() ? 1 : 0)
        };
        Transport::sendData(std::span<const uint8_t>(info, sizeof(info)));
    }

    //-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

    static void receiveCommandHandler(std::span<const uint8_t> receivedData, void* userData)
    {
        Control* self = static_cast<Control*>(userData);
        self->receiveCommand(receivedData);
    }
};