# GB-Link/Celio-Link firmware

This firmware turns an RP2040 into a USB-to-Game Boy Link Cable adapter, bringing various Game Boy and Game Boy Advance games into the Internet age.

The firmware originally stems from the Celio Firmware of the CelioLink project, whose goal is to bring online functionality to the Pokémon Generation III games. It was initially developed independently, but over time it merged with the GBLink project, which added support for playing Tetris online, sending multiboot files, emulating the Game Boy Printer, and playing Advance Wars 1 and 2 online.

The GBLink fork and this repository share the same core codebase and releases, but they are maintained separately to honor the projects from which they originated.

You are free to choose whichever web frontend you prefer to use:

* https://launcher.gblink.io Launcher to all subprojects
* https://celi0.link The GBA part of the project

## Quick Start

Web flash/update
https://launcher.gblink.io

Manual update
1. Download `gblink.uf2` from the latest **[Release](../../releases)**.
2. Hold the **BOOTSEL** button on the RP2040.
3. Connect the device to your computer.
4. Drag and drop the `.uf2` file onto the mounted drive.

---

## Supported Modes

| Mode | Command | Description |
|:---|:---|:---|
| **GBA Trade Emu** | `0x00` | Emulate a GBA link partner for Pokémon Gen 3 trades |
| **GBA Link** | `0x01` | Bridge two GBA systems over the internet |
| **GB Link** | `0x02` | SPI passthrough for Game Boy |
| **GB Printer** | `0x03` | Game Boy Printer emulation (bit-bang SPI slave) |
| **GBA Advance Wars** | `0x04` | Advance Wars 1 + 2 |
| **GBA e-Reader** | `0x05` | Nintendo e-Reader emulation |

---

## Hardware Required

* **USB Adapter:** Assembled v2 hardware from Crowd Supply or DIY PCBs.

Crowd Supply: https://www.crowdsupply.com/gblink/gblink-usb-v2


Existing open source PCB designs

Pico PCB v1.1: https://github.com/GB-Link/GB-Link-USB-DIY (Recommended, adds optional physical link port and better cable compatibility)


Others:

Pico PCB v1: https://github.com/agtbaskara/game-boy-pico-link-board

Pico Zero PCB: https://github.com/weimanc/game-boy-zero-link-board

### Wiring Guide

The firmware uses the RP2040 PIO to communicate with the Game Boy. Connect the Link Cable wires as follows:

| Game Boy Signal | RP2040 Pin |
|:---|:---|
| **SCK** (Clock) | **GP0** |
| **SIN** (Data In) | **GP1** |
| **SOUT** (Data Out) | **GP2** |
| **SD** (Chip Select) | **GP3** (GBA cable) / **GP4** (GBC cable) |
| **GND** | **Ground** |

Additional hardware pins:
|Voltage | Pin | Usage|
|:---|:---|:---|
| **Voltage 3.3V** | **GP11** | Pull low for 3.3V |
| **Voltage 5V** | **GP12** | Pull low for 5V |
| **WS2812 LED** | **GP16** | NeoPixel status indicator |

---

## LED Status Indicators

The onboard WS2812 RGB LED (GP16) indicates the current status:

| Color | Status |
|:---|:---|
| **Red** | Power — Device powered, no data connection |
| **Green** | Connected — USB connected to host |
| **Yellow** | GBA Mode — Game Boy Advance link active |
| **Blue** | GB/GBC Mode — Game Boy or Game Boy Color link active |
| **Purple** | Printer Mode — GB Printer emulation active |
| **White** | Advance Wars Mode — Advance Wars 1 + 2 link active |
| **Light blue** | e-Reader Mode — Nintendo e-Reader emulation active |

LED color can also be set via USB command (`0x42`).

---

## Link Cable Compatibility

| Cable Type | Supported Modes | Notes |
|:---|:---|:---|
| **OEM GBA Link Cable** | GBA modes only | SD on GP3. Select **GBA cable** (or Auto detect) in the launcher. |
| **Generic GBC Link Cable** | All GBA and GB modes | All 6 pins wired through. SD routes to GP4 in GBA mode. Default selection. |

**GBA link cable connectors are not identical: Cheap 3rd party GBA cables missing the hub typically only have 4 conductors and are missing a ground which results in a poor connection.**

- **Slim** connector = **Master**
- **Wide** connector = **Slave**

The device **must** be connected to the **master connector** for GBA modes.

> **Tip:** With **Auto detect** selected, connect the cable to the adapter before selecting a GBA mode in the web app — the cable type is sampled at mode selection time.

---

## USB Command Protocol

Commands are sent over the WebUSB command endpoint:

| Range | Module | Commands |
|:---|:---|:---|
| `0x00–0x0F` | Control | `0x00` SetMode, `0x01` Cancel |
| `0x10–0x1F` | GBA Link | SetModeMaster, SetModeSlave, StartHandshake, ConnectLink |
| `0x20–0x2F` | GBA Emu | *(internal section commands)* |
| `0x30–0x3F` | GB Link | `0x30` SetTimingConfig, `0x31` EnterPrinter, `0x32` ExitPrinter |
| `0x40–0x4F` | Hardware | `0x40` Voltage3V3, `0x41` Voltage5V, `0x42` SetLEDColor, `0x43` RebootBootloader, `0x44` SetWebUsbLanding, `0x45` GetLedConfig, `0x46` SetModeLedColor, `0x47` ResetLedColors, `0x48` Reboot, `0x49` SetCableOverride, `0x4a` GetCableType, `0x4b` SetCableSelection |
| `0x44` | SetWebUsbLanding | (`data[1]`: 1 = on, 0 = off) persists whether the adapter advertises the **launcher.gblink.io** WebUSB landing page (the browser "open site" prompt on connect). It's stored in flash and applies on the next reconnect. The current state is reported as a 5th byte in the `0x0F` GetFirmwareInfo response. |
| `0x45` | GetLedConfig | returns the persisted per-mode LED colours: `[0x45, count, r,g,b …]` (slots: 0 idle, 1 GBA/Celio, 2 GB/GBC, 3 printer, 4 Advance Wars, 5 e-Reader). |
| `0x46` | SetModeLedColor | (`[0x46, slot, r, g, b]`) persists a mode's colour (applied on next entry to that mode |
|`0x42` | SetLEDColor | sets the LED live for previewing). |
| `0x47` | ResetLedColors | restores all slots to the built-in defaults. Colours are full 0–255 RGB — on a WS2812 the magnitude is also the brightness. |
| `0x49` | SetCableOverride | (`[0x49, 0 auto / 1 GBA / 2 GBC]`) session-only override of the cable/SD-pin path for GBA modes; the persisted selection is re-applied at boot. |
| `0x4a` | GetCableType | returns `[0x4a, 0 GBA / 1 GBC]` — the cable path currently in effect: the forced selection/override if one is set, else the cable sampled at mode entry. |
| `0x4b` | SetCableSelection | (`[0x4b, 0 auto / 1 GBA / 2 GBC]`) persists the cable selection in flash and applies it to the session (takes effect at the next link (re)configure). Fresh installs default to GBC (`2`); settings saved by firmware ≤ v2.2.2 load as auto (`0`), and the launcher also persists auto once when updating such an adapter. Reported as the 6th byte of the `0x0F` GetFirmwareInfo response. |

---

# How to bring GBAs online 

The serial interface of the Game Boy Advance supports several modes for transmitting data to other Game Boy systems. This firmware has been designed to emulate **SIO Multi-Player Mode**.

In this mode, there is one **Leader** and up to three **Followers**. Under normal circumstances, the Leader is determined by the plug used on a Game Boy Advance Link Cable. One of the plugs pulls a specific pin to ground, signaling to the Game Boy Advance that it should act as the Leader.

The firmware allows the connected Game Boy Advance to be configured dynamically as either a Leader or a Follower. This means that a third party—in this case, the server—can switch the connected Game Boy Advance into whichever role is currently required.

When it comes to transmitting actual game data, the Game Boy systems must adhere to very precise timing requirements. As a result, connecting two Game Boy Advances directly over the Internet is impossible. However, the firmware keeps the Game Boys in an idle communication loop, making each device appear to be connected to its communication partners. This approach decouples the game data from the strict timing requirements.

The firmware can then relay data received from the Game Boy Advance to the server and forward data received from the server back to the Game Boy Advance.

While this technique is not a game-agnostic solution and will not work for all game genres, it is particularly well suited for turn-based strategy games. Although the game protocols still require a significant amount of reverse engineering, they can operate quite reliably once this process has been completed.

**Currently Supported Games**:

- Pokémon Ruby  
- Pokémon Sapphire  
- Pokémon FireRed  
- Pokémon LeafGreen  
- Pokémon Emerald
- Advance Wars 1
- Advance Wars 2

---

## Zephyr RTOS

This project uses **Zephyr RTOS**:

https://www.zephyrproject.org/

Originally, the target MCU of the Celio project was a STM32F07.
Thanks to Zephyr, migrating to the RP2040 required only minimal to moderate effort.

Zephyr provides most low-level abstractions.  
The only MCU-specific implementation resides in: ```/src/layers/linkLayer_.c```   
This file implements the low-level bit-layer handling of the link protocol.

To port this project to another MCU:

1. Ensure the MCU is supported by Zephyr.
2. Implement a new `linkLayer`.
3. Provide a master clock implementation.

The remaining firmware is largely MCU-agnostic.

Currently built against:

- **Zephyr 3.7.99**

Newer versions should also be compatible.

---

## Troubleshooting

- When refreshing the web page, the USB device may need to be reset (unplug or press reset button).
- On Linux, run `./scripts/setup-permissions.sh` once to install udev rules so the browser can access the adapter without sudo (covers both WebUSB and WebSerial).
- In rare cases, the firmware may not respond to a mode-switch command — reboot the device.
- If you are forking this repo: To avoid damaging the board do not pull both GP11 and GP12 low at the same time
