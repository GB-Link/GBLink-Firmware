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
| **GBA Link** | `0x01` | Link 2 to 4 GBA systems over the internet |
| **GB Link** | `0x02` | Game Boy link: the adapter clocks the Game Boy (active), the Game Boy clocks the adapter (passive), or they take turns (hybrid); see [GB Link Mode](#gb-link-mode) |
| **GB Printer** | `0x03` | Game Boy Printer emulation |
| **GBA Advance Wars** | `0x04` | Advance Wars 1 + 2 |
| **GBA e-Reader** | `0x05` | Nintendo e-Reader emulation. SetMode third byte: `1` SMA4 US, `2` Pokémon RS US, `3` SMA4 JPN, `4` Pokémon RS JPN, `5`–`9` SMA4 EUR En/Fr/De/Es/It, `10` Pokémon Emerald US, `11` Pokémon Emerald JPN, `12` (Émeraude) / Emerald, `13` (Smaragd) / Emerald. |
| **GBA Battle Chip Gate** | `0x06` | Mega Man Battle Network Battle Chip Gate / Progress / Beast Link |
| **GBA Wireless Adapter** | `0x07` | Wireless Adapter emulation: trade and battle with FireRed/LeafGreen on Nintendo Switch through a companion ESP32 running [GB-Link Switch LDN](https://github.com/GB-Link/GB-Link-Switch-LDN) |

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
| **SD** (Chip Select) | **GP3** (link port pin 4) and **GP4** (link port pin 1), both read |
| **GND** | **Ground** |

Additional hardware pins:
|Voltage | Pin | Usage|
|:---|:---|:---|
| **Voltage 3.3V** | **GP11** | Pull low for 3.3V |
| **Voltage 5V** | **GP12** | Pull low for 5V |
| **WS2812 LED** | **GP16** | NeoPixel status indicator |
| **Bridge UART TX** | **GP8** | To the companion ESP32 (wireless adapter mode) |
| **Bridge UART RX** | **GP9** | From the companion ESP32 (wireless adapter mode) |

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
| **Deep blue** | Battle Chip Gate Mode — Battle Chip Gate link active |

LED color can also be set via USB command (`0x42`).

---

## Link Cable Compatibility

| Cable Type | Supported Modes | Notes |
|:---|:---|:---|
| **OEM GBA Link Cable** | GBA modes only | SD arrives on GP3. |
| **Generic GBC Link Cable** | All GBA and GB modes | All 6 pins wired through, with pins 1 and 4 crossed, so SD arrives on GP4. |

In GBA modes the adapter listens for SD on GP3 and GP4 at the same time and only drives the pin the Game Boy Advance answers on. In the GBA Link mode it remembers that pin until the adapter is unplugged or switched to another mode.

**GBA link cable connectors are not identical: Cheap 3rd party GBA cables missing the hub typically only have 4 conductors and are missing a ground which results in a poor connection.**

- **Slim** connector = **Master**
- **Wide** connector = **Slave**

The device **must** be connected to the **master connector** for GBA modes.

---

## USB Command Protocol

Commands are sent over the WebUSB command endpoint:

| Range | Module | Commands |
|:---|:---|:---|
| `0x00–0x0F` | Control | `0x00` SetMode, `0x01` Cancel |
| `0x10–0x1F` | GBA Link | `0x10` SetPlayerSettings (`[0x10, player count 2–4, seat]`, seat 0 is the GBA that hosts the link), `0x12` StartHandshake, `0x13` ConnectLink. Link data travels as 17-byte packets: the seat, then 8 link words. |
| `0x30–0x3F` | GB Link | `0x30` SetTimingConfig, `0x31` EnterPrinter, `0x32` ExitPrinter, `0x33`–`0x3E` the GB Link mode's settings (see [GB Link Mode](#gb-link-mode)) |
| `0x40–0x4F` | Hardware | `0x40` Voltage3V3, `0x41` Voltage5V, `0x42` SetLEDColor, `0x43` RebootBootloader, `0x44` SetWebUsbLanding, `0x45` GetLedConfig, `0x46` SetModeLedColor, `0x47` ResetLedColors, `0x48` Reboot, `0x4c` GetRfuIsrStats |
| `0x44` | SetWebUsbLanding | (`data[1]`: 1 = on, 0 = off) persists whether the adapter advertises the **launcher.gblink.io** WebUSB landing page (the browser "open site" prompt on connect). It's stored in flash and applies on the next reconnect. The current state is reported as a 5th byte in the `0x0F` GetFirmwareInfo response. |
| `0x45` | GetLedConfig | returns the persisted per-mode LED colours: `[0x45, count, r,g,b …]` (slots: 0 idle, 1 GBA/Celio, 2 GB/GBC, 3 printer, 4 Advance Wars, 5 e-Reader, 6 Battle Chip Gate). |
| `0x46` | SetModeLedColor | (`[0x46, slot, r, g, b]`) persists a mode's colour (applied on next entry to that mode |
|`0x42` | SetLEDColor | sets the LED live for previewing). |
| `0x47` | ResetLedColors | restores all slots to the built-in defaults. Colours are full 0–255 RGB — on a WS2812 the magnitude is also the brightness. |
| `0x4c` | GetRfuIsrStats | returns `[0x4c, 5 × u32 LE]` — wireless adapter transfer-path timing: worst interrupt pass (µs), passes longer than the 30 µs staging budget, tightest gap the GBA left between words (µs), passes counted, and the GPIO the last soft-reset pulse arrived on (`0xFF` if none yet). |

## GB Link Mode

The GB Link mode starts in **active** mode: the adapter clocks the Game Boy. Every data packet is clocked out to the Game Boy, and the bytes it sends back in the same transfers come back as one packet. Set the pace with `0x30` SetTimingConfig (`[0x30, µs between transfers as 3 bytes LE, bytes per transfer]`).

In **passive** mode the Game Boy clocks the adapter, which is what a game needs when it plays player 1. Data packets are queued, and each transfer the Game Boy starts is answered with the next queued byte, or with the idle byte while nothing is queued. The Game Boy's answers to queued bytes are sent back as they arrive, so the host never has to answer within a transfer.

In **hybrid** mode the Game Boy and the adapter take turns clocking, one byte each, for games where both Game Boys drive the clock. The queued bytes are used in order, whoever clocks. On its turn the adapter clocks the next one, no sooner than the set wait after the Game Boy's byte ended, and waits for the host while nothing is queued. On the Game Boy's turn it answers with the next one, or with the idle byte. As in passive mode, the Game Boy's answers to queued bytes are sent back as they arrive. Tetris DX links this way.

In all three modes the host does not have to wait for an answer before it sends more, or read an answer right away.

- In active mode the adapter holds up to 24 data packets in order, and drops newer packets while all 24 are waiting.
- In passive and hybrid mode it holds up to 1023 bytes for the Game Boy and drops bytes that arrive while all 1023 are waiting. Each byte the Game Boy sends back means one of them was used, so the host can count how many are still waiting.
- It holds up to 24 answers until the host reads them, and drops newer answers while all 24 are unread. In passive and hybrid mode, up to 1023 of the Game Boy's bytes wait for the host the same way.

The host sets the GB Link mode with these commands. They take their turn with the data packets: a command acts after the data packets sent before it and before the ones sent after it. Only the three queries answer, on the data channel, starting with the command byte. A command shorter than its payload is ignored.

| Command | Payload | Does | Answer |
|:---|:---|:---|:---|
| `30` | µs between transfers (3 bytes), bytes per transfer | the pace of active mode | |
| `33` | | active mode | |
| `34` | | passive mode; coming from another mode, nothing is queued | |
| `35` | u32 µs, first | hybrid mode with nothing queued: the adapter waits at least the µs (up to 1 s) after the Game Boy's byte before it clocks its own. First `0`: the adapter clocks the first byte; `1`: the Game Boy does | |
| `36` | byte | sets the idle byte | |
| `37` | | drops everything queued in both directions | |
| `38` | | passive or hybrid mode starts over with nothing queued, after a transfer went out of step | |
| `39` | preamble byte, waiting byte | drops everything queued and starts loading a section | |
| `3A` | | | `3A`, checksum (u32) and length (u32) of the section loaded so far |
| `3B` | | starts the section | |
| `3C` | | | `3C`, done (1 byte), the Game Boy's section bytes waiting (u32), section bytes left to send (u32) |
| `3D` | count | reads up to that many of the Game Boy's section bytes, 62 at most | `3D`, the count read, then the bytes |
| `3E` | | back to regular passive mode | |

All numbers are little-endian. The section checksum adds up the bytes, each one shifted by 8 bits times its place in a group of four.

A **section** exchanges a whole block in passive mode: load it with `39`, then data packets, check it with `3A` and start it with `3B`. The adapter answers the waiting byte until the Game Boy sends the preamble byte, answers the preamble while the Game Boy repeats it, then sends the block while keeping the Game Boy's block. Poll `3C` until it is done, read the Game Boy's block with `3D`, then return with `3E`. Pokémon Gold, Silver and Crystal exchange their data this way.

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
