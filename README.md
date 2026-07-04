# GBLink Firmware — RP2040 Game Boy Link Adapter

This firmware turns an RP2040 into a USB-to-Game Boy Link Cable adapter supporting both **Game Boy (Gen 1/2)** and **Game Boy Advance** communication over WebUSB.

This is originally forked from Celio Firmware from the CelioLink project and adds the functionality from GBLink Reconfigurable to allow for Tetris, Pokemon gen1-3 trading. Sending multiboot files to GBA, GB Printer emulator, as well as the Celio Gen3 modes for direct gen3 GBA link.

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
| **GBA Advance Wars** | `0x04` | Advance Wars, Sill WIP |

---

## Hardware Required

* **USB Adapter Board:** Raspberry Pi Pico or Waveshare RP2040-Zero, Solderless board in development. Pre built DIY adapter or kits are avalible on Etsy. https://www.etsy.com/listing/1517956485/gb-link-usb-to-gameboy-link-adapter-for 

Existing open source PCB designs

Pico PCB: https://github.com/agtbaskara/game-boy-pico-link-board

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

| **Voltage 3.3V** | **GP11** | Pull low for 3.3V 
| **Voltage 5V** | **GP12** | Pull low for 5V
| **WS2812 LED** | **GP16** | NeoPixel status indicator

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

LED color can also be set via USB command (`0x42`).

---

## Link Cable Compatibility

The firmware automatically detects which cable type is connected at mode selection time.

| Cable Type | Supported Modes | Notes |
|:---|:---|:---|
| **OEM GBA Link Cable** | GBA modes only | SD on GP3. |
| **Generic GBC Link Cable** | All GBA and GB modes | All 6 pins wired through. SD routes to GP4 in GBA mode. Auto-detected. |

**GBA link cable connectors are not identical: Cheap 3rd party GBA cables missing the hub typically only have 4 conductors and are missing a ground which results in a poor connection.**

- **Slim** connector = **Master**
- **Wide** connector = **Slave**

The device **must** be connected to the **master connector** for GBA modes.

> **Tip:** Connect the cable to the adapter before selecting a GBA mode in the web app. The firmware detects the cable type at mode selection time.

---

## USB Command Protocol

Commands are sent over the WebUSB command endpoint:

| Range | Module | Commands |
|:---|:---|:---|
| `0x00–0x0F` | Control | `0x00` SetMode, `0x01` Cancel |
| `0x10–0x1F` | GBA Link | SetModeMaster, SetModeSlave, StartHandshake, ConnectLink |
| `0x20–0x2F` | GBA Emu | *(internal section commands)* |
| `0x30–0x3F` | GB Link | `0x30` SetTimingConfig, `0x31` EnterPrinter, `0x32` ExitPrinter |
| `0x40–0x4F` | Hardware | `0x40` Voltage3V3, `0x41` Voltage5V, `0x42` SetLEDColor, `0x43` RebootBootloader, `0x44` SetWebUsbLanding, `0x45` GetLedConfig, `0x46` SetModeLedColor, `0x47` ResetLedColors |
| `0x44` | SetWebUsbLanding | (`data[1]`: 1 = on, 0 = off) persists whether the adapter advertises the **launcher.gblink.io** WebUSB landing page (the browser "open site" prompt on connect). It's stored in flash and applies on the next reconnect. The current state is reported as a 5th byte in the `0x0F` GetFirmwareInfo response. |

| `0x45` | GetLedConfig | returns the persisted per-mode LED colours: `[0x45, count, r,g,b …]` (slots: 0 idle, 1 GBA/Celio, 2 GB/GBC, 3 printer, 4 Advance Wars). |
| `0x46` | SetModeLedColor | (`[0x46, slot, r, g, b]`) persists a mode's colour (applied on next entry to that mode |
|`0x42` | SetLEDColor | sets the LED live for previewing). |
| `0x47` | ResetLedColors | restores all slots to the built-in defaults. Colours are full 0–255 RGB — on a WS2812 the magnitude is also the brightness. |

---

## Troubleshooting

- When refreshing the web page, the USB device may need to be reset (unplug or press reset button).
- On Linux, run `./scripts/setup-permissions.sh` once to install udev rules so the browser can access the adapter without sudo (covers both WebUSB and WebSerial).
- In rare cases, the firmware may not respond to a mode-switch command — reboot the device.
- If you are forking this repo: To avoid damaging the board do not pull both GP11 and GP12 low at the same time
