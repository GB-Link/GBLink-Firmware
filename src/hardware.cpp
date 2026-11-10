#include "hardware.hpp"

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"

extern "C" {
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
}

// --- Pin Definitions ---
#define WS2812_PIN      16
#define VSWITCH_3V3_PIN 11
#define VSWITCH_5V_PIN  12

// --- WS2812 PIO Program (4 instructions) ---
static const uint16_t ws2812_program_instructions[] = {
    0x6221, //  0: out    x, 1       side 0 [2]
    0x1123, //  1: jmp    !x, 3      side 1 [1]
    0x1400, //  2: jmp    0          side 1 [4]
    0xa442, //  3: nop               side 0 [4]
};

static const struct pio_program ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length = 4,
    .origin = -1,
};

// WS2812 uses pio1 (independent from link SPI on pio0)
static PIO g_ws2812_pio = NULL;
static uint g_ws2812_sm = 0;
static bool g_ws2812_initialized = false;

// --- Hardware Singleton ---

Hardware& Hardware::getInstance()
{
    static Hardware instance;
    return instance;
}

Hardware::Hardware()
{
    // Initialize voltage switching GPIOs (raw Pico SDK, matching linkLayer_pio.c pattern)
    gpio_init(VSWITCH_3V3_PIN);
    gpio_init(VSWITCH_5V_PIN);
    gpio_set_dir(VSWITCH_3V3_PIN, GPIO_OUT);
    gpio_set_dir(VSWITCH_5V_PIN, GPIO_OUT);

    // Safe state: both HIGH first
    gpio_put(VSWITCH_3V3_PIN, 1);
    gpio_put(VSWITCH_5V_PIN, 1);
    busy_wait_us(100);

    // Default to 3.3V mode
    gpio_put(VSWITCH_3V3_PIN, 0);

    // Initialize WS2812 NeoPixel
    ws2812Init();

    // Set initial LED to red (disconnected state)
    ws2812Set(0x000500); // GRB format: G=0x00, R=0x05, B=0x00
}

void Hardware::ws2812Init()
{
    // Pico SDK defines 'pio1' as a macro (pio1_hw) which breaks DT_NODELABEL.
    // Temporarily undefine it so device tree lookup works.
    #undef pio1
    const struct device* dev = DEVICE_DT_GET(DT_NODELABEL(pio1));
    if (!device_is_ready(dev)) {
        return;
    }

    g_ws2812_pio = pio_rpi_pico_get_pio(dev);
    pio_rpi_pico_allocate_sm(dev, &g_ws2812_sm);

    uint offset = pio_add_program(g_ws2812_pio, &ws2812_program);

    // Configure state machine for WS2812 protocol
    pio_gpio_init(g_ws2812_pio, WS2812_PIN);
    pio_sm_set_consecutive_pindirs(g_ws2812_pio, g_ws2812_sm, WS2812_PIN, 1, true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, WS2812_PIN);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_wrap(&c, offset, offset + 3);
    sm_config_set_clkdiv(&c, clock_get_hz(clk_sys) / (800000.0f * 8.0f));

    pio_sm_init(g_ws2812_pio, g_ws2812_sm, offset, &c);
    pio_sm_set_enabled(g_ws2812_pio, g_ws2812_sm, true);

    g_ws2812_initialized = true;
}

void Hardware::ws2812Set(uint32_t grb)
{
    if (!g_ws2812_initialized) return;
    pio_sm_put_blocking(g_ws2812_pio, g_ws2812_sm, grb << 8u);
}

// --- Public API ---

void Hardware::setVoltage3V3()
{
    // Both HIGH first (safe off), then pull 3.3V LOW
    gpio_put(VSWITCH_3V3_PIN, 1);
    gpio_put(VSWITCH_5V_PIN, 1);
    busy_wait_us(100);
    gpio_put(VSWITCH_3V3_PIN, 0);
}

void Hardware::setVoltage5V()
{
    // Both HIGH first (safe off), then pull 5V LOW
    gpio_put(VSWITCH_3V3_PIN, 1);
    gpio_put(VSWITCH_5V_PIN, 1);
    busy_wait_us(100);
    gpio_put(VSWITCH_5V_PIN, 0);
}

void Hardware::setLED(uint8_t r, uint8_t g, uint8_t b, bool on)
{
    if (on) {
        // NeoPixel expects GRB format
        uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
        ws2812Set(grb);
    } else {
        ws2812Set(0x000000);
    }
}

// Reset into the RP2040 USB bootloader (BOOTSEL/PICOBOOT) so the host can
// reflash over WebUSB. The Zephyr hal_rpi_pico module does not ship
// pico/bootrom.h, so look up reset_usb_boot() in the bootrom directly:
// the func-table pointer lives at 0x14 and the table-lookup routine at 0x18.
void Hardware::rebootToBootloader()
{
    typedef void *(*rom_table_lookup_fn)(uint16_t *table, uint32_t code);
    typedef void (*reset_usb_boot_fn)(uint32_t gpio_mask, uint32_t disable_mask);

    uint16_t *func_table = (uint16_t *)(uintptr_t)(*(uint16_t *)0x14);
    rom_table_lookup_fn rom_table_lookup =
        (rom_table_lookup_fn)(uintptr_t)(*(uint16_t *)0x18);

    const uint32_t reset_usb_boot_code = (uint32_t)('U' | ('B' << 8));
    reset_usb_boot_fn reset_usb_boot =
        (reset_usb_boot_fn)rom_table_lookup(func_table, reset_usb_boot_code);

    reset_usb_boot(0, 0); // does not return
}

// Warm-reboot back into the application (not the bootloader). Used to "apply now"
// after settings changes — persisted LED colours are re-read at boot.
void Hardware::reboot()
{
    sys_reboot(SYS_REBOOT_COLD); // does not return
}

// --- Boot-time Initialization ---
static int hardware_init()
{
    Hardware::getInstance();
    return 0;
}

SYS_INIT(hardware_init, APPLICATION, 0);
