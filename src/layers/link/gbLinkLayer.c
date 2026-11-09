#include "gbLinkLayer.h"

#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

// --- Pin Definitions (same as GBA link) ---
#define PIN_SCK  0
#define PIN_SIN  1
#define PIN_SOUT 2
#define PIN_SD   3

// --- GB SPI PIO Program (spi_cpha1, 3 instructions) ---
// Ported from gb-link-firmware-reconfigurable/pio/spi.pio
// Clock phase = 1: data transitions on leading edge, captured on trailing edge.
//   out x, 1    side 0     ; Stall here on empty (keep SCK deasserted)
//   mov pins, x side 1 [1] ; Output data, assert SCK
//   in pins, 1  side 0     ; Input data, deassert SCK
static const uint16_t gb_spi_program_instructions[] = {
    0x6021, //  out x, 1       side 0  (0x6001 was wrong: that encodes 'out pins, 1')
    0xb101, //  mov pins, x    side 1 [1]
    0x4001, //  in pins, 1     side 0
};

static const struct pio_program gb_spi_program = {
    .instructions = gb_spi_program_instructions,
    .length = 3,
    .origin = -1,
};

// Use pio0 (shared with GBA link — only one active at a time)
static PIO g_gb_pio = NULL;
static uint g_gb_sm = 0;
static bool g_gb_initialized = false;

void gb_link_init(void)
{
    if (g_gb_initialized) return;

#if defined(CONFIG_GB_LINK_DRIVE_SD_LOW)
    // Drive SD (pin 4) LOW to signal to the GBA that an external clock master
    // is present (SIOCNT bit 3 = SD state). Required for generic GBC cables
    // where pin 4 is physically wired through to the GBA. OEM GBC cables
    // leave pin 4 unconnected so this has no effect on them.
    gpio_init(PIN_SD);
    gpio_set_dir(PIN_SD, GPIO_OUT);
    gpio_put(PIN_SD, 0);
#endif

    // Get pio0 from device tree (same PIO block as GBA link)
    const struct device* dev = DEVICE_DT_GET(DT_PROP(DT_NODELABEL(pio_link), pio));

    g_gb_pio = pio_rpi_pico_get_pio(dev);

    // Note: SM already allocated by GBA link_init at boot.
    // We reuse the same SM (0) — GBA link must be DISABLED before calling this.
    g_gb_sm = 0;

    // Clear any existing PIO program (GBA link programs)
    pio_sm_set_enabled(g_gb_pio, g_gb_sm, false);
    pio_clear_instruction_memory(g_gb_pio);
    pio_sm_restart(g_gb_pio, g_gb_sm);
    pio_sm_clear_fifos(g_gb_pio, g_gb_sm);

    // Load GB SPI program
    uint32_t offset = pio_add_program(g_gb_pio, &gb_spi_program);

    // Configure state machine for GB 8-bit SPI
    // Must match reconfigurable firmware: pio_spi_init(pio, sm, offs, 8, 4058.838/128, cpha=1, cpol=1, PIN_SCK, PIN_SOUT, PIN_SIN)
    pio_sm_config c = pio_get_default_sm_config();

    // MOSI = PIN_SOUT (pin 2), MISO = PIN_SIN (pin 1)
    sm_config_set_out_pins(&c, PIN_SOUT, 1);
    sm_config_set_in_pins(&c, PIN_SIN);
    // SCK is sideset
    sm_config_set_sideset_pins(&c, PIN_SCK);
    sm_config_set_sideset(&c, 1, false, false);

    // MSB-first, autopush/autopull at 8 bits
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_in_shift(&c, false, true, 8);

    // Clock divider: 4058.838/128 ≈ 31.71 (matching reconfigurable firmware)
    sm_config_set_clkdiv(&c, 4058.838f / 128.0f);

    sm_config_set_wrap(&c, offset, offset + 2);

    // SCK and MOSI are outputs, MISO is input
    pio_sm_set_pins_with_mask(g_gb_pio, g_gb_sm, 0,
        (1u << PIN_SCK) | (1u << PIN_SOUT));
    pio_sm_set_pindirs_with_mask(g_gb_pio, g_gb_sm,
        (1u << PIN_SCK) | (1u << PIN_SOUT),
        (1u << PIN_SCK) | (1u << PIN_SOUT) | (1u << PIN_SIN));

    // Must map the OUT pin direction to the state machine properly
    pio_sm_set_consecutive_pindirs(g_gb_pio, g_gb_sm, PIN_SOUT, 1, true);

    // Init GPIO pins for PIO
    pio_gpio_init(g_gb_pio, PIN_SOUT);
    pio_gpio_init(g_gb_pio, PIN_SIN);
    pio_gpio_init(g_gb_pio, PIN_SCK);

    // CPOL=1: invert SCK output so idle state is HIGH (matches reconfigurable firmware)
    gpio_set_outover(PIN_SCK, GPIO_OVERRIDE_INVERT);

    // Bypass input synchronizer for lower latency on MISO
    hw_set_bits(&g_gb_pio->input_sync_bypass, 1u << PIN_SIN);

    pio_sm_init(g_gb_pio, g_gb_sm, offset, &c);
    pio_sm_set_enabled(g_gb_pio, g_gb_sm, true);

    g_gb_initialized = true;
}

void gb_link_deinit(void)
{
    if (!g_gb_initialized) return;

    pio_sm_set_enabled(g_gb_pio, g_gb_sm, false);
    pio_clear_instruction_memory(g_gb_pio);
    pio_sm_restart(g_gb_pio, g_gb_sm);
    pio_sm_clear_fifos(g_gb_pio, g_gb_sm);

    // Restore SCK output override to normal (was set to INVERT for CPOL=1).
    // Without this, GBA PIO link programs see inverted SCK after GB mode.
    gpio_set_outover(PIN_SCK, GPIO_OVERRIDE_NORMAL);

    // Clear input sync bypass (was set for lower MISO latency)
    hw_clear_bits(&g_gb_pio->input_sync_bypass, 1u << PIN_SIN);

#if defined(CONFIG_GB_LINK_DRIVE_SD_LOW)
    // Release SD back to input with pull-up (restores the pinctrl default
    // state so GBA_LINK and GBA_TRADE_EMU modes can take over the pin via PIO)
    gpio_init(PIN_SD);
    gpio_pull_up(PIN_SD);
#endif

    g_gb_initialized = false;
}

void gb_link_transfer(const uint8_t* tx, uint8_t* rx, uint32_t len, uint32_t us_between)
{
    if (!g_gb_initialized) return;

    // Ported from pio_spi_write8_read8_blocking in gb-link-firmware-reconfigurable
    // Process one byte at a time with configurable inter-byte delay
    io_rw_8 *txfifo = (io_rw_8 *)&g_gb_pio->txf[g_gb_sm];
    io_rw_8 *rxfifo = (io_rw_8 *)&g_gb_pio->rxf[g_gb_sm];

    for (uint32_t i = 0; i < len; i++) {
        // Full-duplex: write TX byte, read RX byte
        size_t tx_remain = 1, rx_remain = 1;
        const uint8_t* src = &tx[i];
        uint8_t* dst = &rx[i];

        while (tx_remain || rx_remain) {
            if (tx_remain && !pio_sm_is_tx_fifo_full(g_gb_pio, g_gb_sm)) {
                *txfifo = *src;
                --tx_remain;
            }
            if (rx_remain && !pio_sm_is_rx_fifo_empty(g_gb_pio, g_gb_sm)) {
                *dst = *rxfifo;
                --rx_remain;
            }
        }

        // Inter-byte delay (configurable from web app)
        if (us_between > 0 && i < len - 1) {
            busy_wait_us(us_between);
        }
    }
}
