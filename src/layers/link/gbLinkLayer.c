#include "gbLinkLayer.h"

#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>

// --- Pin Definitions (same as GBA link) ---
#define PIN_SCK  0
#define PIN_SIN  1
#define PIN_SOUT 2
#define PIN_SD   3

// --- GB SPI PIO Program (spi_cpha1, 3 instructions) ---
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

// --- GB passive PIO Program: the Game Boy clocks (pio_firmware/gb/gb_link_passive.pio) ---
static const uint16_t gb_passive_program_instructions[] = {
            //     .wrap_target
    0x2400, //  0: wait   0 gpio, 0              [4]
    0x8080, //  1: pull   noblock
    0xa022, //  2: mov    x, y
    0x6001, //  3: out    pins, 1
    0x2480, //  4: wait   1 gpio, 0              [4]
    0x4001, //  5: in     pins, 1
    0x2400, //  6: wait   0 gpio, 0              [4]
    0x6001, //  7: out    pins, 1
    0x2480, //  8: wait   1 gpio, 0              [4]
    0x0045, //  9: jmp    x--, 5
    0x4001, // 10: in     pins, 1
    0x8000, // 11: push   noblock
    0xc002, // 12: irq    nowait 2
            //     .wrap
};

static const struct pio_program gb_passive_program = {
    .instructions = gb_passive_program_instructions,
    .length = 13,
    .origin = -1,
};

#define GB_PASSIVE_WRAP_TARGET 0
#define GB_PASSIVE_WRAP 12
#define GB_PASSIVE_BITS 8
#define GB_PASSIVE_CLKDIV 3

// --- GB hybrid clock PIO Program: the adapter clocks on its turn (pio_firmware/gb/gb_link_hybrid_clock.pio) ---
static const uint16_t gb_hybrid_clock_program_instructions[] = {
    0x80a0, //  0: pull   block
    0x6038, //  1: out    x, 24
    0x1f42, //  2: jmp    x--, 2                 [31]
    0xe001, //  3: set    pins, 1
    0xe081, //  4: set    pindirs, 1
    0xe027, //  5: set    x, 7
    0x6001, //  6: out    pins, 1
    0xf300, //  7: set    pins, 0                [19]
    0x4001, //  8: in     pins, 1
    0xf301, //  9: set    pins, 1                [19]
    0x0046, // 10: jmp    x--, 6
    0xe080, // 11: set    pindirs, 0
    0x8000, // 12: push   noblock
    0xc002, // 13: irq    nowait 2
    0x000e, // 14: jmp    14
};

static const struct pio_program gb_hybrid_clock_program = {
    .instructions = gb_hybrid_clock_program_instructions,
    .length = 15,
    .origin = -1,
};

// One pass of the hybrid clock program's wait loop takes 32 PIO cycles.
#define GB_HYBRID_CYCLES_PER_PASS 32
#define GB_HYBRID_MAX_GAP_US 1000000u

// The passive program raises PIO IRQ flag 2 after each byte, on the pio0 line the GBA link does not use.
// So does the hybrid clock program.
#define GB_PASSIVE_IRQ_FLAG 2
#define GB_PASSIVE_IRQ_SOURCE pis_interrupt2

// Use pio0 (shared with GBA link — only one active at a time)
static PIO g_gb_pio = NULL;
static uint g_gb_sm = 0;
static bool g_gb_initialized = false;

static pio_sm_config g_spi_config;
static uint g_spi_offset = 0;
static pio_sm_config g_passive_config;
static uint g_passive_offset = 0;
static pio_sm_config g_hybrid_config;
static uint g_hybrid_clock_offset = 0;

enum gb_link_mode { GB_MODE_ACTIVE, GB_MODE_PASSIVE, GB_MODE_HYBRID };

static volatile enum gb_link_mode g_mode = GB_MODE_ACTIVE;
static gb_link_passive_handler_t g_passive_handler = NULL;
static volatile uint8_t g_passive_next = 0xFF;
static volatile uint32_t g_passive_bytes = 0;
static uint32_t g_stall_bytes = 0;
static int64_t g_stall_since = 0;

static gb_link_hybrid_handler_t g_hybrid_handler = NULL;
static volatile bool g_hybrid_adapter_turn = false;   // the adapter's turn: it clocks the byte in progress
static uint32_t g_hybrid_gap_us = 0;
static volatile uint32_t g_hybrid_last_end = 0;   // µs timer when the last byte ended
static uint32_t g_hybrid_passes_per_us_q10 = 0;   // wait loop passes per µs, times 1024

static void start_spi(void)
{
    // Init GPIO pins for PIO. This clears the pad overrides, so it comes first, and SCK is inverted
    // before it is driven: taking SCK over from the Game Boy is then not a clock edge.
    pio_gpio_init(g_gb_pio, PIN_SOUT);
    pio_gpio_init(g_gb_pio, PIN_SIN);
    pio_gpio_init(g_gb_pio, PIN_SCK);

    // CPOL=1: invert SCK output so idle state is HIGH
    gpio_set_outover(PIN_SCK, GPIO_OVERRIDE_INVERT);

    // SCK and MOSI are outputs, MISO is input
    pio_sm_set_pins_with_mask(g_gb_pio, g_gb_sm, 0,
        (1u << PIN_SCK) | (1u << PIN_SOUT));
    pio_sm_set_pindirs_with_mask(g_gb_pio, g_gb_sm,
        (1u << PIN_SCK) | (1u << PIN_SOUT),
        (1u << PIN_SCK) | (1u << PIN_SOUT) | (1u << PIN_SIN));

    // Must map the OUT pin direction to the state machine properly
    pio_sm_set_consecutive_pindirs(g_gb_pio, g_gb_sm, PIN_SOUT, 1, true);

    // Bypass input synchronizer for lower latency on MISO
    hw_set_bits(&g_gb_pio->input_sync_bypass, 1u << PIN_SIN);

    pio_sm_init(g_gb_pio, g_gb_sm, g_spi_offset, &g_spi_config);
    pio_sm_set_enabled(g_gb_pio, g_gb_sm, true);
}

// The pull-up holds SCK high once it is let go, and the override only changes after that, so
// handing SCK to the Game Boy is not a clock edge either.
static void release_sck(void)
{
    pio_sm_set_consecutive_pindirs(g_gb_pio, g_gb_sm, PIN_SCK, 1, false);
    gpio_set_outover(PIN_SCK, GPIO_OVERRIDE_NORMAL);
}

static void stop_sm(void)
{
    pio_set_irq1_source_enabled(g_gb_pio, GB_PASSIVE_IRQ_SOURCE, false);
    pio_sm_set_enabled(g_gb_pio, g_gb_sm, false);
    pio_sm_clear_fifos(g_gb_pio, g_gb_sm);
    pio_sm_restart(g_gb_pio, g_gb_sm);
    pio_interrupt_clear(g_gb_pio, GB_PASSIVE_IRQ_FLAG);
}

static void start_passive(uint8_t first)
{
    release_sck();
    pio_sm_set_consecutive_pindirs(g_gb_pio, g_gb_sm, PIN_SIN, 1, false);
    pio_sm_set_consecutive_pindirs(g_gb_pio, g_gb_sm, PIN_SOUT, 1, true);

    pio_sm_init(g_gb_pio, g_gb_sm, g_passive_offset, &g_passive_config);
    pio_sm_exec(g_gb_pio, g_gb_sm, pio_encode_set(pio_y, GB_PASSIVE_BITS - 2));

    g_passive_next = first;
    pio_sm_put(g_gb_pio, g_gb_sm, (uint32_t)first << (32 - GB_PASSIVE_BITS));
    g_stall_bytes = g_passive_bytes;
    g_stall_since = k_uptime_get();

    pio_set_irq1_source_enabled(g_gb_pio, GB_PASSIVE_IRQ_SOURCE, true);
    pio_sm_set_enabled(g_gb_pio, g_gb_sm, true);
}

// Hybrid: the Game Boy's turns run on the passive program, the adapter's on the hybrid clock program.
// The interrupt moves the state machine from one to the other after every byte.
static void start_hybrid(bool game_boy_first, uint8_t first)
{
    release_sck();
    pio_sm_set_consecutive_pindirs(g_gb_pio, g_gb_sm, PIN_SIN, 1, false);
    pio_sm_set_consecutive_pindirs(g_gb_pio, g_gb_sm, PIN_SOUT, 1, true);

    pio_sm_init(g_gb_pio, g_gb_sm, game_boy_first ? g_passive_offset : g_hybrid_clock_offset, &g_hybrid_config);
    pio_sm_exec(g_gb_pio, g_gb_sm, pio_encode_set(pio_y, GB_PASSIVE_BITS - 2));

    g_hybrid_adapter_turn = !game_boy_first;
    g_hybrid_last_end = time_us_32();
    g_passive_next = first;
    if (game_boy_first) pio_sm_put(g_gb_pio, g_gb_sm, (uint32_t)first << (32 - GB_PASSIVE_BITS));
    g_stall_bytes = g_passive_bytes;
    g_stall_since = k_uptime_get();

    pio_set_irq1_source_enabled(g_gb_pio, GB_PASSIVE_IRQ_SOURCE, true);
    pio_sm_set_enabled(g_gb_pio, g_gb_sm, true);
}

// The hybrid clock program's TX word: the wait that is left of the gap since the last byte ended,
// then the byte.
static uint32_t hybrid_clock_word(uint8_t byte)
{
    const uint32_t since = time_us_32() - g_hybrid_last_end;
    const uint32_t wait_us = (since < g_hybrid_gap_us) ? g_hybrid_gap_us - since : 0;
    const uint32_t passes = (wait_us * g_hybrid_passes_per_us_q10) >> 10;
    return (passes << GB_PASSIVE_BITS) | byte;
}

static void hybrid_byte_done(uint8_t received)
{
    const bool clocked = g_hybrid_adapter_turn;
    g_hybrid_last_end = time_us_32();
    const int next = g_hybrid_handler ? g_hybrid_handler(received, clocked) : -1;

    pio_sm_clear_fifos(g_gb_pio, g_gb_sm);
    if (clocked)
    {
        // The Game Boy's turn. The answer goes in before the jump, so it is there for the first edge.
        g_hybrid_adapter_turn = false;
        g_passive_next = (next >= 0) ? (uint8_t)next : 0xFF;
        pio_sm_put(g_gb_pio, g_gb_sm, (uint32_t)g_passive_next << (32 - GB_PASSIVE_BITS));
        pio_sm_exec(g_gb_pio, g_gb_sm, pio_encode_jmp(g_passive_offset + GB_PASSIVE_WRAP_TARGET));
    }
    else
    {
        // The adapter's turn. With nothing to send yet, the program waits for gb_link_hybrid_clock().
        g_hybrid_adapter_turn = true;
        pio_sm_exec(g_gb_pio, g_gb_sm, pio_encode_jmp(g_hybrid_clock_offset));
        if (next >= 0) pio_sm_put(g_gb_pio, g_gb_sm, hybrid_clock_word((uint8_t)next));
    }
}

static void gb_passive_isr(const void* arg)
{
    ARG_UNUSED(arg);

    if (!pio_interrupt_get(g_gb_pio, GB_PASSIVE_IRQ_FLAG)) return;
    pio_interrupt_clear(g_gb_pio, GB_PASSIVE_IRQ_FLAG);
    if (pio_sm_is_rx_fifo_empty(g_gb_pio, g_gb_sm)) return;

    const uint8_t received = (uint8_t)pio_sm_get(g_gb_pio, g_gb_sm);
    g_passive_bytes++;

    if (g_mode == GB_MODE_HYBRID)
    {
        hybrid_byte_done(received);
        return;
    }

    const uint8_t next = g_passive_handler ? g_passive_handler(received) : 0xFF;

    // Only the newest byte may wait for the next transfer.
    g_passive_next = next;
    pio_sm_clear_fifos(g_gb_pio, g_gb_sm);
    pio_sm_put(g_gb_pio, g_gb_sm, (uint32_t)next << (32 - GB_PASSIVE_BITS));
}

static int gb_link_irq_init(void)
{
    // The line stays enabled; the passive program's flag is routed to it only while that program runs.
    IRQ_CONNECT(PIO0_IRQ_1, 0, gb_passive_isr, NULL, 0);
    irq_enable(PIO0_IRQ_1);
    return 0;
}

SYS_INIT(gb_link_irq_init, APPLICATION, 2);

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
    g_spi_offset = pio_add_program(g_gb_pio, &gb_spi_program);

    // Configure state machine for GB 8-bit SPI
    // 8 bits per transfer, CPHA=1, CPOL=1
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

    // Clock divider: 4058.838/128 ≈ 31.71
    sm_config_set_clkdiv(&c, 4058.838f / 128.0f);

    sm_config_set_wrap(&c, g_spi_offset, g_spi_offset + 2);
    g_spi_config = c;

    // The passive program: SCK in, SIN in, SOUT out, MSB first, no autopush/autopull.
    g_passive_offset = pio_add_program(g_gb_pio, &gb_passive_program);
    pio_sm_config p = pio_get_default_sm_config();
    sm_config_set_in_pins(&p, PIN_SIN);
    sm_config_set_in_shift(&p, false, false, 32);
    sm_config_set_out_pins(&p, PIN_SOUT, 1);
    sm_config_set_out_shift(&p, false, false, 32);
    sm_config_set_clkdiv(&p, GB_PASSIVE_CLKDIV);
    sm_config_set_wrap(&p, g_passive_offset + GB_PASSIVE_WRAP_TARGET, g_passive_offset + GB_PASSIVE_WRAP);
    g_passive_config = p;

    // Hybrid: the passive program's settings, and SCK as the set pin of the hybrid clock program,
    // which jumps where it needs to and so leaves the passive program's wrap alone.
    g_hybrid_clock_offset = pio_add_program(g_gb_pio, &gb_hybrid_clock_program);
    pio_sm_config t = p;
    sm_config_set_set_pins(&t, PIN_SCK, 1);
    g_hybrid_config = t;
    g_hybrid_passes_per_us_q10 = (uint32_t)(((uint64_t)clock_get_hz(clk_sys) * 1024u)
        / ((uint64_t)GB_PASSIVE_CLKDIV * GB_HYBRID_CYCLES_PER_PASS * 1000000u));

    g_mode = GB_MODE_ACTIVE;
    start_spi();

    g_gb_initialized = true;
}

void gb_link_deinit(void)
{
    if (!g_gb_initialized) return;

    stop_sm();
    g_mode = GB_MODE_ACTIVE;
    pio_clear_instruction_memory(g_gb_pio);

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
    if (!g_gb_initialized || g_mode != GB_MODE_ACTIVE) return;

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

void gb_link_set_passive_handler(gb_link_passive_handler_t handler)
{
    g_passive_handler = handler;
}

void gb_link_set_passive(bool passive, uint8_t first)
{
    const enum gb_link_mode mode = passive ? GB_MODE_PASSIVE : GB_MODE_ACTIVE;
    if (!g_gb_initialized || mode == g_mode) return;

    stop_sm();
    g_mode = mode;
    if (passive) start_passive(first);
    else start_spi();
}

void gb_link_restart_passive(uint8_t first)
{
    if (!g_gb_initialized) return;

    stop_sm();
    g_mode = GB_MODE_PASSIVE;
    start_passive(first);
}

void gb_link_set_hybrid_handler(gb_link_hybrid_handler_t handler)
{
    g_hybrid_handler = handler;
}

void gb_link_start_hybrid(uint32_t gap_us, bool game_boy_first, uint8_t first)
{
    if (!g_gb_initialized) return;

    stop_sm();
    g_mode = GB_MODE_HYBRID;
    g_hybrid_gap_us = (gap_us < GB_HYBRID_MAX_GAP_US) ? gap_us : GB_HYBRID_MAX_GAP_US;
    start_hybrid(game_boy_first, first);
}

void gb_link_hybrid_clock(uint8_t byte)
{
    if (!g_gb_initialized || g_mode != GB_MODE_HYBRID) return;

    pio_sm_put(g_gb_pio, g_gb_sm, hybrid_clock_word(byte));
}

void gb_link_check_passive_stall(uint32_t stall_ms)
{
    if (!g_gb_initialized || g_mode == GB_MODE_ACTIVE) return;

    // Waiting for the Game Boy's next byte, or in hybrid on the adapter's turn, nothing can be stuck.
    const uint pc = pio_sm_get_pc(g_gb_pio, g_gb_sm);
    const bool idle = pc == g_passive_offset + GB_PASSIVE_WRAP_TARGET
        || (g_mode == GB_MODE_HYBRID && pc >= g_hybrid_clock_offset
            && pc < g_hybrid_clock_offset + gb_hybrid_clock_program.length);
    const uint32_t bytes = g_passive_bytes;
    const int64_t now = k_uptime_get();

    if (idle || bytes != g_stall_bytes)
    {
        g_stall_bytes = bytes;
        g_stall_since = now;
        return;
    }

    if (now - g_stall_since >= stall_ms)
    {
        stop_sm();
        if (g_mode == GB_MODE_HYBRID) start_hybrid(true, g_passive_next);
        else start_passive(g_passive_next);
    }
}
