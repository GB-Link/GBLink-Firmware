#include "linkLayer.h"

#include <zephyr/kernel.h>

#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/regs/sio.h"

#include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>
#include <zephyr/drivers/pinctrl.h>

#define TX_RX_DONE_IRQ 0
#define TX_VALUE_IRQ 1

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

PINCTRL_DT_DEFINE(DT_NODELABEL(pio_link));

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void* g_receiveUserData = NULL;
static ReceiveHandler g_receiveCallback = NULL;

void* g_transmitUserData = NULL;
static TransmitHandler g_transmitCallback = NULL;

void* g_transiveDoneUserdata = NULL;
static TransiveDoneHandler g_transiveDoneCallback = NULL;

static enum LinkMode g_mode = SLAVE;

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

/* SET-instruction pin values.
 *   bit 0 = SC  (GP0)       bit 1 = SI  (GP1, always input)
 *   bit 2 = SO  (GP2)       bit 3/4 = SD (GP3 or GP4)       */
#define PIO_SC          1   /* bit 0 */
#define PIO_SO          4   /* bit 2 */
#define PIO_SD_GBA      8   /* bit 3 — GBA cable, SD on GP3, set_count=4 */
#define PIO_SD_GBC      16  /* bit 4 — GBC cable, SD on GP4, set_count=5 */

/* --- GBA cable programs (SD on GP3) --- */

RPI_PICO_PIO_DEFINE_PROGRAM(pio_master_gba, 0, 26,
    (0xe080 | PIO_SC | PIO_SO | PIO_SD_GBA), //  0: set    pindirs, SC|SO|SD=out
    (0xe000 | PIO_SC | PIO_SO | PIO_SD_GBA), //  1: set    pins, SC|SO|SD=HIGH
    0x0082, 0xc001, 0xe03e, 0x0245, 0x80a0, 0xa047, 0xe02f, 0x80a0,
    (0xe000 | PIO_SO | PIO_SD_GBA),          // 10: set    pins, SO|SD=HIGH
    0xef04, 0x6e01, 0x004c,
    (0xef00 | PIO_SO | PIO_SD_GBA),          // 14: set    pins, SO|SD=HIGH   [15]
    (0xe000 | PIO_SD_GBA),                   // 15: set    pins, SD=HIGH
    0xe085, 0xe03f, 0x1f53, 0x0035, 0x00d2,
    0xf62f, 0x4e01, 0x0056, 0x9020,
    (0xfe00 | PIO_SO | PIO_SD_GBA),          // 25: set    pins, SO|SD=HIGH   [30]
    0xc000);

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_gba, 0, 18,
    (0xe000 | PIO_SD_GBA), 0xe084, 0xe02f, 0x2020,
    0xd701, 0x4e01, 0x0045, 0x8020,
    (0xe000 | PIO_SD_GBA),
    (0xe080 | PIO_SO | PIO_SD_GBA),
    0xbf42, 0xe02f, 0x80a0, 0xf000, 0x6e01, 0x004e,
    (0xf000 | PIO_SD_GBA),
    0xc000, 0xbf42);

/* MULTI child: SO stays on GPIO LOW — no PIO_SO in set ops. */
RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_multi_gba, 0, 18,
    (0xe000 | PIO_SD_GBA), 0xe080, 0xe02f, 0x2020,
    0xd701, 0x4e01, 0x0045, 0x8020,
    (0xe000 | PIO_SD_GBA),
    (0xe080 | PIO_SD_GBA),
    0xbf42, 0xe02f, 0x80a0, 0xf000, 0x6e01, 0x004e,
    (0xf000 | PIO_SD_GBA),
    0xc000, 0xbf42);

/* --- GBC cable programs (SD on GP4) --- */

RPI_PICO_PIO_DEFINE_PROGRAM(pio_master_gbc, 0, 26,
    (0xe080 | PIO_SC | PIO_SO | PIO_SD_GBC), //  0: set    pindirs, SC|SO|SD=out
    (0xe000 | PIO_SC | PIO_SO | PIO_SD_GBC), //  1: set    pins, SC|SO|SD=HIGH
    0x0082, 0xc001, 0xe03e, 0x0245, 0x80a0, 0xa047, 0xe02f, 0x80a0,
    (0xe000 | PIO_SO | PIO_SD_GBC),          // 10: set    pins, SO|SD=HIGH
    0xef04, 0x6e01, 0x004c,
    (0xef00 | PIO_SO | PIO_SD_GBC),          // 14: set    pins, SO|SD=HIGH   [15]
    (0xe000 | PIO_SD_GBC),                   // 15: set    pins, SD=HIGH
    0xe085, 0xe03f, 0x1f53, 0x0035, 0x00d2,
    0xf62f, 0x4e01, 0x0056, 0x9020,
    (0xfe00 | PIO_SO | PIO_SD_GBC),          // 25: set    pins, SO|SD=HIGH   [30]
    0xc000);

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_gbc, 0, 18,
    (0xe000 | PIO_SD_GBC), 0xe084, 0xe02f, 0x2020,
    0xd701, 0x4e01, 0x0045, 0x8020,
    (0xe000 | PIO_SD_GBC),
    (0xe080 | PIO_SO | PIO_SD_GBC),
    0xbf42, 0xe02f, 0x80a0, 0xf000, 0x6e01, 0x004e,
    (0xf000 | PIO_SD_GBC),
    0xc000, 0xbf42);

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_multi_gbc, 0, 18,
    (0xe000 | PIO_SD_GBC), 0xe080, 0xe02f, 0x2020,
    0xd701, 0x4e01, 0x0045, 0x8020,
    (0xe000 | PIO_SD_GBC),
    (0xe080 | PIO_SD_GBC),
    0xbf42, 0xe02f, 0x80a0, 0xf000, 0x6e01, 0x004e,
    (0xf000 | PIO_SD_GBC),
    0xc000, 0xbf42);

/* Detect cable type by reading GP1 (SI pin).
 * GBA cable: GP1 hardwired to GND in cable — reads LOW, always
 * GBC cable: GP1 connected to GBA SO or floating — pull-up → reads HIGH
 *
 * A GBA that is already in MULTI link mode drives SO LOW, so on a GBC cable
 * an instantaneous sample can misread as a GBA cable. Sample over a window:
 * a hardwired ground can never read high, so any high sample proves GBC.
 * (A steadily driven-low SO still misreads; the host's cable-flip fallback
 * covers that case.) */
static bool detect_gbc_cable(void)
{
    gpio_set_function(1, GPIO_FUNC_SIO);
    gpio_set_dir(1, GPIO_IN);
    gpio_pull_up(1);
    for (volatile int i = 0; i < 1000; i++);  /* settle ~10us */
    bool gbc = false;
    for (volatile int i = 0; i < 200000; i++)  /* sample ~2ms */
    {
        if (gpio_get(1)) { gbc = true; break; }
    }
    gpio_set_function(1, GPIO_FUNC_PIO0);
    return gbc;
}

static bool g_gbc_cable = false;

#define CABLE_AUTO      0
#define CABLE_FORCE_GBA 1
#define CABLE_FORCE_GBC 2
static uint8_t g_cable_override = CABLE_AUTO;

static bool link_useGbcCable(void)
{
    if (g_cable_override == CABLE_FORCE_GBA) return false;
    if (g_cable_override == CABLE_FORCE_GBC) return true;
    return g_gbc_cable;
}

void link_detectCableType(void)
{
    g_gbc_cable = detect_gbc_cable();
}

uint8_t link_getDetectedCableType(void)
{
    return link_useGbcCable() ? 1 : 0;
}

void link_setCableOverride(uint8_t mode)
{
    if (mode > CABLE_FORCE_GBC) mode = CABLE_AUTO;
    g_cable_override = mode;
}

/* Force the SD path opposite to the currently effective one (wrong-pin
 * recovery). Cleared back to auto by the next SetCableOverride. */
void link_flipSdPath(void)
{
    g_cable_override = link_useGbcCable() ? CABLE_FORCE_GBA : CABLE_FORCE_GBC;
}

/* Which slave flavour is configured. The classic kind is watched by the
 * link-layer wrong-pin watchdog below; the e-reader kinds re-arm through
 * section state (packet layer, partner presence), so that section runs its
 * own equivalent watchdog. */
enum SlaveKind { SLAVE_KIND_NONE, SLAVE_KIND_CLASSIC, SLAVE_KIND_EREADER };
static volatile uint8_t g_slave_kind = SLAVE_KIND_NONE;

/* One recovery flip per mode-armed slave session; granted by
 * link_changeMode(SLAVE), consumed by the watchdog. */
static volatile uint8_t g_watchdog_flips_left = 0;

static PIO g_pio = NULL;
static size_t g_sm = 0;

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

uint16_t reverse_bit16(uint16_t x)
{
	x = ((x & 0x5555) << 1) | ((x & 0xAAAA) >> 1);
	x = ((x & 0x3333) << 2) | ((x & 0xCCCC) >> 2);
	x = ((x & 0x0F0F) << 4) | ((x & 0xF0F0) >> 4);
	return (x << 8) | (x >> 8);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

uint16_t g_lastTxValue = 0x00;

/* Words received since the last (re)configure — lets a mode distinguish
 * "partner is clocking but we hear nothing" (wrong SD pin) from a healthy
 * or idle link, independent of any logging state. */
static volatile uint32_t g_rx_words = 0;

uint32_t link_getReceivedWordCount(void)
{
    return g_rx_words;
}

static void pioIsr_done(const void* arg)
{
    (void)arg;
    g_rx_words++;
    uint16_t rxData = pio_sm_get(g_pio, g_sm);
    rxData = reverse_bit16(rxData);
    if (g_receiveCallback) g_receiveCallback(rxData, g_receiveUserData);
    if (g_transiveDoneCallback) g_transiveDoneCallback(rxData, g_lastTxValue, g_transiveDoneUserdata);
    pio_interrupt_clear(g_pio, TX_RX_DONE_IRQ);
}

static void pioIsr_tx(const void* arg)
{
    (void)arg;
    struct NextTransmit txValue = 
    {
        0xDEAD, 
        50000
    };

    if (g_transmitCallback) txValue = g_transmitCallback(g_transmitUserData);
    if (g_mode == MASTER) pio_sm_put(g_pio, g_sm, txValue.timingUs);
    pio_sm_put(g_pio, g_sm, txValue.value);
    g_lastTxValue = txValue.value;
    pio_interrupt_clear(g_pio, TX_VALUE_IRQ);
    return;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// Interface
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_setTransmitCallback(TransmitHandler cb, void* userData) 
{
    g_transmitUserData = userData;
    g_transmitCallback = cb;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_setReceiveCallback(ReceiveHandler cb, void* userData) 
{
    g_receiveUserData = userData;
    g_receiveCallback = cb;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_setTransiveDoneCallback(TransiveDoneHandler cb, void* user_data)
{
    g_transiveDoneUserdata = user_data;
    g_transiveDoneCallback = cb;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

enum LinkMode link_getMode()
{
    return g_mode;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_startTransive() {}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void link_configureMaster()
{
    g_mode = MASTER;
    g_slave_kind = SLAVE_KIND_NONE;
    g_rx_words = 0;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);

    bool gbc = link_useGbcCable();

	uint32_t offset = gbc
        ? pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_master_gbc))
        : pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_master_gba));

	pio_sm_config sm_config = pio_get_default_sm_config();

    #pragma push_macro("pio0")
    #undef pio0
    uint32_t SC_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 0);
    uint32_t SI_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1);
    uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
    #pragma pop_macro("pio0")
    uint32_t SD_pin = gbc ? 4 : 3;

    sm_config_set_out_pins(&sm_config, SD_pin, 1);
    sm_config_set_set_pins(&sm_config, SC_pin, gbc ? 5 : 4);
    sm_config_set_in_pins(&sm_config, SD_pin);
    sm_config_set_jmp_pin(&sm_config, SD_pin);

    sm_config_set_out_shift(&sm_config, true, false, 0);
    sm_config_set_in_shift(&sm_config, false, false, 0);

    pio_gpio_init(g_pio, SD_pin);
    if (gbc) gpio_pull_up(SD_pin);
    pio_gpio_init(g_pio, SC_pin);
    pio_gpio_init(g_pio, SO_pin);
    pio_gpio_init(g_pio, SI_pin);

	sm_config_set_clkdiv(&sm_config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200

	sm_config_set_wrap(&sm_config,
			   offset + (gbc ? RPI_PICO_PIO_GET_WRAP_TARGET(pio_master_gbc) : RPI_PICO_PIO_GET_WRAP_TARGET(pio_master_gba)),
			   offset + (gbc ? RPI_PICO_PIO_GET_WRAP(pio_master_gbc) : RPI_PICO_PIO_GET_WRAP(pio_master_gba)));
    
	pio_sm_init(g_pio, g_sm, -1, &sm_config);
	pio_sm_set_enabled(g_pio, g_sm, true);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void link_assertBothSdPartnerGpio(void)
{
    gpio_init(3); gpio_set_dir(3, GPIO_OUT); gpio_put(3, 1);
    gpio_init(4); gpio_set_dir(4, GPIO_OUT); gpio_put(4, 1);
}

uint8_t link_readPartnerPins(void)
{
    const uint32_t in = sio_hw->gpio_in;
    uint8_t mask = 0;
    if (!(in & (1u << 0))) mask |= 0x01;
    if (in & (1u << 1)) mask |= 0x02;
    if (in & (1u << 2)) mask |= 0x04;
    if (in & (1u << 3)) mask |= 0x08;
    if (in & (1u << 4)) mask |= 0x10;
    return mask;
}

static void link_releaseInactiveSdPin(void)
{
    const bool gbc = link_useGbcCable();
    const uint32_t inactive = gbc ? 3u : 4u;
    gpio_init(inactive);
    gpio_set_dir(inactive, GPIO_IN);
    gpio_disable_pulls(inactive);
}

static void link_configureSlaveCore(bool multiChild)
{
    g_mode = SLAVE;
    g_rx_words = 0;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);

    bool gbc = link_useGbcCable();

    const void* prog;
    if (multiChild)
        prog = gbc ? RPI_PICO_PIO_GET_PROGRAM(pio_slave_multi_gbc) : RPI_PICO_PIO_GET_PROGRAM(pio_slave_multi_gba);
    else
        prog = gbc ? RPI_PICO_PIO_GET_PROGRAM(pio_slave_gbc) : RPI_PICO_PIO_GET_PROGRAM(pio_slave_gba);

    uint32_t offset = pio_add_program(g_pio, prog);
    pio_sm_config sm_config = pio_get_default_sm_config();

    #pragma push_macro("pio0")
    #undef pio0
    uint32_t SC_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 0);
    uint32_t SI_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1);
    uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
    #pragma pop_macro("pio0")
    uint32_t SD_pin = gbc ? 4 : 3;

    sm_config_set_out_pins(&sm_config, SD_pin, 1);
    sm_config_set_set_pins(&sm_config, SC_pin, gbc ? 5 : 4);
    sm_config_set_in_pins(&sm_config, SD_pin);
    sm_config_set_out_shift(&sm_config, true, false, 0);
    sm_config_set_in_shift(&sm_config, false, false, 0);

    pio_gpio_init(g_pio, SD_pin);
    if (gbc) gpio_pull_up(SD_pin);
    pio_gpio_init(g_pio, SC_pin);
    pio_gpio_init(g_pio, SI_pin);
    if (multiChild)
    {
        gpio_init(SO_pin);
        gpio_set_dir(SO_pin, GPIO_OUT);
        gpio_put(SO_pin, 0);
    }
    else
    {
        pio_gpio_init(g_pio, SO_pin);
    }

    sm_config_set_clkdiv(&sm_config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200
    sm_config_set_wrap(&sm_config,
        offset + (gbc
            ? (multiChild ? RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_multi_gbc) : RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_gbc))
            : (multiChild ? RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_multi_gba) : RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_gba))),
        offset + (gbc
            ? (multiChild ? RPI_PICO_PIO_GET_WRAP(pio_slave_multi_gbc) : RPI_PICO_PIO_GET_WRAP(pio_slave_gbc))
            : (multiChild ? RPI_PICO_PIO_GET_WRAP(pio_slave_multi_gba) : RPI_PICO_PIO_GET_WRAP(pio_slave_gba))));

    pio_sm_init(g_pio, g_sm, -1, &sm_config);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SC_pin, 1, false);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SI_pin, 1, false);
    if (!multiChild)
        pio_sm_set_consecutive_pindirs(g_pio, g_sm, SO_pin, 1, true);

    pio_sm_set_enabled(g_pio, g_sm, true);
}

static void link_configureSlave(void)
{
    link_configureSlaveCore(false);
    g_slave_kind = SLAVE_KIND_CLASSIC;
}

static void link_disablePio(void)
{
    g_mode = DISABLED;
    g_slave_kind = SLAVE_KIND_NONE;
    g_rx_words = 0;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
}

void link_configurePartnerPresence(void)
{
    link_disablePio();
    g_mode = SLAVE;

    #pragma push_macro("pio0")
    #undef pio0
    uint32_t SC_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 0);
    uint32_t SI_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1);
    uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
    #pragma pop_macro("pio0")

    gpio_init(SC_pin); gpio_pull_up(SC_pin); gpio_set_dir(SC_pin, GPIO_IN);
    gpio_init(SI_pin); gpio_pull_up(SI_pin); gpio_set_dir(SI_pin, GPIO_IN);
    gpio_init(SO_pin); gpio_set_dir(SO_pin, GPIO_OUT); gpio_put(SO_pin, 0);
    link_assertBothSdPartnerGpio();
}

void link_configurePokemonSlave(void)
{
    link_disablePio();
    link_configureSlaveCore(false);
    g_slave_kind = SLAVE_KIND_EREADER;
}

void link_configureMultiSlave(void)
{
    link_disablePio();
    g_mode = SLAVE;

    #pragma push_macro("pio0")
    #undef pio0
    uint32_t SC_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 0);
    uint32_t SI_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1);
    uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
    #pragma pop_macro("pio0")

    gpio_init(SC_pin); gpio_pull_up(SC_pin); gpio_set_dir(SC_pin, GPIO_IN);
    gpio_init(SI_pin); gpio_pull_up(SI_pin); gpio_set_dir(SI_pin, GPIO_IN);
    gpio_init(SO_pin); gpio_set_dir(SO_pin, GPIO_OUT); gpio_put(SO_pin, 0);
    link_releaseInactiveSdPin();

    link_configureSlaveCore(true);
    g_slave_kind = SLAVE_KIND_EREADER;
}

/* Undo the SIO overrides the e-reader flows leave behind (SD pins driven high
 * for partner presence, SO forced low as MULTI child) so later modes start
 * from cable-idle pin state; the PIO paths only reclaim the pins they use. */
void link_releasePartnerPins(void)
{
    #pragma push_macro("pio0")
    #undef pio0
    uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
    #pragma pop_macro("pio0")

    gpio_init(SO_pin); gpio_set_dir(SO_pin, GPIO_IN); gpio_disable_pulls(SO_pin);
    gpio_init(3); gpio_set_dir(3, GPIO_IN); gpio_disable_pulls(3);
    gpio_init(4); gpio_set_dir(4, GPIO_IN); gpio_disable_pulls(4);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

/* Wrong-pin watchdog for classic slave modes (trade emu, GBA link relay, AW
 * slave). Cable detection is a heuristic and can pick the wrong SD pin (GBA
 * already in link mode at detect time, or cable plugged in afterwards). The
 * partner clocking SC while zero words arrive is proof of the wrong pin — SC
 * is the same pin for both cable types — so after sustained evidence the
 * watchdog flips the SD path and reconfigures, once per armed session.
 * Requiring both SC levels (a real toggle) rejects a stuck line, and any
 * received word disarms the watchdog until the next configure. */
static void link_watchdogThread(void* a, void* b, void* c)
{
    (void)a; (void)b; (void)c;
    bool scSeenLow = false;
    bool scSeenHigh = false;
    int ticks = 0;
    int clockingIntervals = 0;

    for (;;)
    {
        if (g_slave_kind != SLAVE_KIND_CLASSIC || g_watchdog_flips_left == 0)
        {
            scSeenLow = scSeenHigh = false;
            ticks = 0;
            clockingIntervals = 0;
            k_sleep(K_MSEC(100));
            continue;
        }

        k_sleep(K_MSEC(1));
        if (link_readPartnerPins() & 0x01) scSeenLow = true;
        else scSeenHigh = true;

        if (++ticks < 550) continue;
        ticks = 0;

        const bool scToggled = scSeenLow && scSeenHigh;
        scSeenLow = scSeenHigh = false;

        if (link_getReceivedWordCount() != 0)
        {
            clockingIntervals = 0;
            continue;
        }
        if (scToggled) clockingIntervals++;

        if (clockingIntervals >= 4)
        {
            clockingIntervals = 0;
            /* Re-check under lock: a mode switch may have raced the decision,
             * and the reconfigure must not interleave with one. */
            const unsigned int key = irq_lock();
            if (g_slave_kind == SLAVE_KIND_CLASSIC && g_rx_words == 0
                && g_watchdog_flips_left > 0)
            {
                g_watchdog_flips_left--;
                link_flipSdPath();
                link_configureSlave();
            }
            irq_unlock(key);
        }
    }
}

K_THREAD_DEFINE(link_watchdog_tid, 768, link_watchdogThread,
                NULL, NULL, NULL, 12, 0, 0);

void link_changeMode(enum LinkMode mode)
{
    switch (mode)
    {
        case SLAVE:
            g_watchdog_flips_left = 1;
            link_configureSlave();
            break;
        case MASTER:
            link_configureMaster();
            break;
        case DISABLED:
            link_disablePio();
    }   
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static int link_init()
{
    const struct device* dev = DEVICE_DT_GET(DT_PROP(DT_NODELABEL(pio_link), pio));

    const struct pinctrl_dev_config* config = PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(pio_link));
    g_pio = pio_rpi_pico_get_pio(dev);

    pio_rpi_pico_allocate_sm(dev, &g_sm);

    IRQ_CONNECT(PIO0_IRQ_0 , 0, pioIsr_done, NULL, 0);
    IRQ_CONNECT(PIO0_IRQ_1 , 0, pioIsr_tx, NULL, 0);

    irq_enable(PIO0_IRQ_0);
    irq_enable(PIO0_IRQ_1);

    pio_set_irq0_source_enabled(g_pio, pis_interrupt0, true);
    pio_set_irq1_source_enabled(g_pio, pis_interrupt1, true);

    int ret = pinctrl_apply_state(config, PINCTRL_STATE_DEFAULT);

    link_disablePio();

    return ret;
}

SYS_INIT(link_init, APPLICATION, 1);
