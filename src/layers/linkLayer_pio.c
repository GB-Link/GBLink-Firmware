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

/* e-reader child: SO stays on GPIO LOW — no PIO_SO in set ops. */
RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_ereader_gba, 0, 18,
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

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_ereader_gbc, 0, 18,
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
#define CABLE_DETECT_SETTLE_ITERS   1000    /* ~10us */
#define CABLE_DETECT_SAMPLE_ITERS   200000  /* ~2ms */

/* link_pins indices: 0=SC, 1=SI, 2=SO, 3=SD_GBA, 4=SD_GBC */
static bool detect_gbc_cable(void)
{
    #pragma push_macro("pio0")
    #undef pio0
    const uint32_t si_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1);
    #pragma pop_macro("pio0")

    gpio_set_function(si_pin, GPIO_FUNC_SIO);
    gpio_set_dir(si_pin, GPIO_IN);
    gpio_pull_up(si_pin);
    for (volatile int i = 0; i < CABLE_DETECT_SETTLE_ITERS; i++);  /* settle ~10us */
    bool gbcCableDetected = false;
    for (volatile int i = 0; i < CABLE_DETECT_SAMPLE_ITERS; i++)  /* sample ~2ms */
    {
        if (gpio_get(si_pin)) { gbcCableDetected = true; break; }
    }
    gpio_set_function(si_pin, GPIO_FUNC_PIO0);
    return gbcCableDetected;
}

static bool g_gbc_cable = false;

#define CABLE_AUTO      0
#define CABLE_FORCE_GBA 1
#define CABLE_FORCE_GBC 2
static uint8_t g_cableOverride = CABLE_AUTO;

static bool useGbcCable(void)
{
    if (g_cableOverride == CABLE_FORCE_GBA) return false;
    if (g_cableOverride == CABLE_FORCE_GBC) return true;
    return g_gbc_cable;
}

void link_detectCableType(void)
{
    g_gbc_cable = detect_gbc_cable();
}

enum CableType link_getDetectedCableType(void)
{
    return useGbcCable() ? GBC : GBA;
}

void link_setCableOverride(uint8_t mode)
{
    if (mode > CABLE_FORCE_GBC) mode = CABLE_AUTO;
    g_cableOverride = mode;
}

/* Force the SD pin path opposite to the currently effective one (wrong-pin
 * recovery). Cleared back to auto by the next SetCableOverride. */
void link_flipSdPinPath(void)
{
    g_cableOverride = useGbcCable() ? CABLE_FORCE_GBA : CABLE_FORCE_GBC;
}

/* Which slave flavour is configured. Classic is watched by the external
 * wrong-pin watchdog; e-reader kinds re-arm through section state. */
static volatile uint8_t g_slave_kind = NONE;

/* One recovery flip per mode-armed slave session; granted by
 * link_changeMode(SLAVE), consumed by the wrong-pin watchdog. */
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

enum SlaveMode link_getSlaveMode(void)
{
    return (enum SlaveMode)g_slave_kind;
}

uint8_t link_getWrongPinFlipsLeft(void)
{
    return g_watchdog_flips_left;
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
    g_slave_kind = NONE;
    g_rx_words = 0;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);

    bool gbc = useGbcCable();

	uint32_t offset = gbc
        ? pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_master_gbc))
        : pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_master_gba));

	pio_sm_config sm_config = pio_get_default_sm_config();

    #pragma push_macro("pio0")
    #undef pio0
    uint32_t SC_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 0);
    uint32_t SI_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1);
    uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
    uint32_t SD_pin = gbc
        ? DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 4)
        : DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 3);
    #pragma pop_macro("pio0")

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

static void assertBothSdPartnerGpio(void)
{
    #pragma push_macro("pio0")
    #undef pio0
    const uint32_t sd_gba = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 3);
    const uint32_t sd_gbc = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 4);
    #pragma pop_macro("pio0")

    gpio_init(sd_gba); gpio_set_dir(sd_gba, GPIO_OUT); gpio_put(sd_gba, 1);
    gpio_init(sd_gbc); gpio_set_dir(sd_gbc, GPIO_OUT); gpio_put(sd_gbc, 1);
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

static void releaseInactiveSdPin(void)
{
    const bool gbc = useGbcCable();
    #pragma push_macro("pio0")
    #undef pio0
    const uint32_t inactive = gbc
        ? DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 3)
        : DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 4);
    #pragma pop_macro("pio0")
    gpio_init(inactive);
    gpio_set_dir(inactive, GPIO_IN);
    gpio_disable_pulls(inactive);
}

static void beginSlavePioConfig(void)
{
    g_mode = SLAVE;
    g_rx_words = 0;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);
}

static void configureClassicSlavePio(void)
{
    beginSlavePioConfig();

    bool gbc = useGbcCable();
    const void* prog = gbc
        ? RPI_PICO_PIO_GET_PROGRAM(pio_slave_gbc)
        : RPI_PICO_PIO_GET_PROGRAM(pio_slave_gba);

    uint32_t offset = pio_add_program(g_pio, prog);
    pio_sm_config sm_config = pio_get_default_sm_config();

    #pragma push_macro("pio0")
    #undef pio0
    uint32_t SC_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 0);
    uint32_t SI_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1);
    uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
    uint32_t SD_pin = gbc
        ? DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 4)
        : DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 3);
    #pragma pop_macro("pio0")

    sm_config_set_out_pins(&sm_config, SD_pin, 1);
    sm_config_set_set_pins(&sm_config, SC_pin, gbc ? 5 : 4);
    sm_config_set_in_pins(&sm_config, SD_pin);
    sm_config_set_out_shift(&sm_config, true, false, 0);
    sm_config_set_in_shift(&sm_config, false, false, 0);

    pio_gpio_init(g_pio, SD_pin);
    if (gbc) gpio_pull_up(SD_pin);
    pio_gpio_init(g_pio, SC_pin);
    pio_gpio_init(g_pio, SI_pin);
    pio_gpio_init(g_pio, SO_pin);

    sm_config_set_clkdiv(&sm_config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200
    sm_config_set_wrap(&sm_config,
        offset + (gbc ? RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_gbc) : RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_gba)),
        offset + (gbc ? RPI_PICO_PIO_GET_WRAP(pio_slave_gbc) : RPI_PICO_PIO_GET_WRAP(pio_slave_gba)));

    pio_sm_init(g_pio, g_sm, -1, &sm_config);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SC_pin, 1, false);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SI_pin, 1, false);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SO_pin, 1, true);

    pio_sm_set_enabled(g_pio, g_sm, true);
}

static void configureEreaderSlavePio(void)
{
    beginSlavePioConfig();

    bool gbc = useGbcCable();
    const void* prog = gbc
        ? RPI_PICO_PIO_GET_PROGRAM(pio_slave_ereader_gbc)
        : RPI_PICO_PIO_GET_PROGRAM(pio_slave_ereader_gba);

    uint32_t offset = pio_add_program(g_pio, prog);
    pio_sm_config sm_config = pio_get_default_sm_config();

    #pragma push_macro("pio0")
    #undef pio0
    uint32_t SC_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 0);
    uint32_t SI_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1);
    uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
    uint32_t SD_pin = gbc
        ? DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 4)
        : DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 3);
    #pragma pop_macro("pio0")

    sm_config_set_out_pins(&sm_config, SD_pin, 1);
    sm_config_set_set_pins(&sm_config, SC_pin, gbc ? 5 : 4);
    sm_config_set_in_pins(&sm_config, SD_pin);
    sm_config_set_out_shift(&sm_config, true, false, 0);
    sm_config_set_in_shift(&sm_config, false, false, 0);

    pio_gpio_init(g_pio, SD_pin);
    if (gbc) gpio_pull_up(SD_pin);
    pio_gpio_init(g_pio, SC_pin);
    pio_gpio_init(g_pio, SI_pin);
    gpio_init(SO_pin);
    gpio_set_dir(SO_pin, GPIO_OUT);
    gpio_put(SO_pin, 0);

    sm_config_set_clkdiv(&sm_config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200
    sm_config_set_wrap(&sm_config,
        offset + (gbc ? RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_ereader_gbc) : RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_ereader_gba)),
        offset + (gbc ? RPI_PICO_PIO_GET_WRAP(pio_slave_ereader_gbc) : RPI_PICO_PIO_GET_WRAP(pio_slave_ereader_gba)));

    pio_sm_init(g_pio, g_sm, -1, &sm_config);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SC_pin, 1, false);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SI_pin, 1, false);

    pio_sm_set_enabled(g_pio, g_sm, true);
}

static void link_configureSlave(void)
{
    configureClassicSlavePio();
    g_slave_kind = CLASSIC;
}

bool link_tryWrongPinRecovery(void)
{
    const unsigned int key = irq_lock();
    bool recovered = false;
    if (g_slave_kind == CLASSIC && g_rx_words == 0 && g_watchdog_flips_left > 0)
    {
        g_watchdog_flips_left--;
        link_flipSdPinPath();
        link_configureSlave();
        recovered = true;
    }
    irq_unlock(key);
    return recovered;
}

static void link_disablePio(void)
{
    g_mode = DISABLED;
    g_slave_kind = NONE;
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
    assertBothSdPartnerGpio();
}

void link_configurePokemonSlave(void)
{
    link_disablePio();
    configureClassicSlavePio();
    g_slave_kind = EREADER;
}

void link_configureEreaderSlave(void)
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
    releaseInactiveSdPin();

    configureEreaderSlavePio();
    g_slave_kind = EREADER;
}

/* Undo the SIO overrides the e-reader flows leave behind (SD pins driven high
 * for partner presence, SO forced low as e-reader child) so later modes start
 * from cable-idle pin state; the PIO paths only reclaim the pins they use. */
void link_releasePartnerPins(void)
{
    #pragma push_macro("pio0")
    #undef pio0
    uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
    uint32_t sd_gba = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 3);
    uint32_t sd_gbc = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 4);
    #pragma pop_macro("pio0")

    gpio_init(SO_pin); gpio_set_dir(SO_pin, GPIO_IN); gpio_disable_pulls(SO_pin);
    gpio_init(sd_gba); gpio_set_dir(sd_gba, GPIO_IN); gpio_disable_pulls(sd_gba);
    gpio_init(sd_gbc); gpio_set_dir(sd_gbc, GPIO_IN); gpio_disable_pulls(sd_gbc);
}

void link_changeMode(enum LinkMode mode)
{
    switch (mode)
    {
        case SLAVE:
            /* External wrong-pin watchdog consumes this budget. */
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
