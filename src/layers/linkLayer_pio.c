#include "linkLayer.h"
#include "linkSdPins.h"

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

/* link_pins: SC, SI, SO, then SD as it arrives on port pin 4 (GP3) and on port pin 1 (GP4). */
#pragma push_macro("pio0")
#undef pio0
enum
{
    LINK_PIN_SC  = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 0),
    LINK_PIN_SI  = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1),
    LINK_PIN_SO  = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2),
    LINK_PIN_SD  = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 3),
    LINK_PIN_SD2 = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 4),
};
#pragma pop_macro("pio0")

/* The programs address SC..SD2 as one SET group and the two SD pins as one OUT/IN group. */
BUILD_ASSERT(LINK_PIN_SI == LINK_PIN_SC + 1 && LINK_PIN_SO == LINK_PIN_SC + 2 &&
             LINK_PIN_SD == LINK_PIN_SC + 3 && LINK_PIN_SD2 == LINK_PIN_SC + 4,
             "link pins must be consecutive: SC, SI, SO, SD, SD2");

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void* g_receiveUserData = NULL;
static LegacyReceiveHandler g_receiveCallback = NULL;

void* g_transmitUserData = NULL;
static LegacyTransmitHandler g_transmitCallback = NULL;

void* g_transiveDoneUserdata = NULL;
static LegacyTransiveDoneHandler g_transiveDoneCallback = NULL;

static enum LinkMode g_mode = SLAVE;

/* Set while the state machine is lent to the 2-4 player programs (link_lendStateMachine). */
static LinkIrqHandler volatile g_lentIrq0 = NULL;
static LinkIrqHandler volatile g_lentIrq1 = NULL;
static LinkIrqHandler volatile g_lentRevoke = NULL;

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

/* SET-instruction pin values: bit 0 SC (GP0), bit 1 SI (GP1, always input), bit 2 SO (GP2),
 * bit 3 SD on GP3, bit 4 SD on GP4. OUT and IN cover GP3..GP4.
 *
 * Depending on the cable, SD arrives on GP3 or GP4 and the other pin is open or carries the partner's
 * VCC. Both programs read the two pins together and drive SD only on the pin the partner's start bit came
 * in on, so no cable selection is needed and a pin wired to VCC is never driven.
 * Sources: pio_firmware/pio_master_mode.pio, pio_firmware/pio_slave_mode.pio. */

/* TX FIFO per transfer: SD output mask (0 until the GBA has answered once), value with each bit doubled,
 * gap before the next transfer. The gap for the first transfer is queued at configure. */
RPI_PICO_PIO_DEFINE_PROGRAM(pio_master, 0, 31,
    0xe085, //  0: set    pindirs, 5
    0xe01d, //  1: set    pins, 29
    0x80a0, //  2: pull   block
    0xa047, //  3: mov    y, osr
    0x0084, //  4: jmp    y--, 4
    0xc001, //  5: irq    nowait 1
    0xe03e, //  6: set    x, 30
    0x0247, //  7: jmp    x--, 7                 [2]
    0x80a0, //  8: pull   block
    0x6082, //  9: out    pindirs, 2
    0xe02f, // 10: set    x, 15
    0x80a0, // 11: pull   block
    0xe01c, // 12: set    pins, 28
    0xef04, // 13: set    pins, 4                [15]
    0x6e02, // 14: out    pins, 2                [14]
    0x004e, // 15: jmp    x--, 14
    0xef1c, // 16: set    pins, 28               [15]
    0xe018, // 17: set    pins, 24
    0xe085, // 18: set    pindirs, 5
    0xe03f, // 19: set    x, 31
    0xa1cb, // 20: mov    isr, !null             [1]
    0x1d56, // 21: jmp    x--, 22                [29]
    0x003a, // 22: jmp    !x, 26
    0x4002, // 23: in     pins, 2
    0xa04e, // 24: mov    y, !isr
    0x0075, // 25: jmp    !y, 21
    0xf42f, // 26: set    x, 15                  [20]
    0x4e02, // 27: in     pins, 2                [14]
    0x005b, // 28: jmp    x--, 27
    0x9020, // 29: push   block                  [16]
    0xfe1c, // 30: set    pins, 28               [30]
    0xc000);// 31: irq    nowait 0

/* TX FIFO per transfer: value with each bit doubled. SO is driven by the program, or held low by GPIO for
 * the e-reader child (its SET writes to SO then go nowhere). */
RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave, 0, 29,
    0xe018, //  0: set    pins, 24
    0xe084, //  1: set    pindirs, 4
    0xe02f, //  2: set    x, 15
    0xa0cb, //  3: mov    isr, !null
    0x4002, //  4: in     pins, 2
    0xa04e, //  5: mov    y, !isr
    0x0083, //  6: jmp    y--, 3
    0x4002, //  7: in     pins, 2
    0xa04e, //  8: mov    y, !isr
    0x0067, //  9: jmp    !y, 7
    0xd401, // 10: irq    nowait 1               [20]
    0x4e02, // 11: in     pins, 2                [14]
    0x004b, // 12: jmp    x--, 11
    0x8020, // 13: push   block
    0xa0cb, // 14: mov    isr, !null
    0x4002, // 15: in     pins, 2
    0xa02e, // 16: mov    x, !isr
    0x0033, // 17: jmp    !x, 19
    0xa043, // 18: mov    y, null
    0xa0e2, // 19: mov    osr, y
    0x6082, // 20: out    pindirs, 2
    0xbb42, // 21: nop                           [27]
    0xe02f, // 22: set    x, 15
    0x80a0, // 23: pull   block
    0xf000, // 24: set    pins, 0                [16]
    0x6e02, // 25: out    pins, 2                [14]
    0x0059, // 26: jmp    x--, 25
    0xf018, // 27: set    pins, 24               [16]
    0xc000, // 28: irq    nowait 0
    0xbf42);// 29: nop                           [31]

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

/* Master: pin the GBA answers on, 0 until it has answered in this session. Each word is driven on this pin
 * only, so the first transfer leaves SD released and the GBA's reply to it shows which pin carries SD. */
static volatile uint8_t g_sdTxMask = 0;

static PIO g_pio = NULL;
static size_t g_sm = 0;

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

uint16_t g_lastTxValue = 0x00;

/* Words received since the last (re)configure, so a mode can tell a partner
 * that clocks without any word arriving from a healthy or idle link,
 * independent of any logging state. */
static volatile uint32_t g_rx_words = 0;

uint32_t link_getReceivedWordCount(void)
{
    return g_rx_words;
}

static void pioIsr_done(const void* arg)
{
    (void)arg;
    if (g_lentIrq0)
    {
        g_lentIrq0();
        return;
    }
    g_rx_words++;
    const bool fresh = !pio_sm_is_rx_fifo_empty(g_pio, g_sm);
    const uint32_t raw = pio_sm_get(g_pio, g_sm);
    if (fresh)
    {
        const uint8_t pin = sdPinsFramePin(raw);
        if (pin && g_mode == MASTER) g_sdTxMask = pin;
    }
    const uint16_t rxData = sdPinsDecode(raw);
    if (g_receiveCallback) g_receiveCallback(rxData, g_receiveUserData);
    if (g_transiveDoneCallback) g_transiveDoneCallback(rxData, g_lastTxValue, g_transiveDoneUserdata);
    pio_interrupt_clear(g_pio, TX_RX_DONE_IRQ);
}

static void pioIsr_tx(const void* arg)
{
    (void)arg;
    if (g_lentIrq1)
    {
        g_lentIrq1();
        return;
    }
    struct LegacyTransmit txValue =
    {
        0xDEAD,
        50000
    };

    if (g_transmitCallback) txValue = g_transmitCallback(g_transmitUserData);
    if (g_mode == MASTER)
    {
        pio_sm_put(g_pio, g_sm, g_sdTxMask);
        pio_sm_put(g_pio, g_sm, sdPinsSpread(txValue.value));
        pio_sm_put(g_pio, g_sm, txValue.timingUs);
    }
    else
    {
        pio_sm_put(g_pio, g_sm, sdPinsSpread(txValue.value));
    }
    g_lastTxValue = txValue.value;
    pio_interrupt_clear(g_pio, TX_VALUE_IRQ);
    return;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// Interface
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_setLegacyTransmitCallback(LegacyTransmitHandler cb, void* userData)
{
    g_transmitUserData = userData;
    g_transmitCallback = cb;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_setLegacyReceiveCallback(LegacyReceiveHandler cb, void* userData)
{
    g_receiveUserData = userData;
    g_receiveCallback = cb;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_setLegacyTransiveDoneCallback(LegacyTransiveDoneHandler cb, void* user_data)
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

/* Takes a lent state machine back, once it has stopped. */
static void reclaimStateMachine(void)
{
    if (g_lentRevoke) g_lentRevoke();
    g_lentIrq0 = NULL;
    g_lentIrq1 = NULL;
    g_lentRevoke = NULL;
}

static pio_sm_config linkSmConfig(uint32_t offset, uint32_t wrapTarget, uint32_t wrap)
{
    pio_sm_config sm_config = pio_get_default_sm_config();
    sm_config_set_set_pins(&sm_config, LINK_PIN_SC, 5);
    sm_config_set_out_pins(&sm_config, LINK_PIN_SD, 2);
    sm_config_set_in_pins(&sm_config, LINK_PIN_SD);
    sm_config_set_out_shift(&sm_config, true, false, 0);
    sm_config_set_in_shift(&sm_config, false, false, 0);
    sm_config_set_clkdiv(&sm_config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200
    sm_config_set_wrap(&sm_config, offset + wrapTarget, offset + wrap);
    return sm_config;
}

/* Both SD pins go to the PIO with a pull-up, so the one that does not carry SD reads high when it is open.
 * Hysteresis on: Zephyr's pinctrl turns it off for the link pins. */
static void claimSdPins(void)
{
    for (uint32_t pin = LINK_PIN_SD; pin <= LINK_PIN_SD2; pin++)
    {
        pio_gpio_init(g_pio, pin);
        gpio_pull_up(pin);
        gpio_set_input_hysteresis_enabled(pin, true);
    }
}

static void link_configureMaster()
{
    g_mode = MASTER;
    g_rx_words = 0;
    g_sdTxMask = 0;
    pio_sm_set_enabled(g_pio, g_sm, false);
    reclaimStateMachine();
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);

    uint32_t offset = pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_master));
    pio_sm_config sm_config = linkSmConfig(offset,
        RPI_PICO_PIO_GET_WRAP_TARGET(pio_master), RPI_PICO_PIO_GET_WRAP(pio_master));

    claimSdPins();
    pio_gpio_init(g_pio, LINK_PIN_SC);
    pio_gpio_init(g_pio, LINK_PIN_SO);
    pio_gpio_init(g_pio, LINK_PIN_SI);

    pio_sm_init(g_pio, g_sm, -1, &sm_config);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, LINK_PIN_SD, 2, false);
    pio_sm_put(g_pio, g_sm, 0); // gap before the first transfer
    pio_sm_set_enabled(g_pio, g_sm, true);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void assertBothSdPartnerGpio(void)
{
    gpio_init(LINK_PIN_SD); gpio_set_dir(LINK_PIN_SD, GPIO_OUT); gpio_put(LINK_PIN_SD, 1);
    gpio_init(LINK_PIN_SD2); gpio_set_dir(LINK_PIN_SD2, GPIO_OUT); gpio_put(LINK_PIN_SD2, 1);
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

/* driveSo: the program drives SO (classic child). Otherwise SO stays a GPIO held low (e-reader child). */
static void configureSlavePio(bool driveSo)
{
    g_mode = SLAVE;
    g_rx_words = 0;
    pio_sm_set_enabled(g_pio, g_sm, false);
    reclaimStateMachine();
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);

    uint32_t offset = pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_slave));
    pio_sm_config sm_config = linkSmConfig(offset,
        RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave), RPI_PICO_PIO_GET_WRAP(pio_slave));

    claimSdPins();
    pio_gpio_init(g_pio, LINK_PIN_SC);
    pio_gpio_init(g_pio, LINK_PIN_SI);
    if (driveSo)
    {
        pio_gpio_init(g_pio, LINK_PIN_SO);
    }
    else
    {
        gpio_init(LINK_PIN_SO);
        gpio_set_dir(LINK_PIN_SO, GPIO_OUT);
        gpio_put(LINK_PIN_SO, 0);
    }

    pio_sm_init(g_pio, g_sm, -1, &sm_config);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, LINK_PIN_SC, 1, false);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, LINK_PIN_SI, 1, false);
    if (driveSo) pio_sm_set_consecutive_pindirs(g_pio, g_sm, LINK_PIN_SO, 1, true);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, LINK_PIN_SD, 2, false);

    pio_sm_set_enabled(g_pio, g_sm, true);
}

static void link_disablePio(void)
{
    g_mode = DISABLED;
    g_rx_words = 0;
    pio_sm_set_enabled(g_pio, g_sm, false);
    reclaimStateMachine();
    pio_clear_instruction_memory(g_pio);
}

void link_configurePartnerPresence(void)
{
    link_disablePio();
    g_mode = SLAVE;

    gpio_init(LINK_PIN_SC); gpio_pull_up(LINK_PIN_SC); gpio_set_dir(LINK_PIN_SC, GPIO_IN);
    gpio_init(LINK_PIN_SI); gpio_pull_up(LINK_PIN_SI); gpio_set_dir(LINK_PIN_SI, GPIO_IN);
    gpio_init(LINK_PIN_SO); gpio_set_dir(LINK_PIN_SO, GPIO_OUT); gpio_put(LINK_PIN_SO, 0);
    assertBothSdPartnerGpio();
}

void link_configurePokemonSlave(void)
{
    link_disablePio();
    configureSlavePio(true);
}

void link_configureEreaderSlave(void)
{
    link_disablePio();
    g_mode = SLAVE;

    gpio_init(LINK_PIN_SC); gpio_pull_up(LINK_PIN_SC); gpio_set_dir(LINK_PIN_SC, GPIO_IN);
    gpio_init(LINK_PIN_SI); gpio_pull_up(LINK_PIN_SI); gpio_set_dir(LINK_PIN_SI, GPIO_IN);
    gpio_init(LINK_PIN_SO); gpio_set_dir(LINK_PIN_SO, GPIO_OUT); gpio_put(LINK_PIN_SO, 0);

    configureSlavePio(false);
}

void link_configureBattleChipGateSlave(void)
{
    link_configureEreaderSlave();
}

/* Undo the SIO overrides the e-reader flows leave behind (SD pins driven high
 * for partner presence, SO forced low as e-reader child) so later modes start
 * from cable-idle pin state; the PIO paths only reclaim the pins they use. */
void link_releasePartnerPins(void)
{
    gpio_init(LINK_PIN_SO); gpio_set_dir(LINK_PIN_SO, GPIO_IN); gpio_disable_pulls(LINK_PIN_SO);
    gpio_init(LINK_PIN_SD); gpio_set_dir(LINK_PIN_SD, GPIO_IN); gpio_disable_pulls(LINK_PIN_SD);
    gpio_init(LINK_PIN_SD2); gpio_set_dir(LINK_PIN_SD2, GPIO_IN); gpio_disable_pulls(LINK_PIN_SD2);
}

void link_changeMode(enum LinkMode mode)
{
    switch (mode)
    {
        case SLAVE:
            configureSlavePio(true);
            break;
        case MASTER:
            link_configureMaster();
            break;
        case DISABLED:
            link_disablePio();
    }
}

uint32_t link_lendStateMachine(LinkIrqHandler irq0, LinkIrqHandler irq1, LinkIrqHandler revoke)
{
    link_disablePio();
    g_lentIrq0 = irq0;
    g_lentIrq1 = irq1;
    g_lentRevoke = revoke;
    return g_sm;
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
    pio_set_irq1_source_enabled(g_pio, pis_interrupt2, true);

    int ret = pinctrl_apply_state(config, PINCTRL_STATE_DEFAULT);

    link_disablePio();

    return ret;
}

SYS_INIT(link_init, APPLICATION, 1);
