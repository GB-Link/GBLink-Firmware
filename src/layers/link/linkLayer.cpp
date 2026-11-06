extern "C"
{
    #include <zephyr/kernel.h>

    #include "hardware/pio.h"
    #include "hardware/gpio.h"

    #include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>
    #include <zephyr/drivers/pinctrl.h>

    #include "../linkSdPins.h"
}

#include "./linkLayer.hpp"
#include "./multiMode/multiModePio.hpp"

#define TX_RX_DONE_IRQ 0
#define TX_VALUE_IRQ 1
#define REPLY_WATCHDOG_IRQ 2

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void* g_receiveUserData = nullptr;
static ReceiveHandler g_receiveCallback = nullptr;

static void* g_transmitUserData = nullptr;
static TransmitHandler g_transmitCallback = nullptr;

static void* g_transiveDoneUserData = nullptr;
static TransiveDoneHandler g_transiveDoneCallback = nullptr;

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// State
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static PIO g_pio = nullptr;
static uint32_t g_sm = 0;
static uint32_t g_programOffset = 0;

// DISABLED while the 2 player programs have the state machine.
static volatile enum LinkMode g_role = DISABLED;

// The transfer in progress, and the master's gap after it.
static struct NextTransmit g_nextTransmission = {};
static uint32_t g_gap = 0;

// Master: pin the GBA answers on, 0 until it has answered. Each word is driven on this pin only, so while it is 0
// the transfer leaves SD released and the GBA's reply shows which pin carries SD. It outlasts the session: a GBA
// already in its handshake in slot 2 or 3 does not answer a transfer whose earlier slots are quiet. Another mode
// clears it (link_forgetSdPin).
static volatile uint8_t g_sdTxMask = 0;

// Master: ends the wait for a GBA that does not answer by receiving the idle line (0xFFFF).
static struct k_timer g_replyWatchdog;

// Master: a gap of at least this many cycles (~0.6 ms with the offset) outlasts the done interrupt's callbacks even
// from a cold flash cache, so its control word can go out before them.
static constexpr uint32_t CONTROL_FIRST_MIN_GAP = 800;

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

#pragma push_macro("pio0")
#undef pio0
static constexpr uint32_t SC_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 0);
static constexpr uint32_t SI_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1);
static constexpr uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
static constexpr uint32_t SD_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 3);
static constexpr uint32_t SD2_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 4);
#pragma pop_macro("pio0")

// The programs address SC..SD2 as one SET group and the two SD pins as one OUT/IN group.
static_assert(SI_pin == SC_pin + 1 && SO_pin == SC_pin + 2 && SD_pin == SC_pin + 3 && SD2_pin == SC_pin + 4,
              "link pins must be consecutive: SC, SI, SO, SD, SD2");

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static NextTransmit nextTransmission()
{
    NextTransmit next = {1, TxValues(0xFFFF), 50000};
    if (g_transmitCallback) next = g_transmitCallback(g_transmitUserData);
    // The slave program waits for at least one word.
    if (next.txValues.size() == 0) next.txValues.push(0x0000);
    return next;
}

static void putWords()
{
    for (uint16_t value : g_nextTransmission.txValues)
    {
        pio_sm_put(g_pio, g_sm, sdPinsSpread(value));
    }
}

// Master: the control word of the next transfer. Its gap is the one handed out with the transfer just done.
static void putControl()
{
    pio_sm_put(g_pio, g_sm, g_sdTxMask | ((uint32_t)(g_nextTransmission.seatNumber & 0x3) << 2)
        | ((g_gap + MULTI_GAP_OFFSET) << 4));
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// IRQ
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void pioIsr_done()
{
    if (!pio_interrupt_get(g_pio, TX_RX_DONE_IRQ)) return;
    k_timer_stop(&g_replyWatchdog);

    if (pio_sm_is_rx_fifo_empty(g_pio, g_sm))
    {
        // Master start: nothing received yet.
        if (g_role == MASTER)
        {
            g_nextTransmission = nextTransmission();
            putControl();
            putWords();
            g_gap = g_nextTransmission.timingUs;
        }
        pio_interrupt_clear(g_pio, TX_RX_DONE_IRQ);
        return;
    }

    const uint32_t raw = pio_sm_get(g_pio, g_sm);
    const uint8_t pin = sdPinsFramePin(raw);
    if (pin && g_role == MASTER) g_sdTxMask = pin;

    // A slot that finds no word queued ends the transfer, and late words would then be read as the next control
    // word. So the control word goes first, starting the gap on time, only when the gap outlasts the callbacks;
    // a shorter one (the first of a session is 0) starts once the words are queued.
    const bool controlFirst = (g_role == MASTER) && (g_gap >= CONTROL_FIRST_MIN_GAP);
    if (controlFirst) putControl();

    const uint16_t rxData = sdPinsDecode(raw);
    if (g_receiveCallback) g_receiveCallback(rxData, g_receiveUserData);
    if (g_transiveDoneCallback) g_transiveDoneCallback(rxData, g_nextTransmission.txValues, g_transiveDoneUserData);

    if (g_role == MASTER)
    {
        const NextTransmit next = nextTransmission();
        if (!controlFirst) putControl();
        g_nextTransmission = next;
        putWords();
        g_gap = g_nextTransmission.timingUs;
    }
    pio_interrupt_clear(g_pio, TX_RX_DONE_IRQ);
}

static void pioIsr_tx()
{
    if (pio_interrupt_get(g_pio, REPLY_WATCHDOG_IRQ))
    {
        k_timer_start(&g_replyWatchdog, K_MSEC(1), K_NO_WAIT);
        pio_interrupt_clear(g_pio, REPLY_WATCHDOG_IRQ);
    }
    if (pio_interrupt_get(g_pio, TX_VALUE_IRQ))
    {
        // Slave: the words that answer the GBA's word arriving now.
        g_nextTransmission = nextTransmission();
        putWords();
        pio_interrupt_clear(g_pio, TX_VALUE_IRQ);
    }
}

static void replyWatchdogExpired(struct k_timer* timer)
{
    (void)timer;
    if (g_role != MASTER) return;
    const uint32_t pc = pio_sm_get_pc(g_pio, g_sm) - g_programOffset;
    if (pc >= MULTI_POLL_FIRST && pc <= MULTI_POLL_LAST)
    {
        pio_sm_exec(g_pio, g_sm, pio_encode_jmp(g_programOffset + MULTI_RECEIVE));
    }
}

// The 2 player programs take the state machine back.
static void revoke()
{
    k_timer_stop(&g_replyWatchdog);
    g_role = DISABLED;
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

void link_setTransiveDoneCallback(TransiveDoneHandler cb, void* userData)
{
    g_transiveDoneUserData = userData;
    g_transiveDoneCallback = cb;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_disable()
{
    link_changeMode(DISABLED);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_enable()
{
    pio_sm_set_enabled(g_pio, g_sm, true);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_forgetSdPin()
{
    g_sdTxMask = 0;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_configureProgram(const pio_program_t* program, uint32_t wrap, uint32_t wrapTarget, enum LinkMode role)
{
    g_sm = link_lendStateMachine(&pioIsr_done, &pioIsr_tx, &revoke);
    g_role = role;
    g_gap = 0;
    g_nextTransmission = {};

    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);
    pio_interrupt_clear(g_pio, TX_RX_DONE_IRQ);
    pio_interrupt_clear(g_pio, TX_VALUE_IRQ);
    pio_interrupt_clear(g_pio, REPLY_WATCHDOG_IRQ);

    g_programOffset = pio_add_program(g_pio, program);

    pio_sm_config config = pio_get_default_sm_config();
    sm_config_set_set_pins(&config, SC_pin, 5);
    sm_config_set_out_pins(&config, SD_pin, 2);
    sm_config_set_in_pins(&config, SD_pin);
    sm_config_set_out_shift(&config, true, false, 0);
    sm_config_set_in_shift(&config, false, false, 0);
    sm_config_set_mov_status(&config, STATUS_TX_LESSTHAN, 1);
    sm_config_set_clkdiv(&config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200
    sm_config_set_wrap(&config, g_programOffset + wrapTarget, g_programOffset + wrap);

    // Both SD pins with a pull-up, so the one that does not carry SD reads high when it is open. Hysteresis on:
    // Zephyr's pinctrl turns it off for the link pins.
    for (uint32_t pin = SD_pin; pin <= SD2_pin; pin++)
    {
        pio_gpio_init(g_pio, pin);
        gpio_pull_up(pin);
        gpio_set_input_hysteresis_enabled(pin, true);
    }
    pio_gpio_init(g_pio, SC_pin);
    pio_gpio_init(g_pio, SI_pin);
    pio_gpio_init(g_pio, SO_pin);

    pio_sm_init(g_pio, g_sm, g_programOffset, &config);

    // Idle levels before the pins turn into outputs: the master's first SET only follows its first done interrupt.
    const bool master = (role == MASTER);
    const uint32_t pins = (1u << SC_pin) | (1u << SO_pin) | (3u << SD_pin);
    const uint32_t levels = (master ? (1u << SC_pin) | (1u << SO_pin) : 0) | (3u << SD_pin);
    pio_sm_set_pins_with_mask(g_pio, g_sm, levels, pins);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SC_pin, 1, master);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SI_pin, 1, false);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SO_pin, 1, true);
    pio_sm_set_consecutive_pindirs(g_pio, g_sm, SD_pin, 2, false);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static int init()
{
    const struct device* dev = DEVICE_DT_GET(DT_PROP(DT_NODELABEL(pio_link), pio));
    g_pio = pio_rpi_pico_get_pio(dev);

    k_timer_init(&g_replyWatchdog, replyWatchdogExpired, NULL);

    return 0;
}

SYS_INIT(init, APPLICATION, 1);
