extern "C"
{
    #include <zephyr/kernel.h>

    #include "hardware/pio.h"
    #include "hardware/gpio.h"

    #include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>
    #include <zephyr/drivers/pinctrl.h>
    #include "linkPinctrl.h"
}

#include "./gbaLinkLayer.hpp"



#define PREP_NEXT_TRANSMISSION_FLAG 0
#define RX_TX_DONE_FLAG 1
#define START_WATCHDOG 2

#define SD_PIN_GP3 0x1u
#define SD_PIN_GP4 0x2u

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

struct LinkPio
{
    PIO device;
    size_t id;
};

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void* g_receiveUserData = NULL;
static ReceiveHandler g_receiveCallback = NULL;

void* g_transmitUserData = NULL;
static TransmitHandler g_transmitCallback = NULL;

void* g_transiveDoneUserdata = NULL;
static TransiveDoneHandler g_transiveDoneCallback = NULL;

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// State
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static struct LinkPio g_pio = { .device = NULL, .id = 0};

static pio_sm_config g_config = {};

static uint32_t g_wordCount = 0;

static struct NextTransmit g_nextTransmission = {};

static constexpr uint32_t prepWindowCycles = 304;

// releases the pio from waiting on a start bit that never comes, e.g. no GBA connected
static struct k_timer g_rxWatchdog;
static constexpr k_timeout_t rxWatchdogTimeout = K_MSEC(1);

static uint8_t g_sdPin = 0;
static uint8_t g_sdCandidate = 0;
static bool g_sdLearning = true;

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

#pragma push_macro("pio0")
#undef pio0
uint32_t SC_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 0);
uint32_t SI_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 1);
uint32_t SO_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 2);
uint32_t SD_GBA_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 3);
uint32_t SD_GBC_pin = DT_RPI_PICO_PIO_PIN_BY_NAME(DT_CHILD(DT_NODELABEL(pio0), piolink), default, 0, link_pins, 4);
#pragma pop_macro("pio0")

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void assignGpioToPio();

void link_disable();

void link_enable();

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// SD Pin Helpers
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static inline uint32_t sdPinsSpread(uint16_t value)
{
    uint32_t x = value;
    x = (x | (x << 8)) & 0x00FF00FFu;
    x = (x | (x << 4)) & 0x0F0F0F0Fu;
    x = (x | (x << 2)) & 0x33333333u;
    x = (x | (x << 1)) & 0x55555555u;
    return x | (x << 1);
}


static inline uint16_t sdPinsDecode(uint32_t raw)
{
    uint32_t x = raw & (raw >> 1) & 0x55555555u;
    x = (x | (x >> 1)) & 0x33333333u;
    x = (x | (x >> 2)) & 0x0F0F0F0Fu;
    x = (x | (x >> 4)) & 0x00FF00FFu;
    x = (x | (x >> 8)) & 0x0000FFFFu;
    x = ((x & 0x5555u) << 1) | ((x & 0xAAAAu) >> 1);
    x = ((x & 0x3333u) << 2) | ((x & 0xCCCCu) >> 2);
    x = ((x & 0x0F0Fu) << 4) | ((x & 0xF0F0u) >> 4);
    x = ((x & 0x00FFu) << 8) | ((x & 0xFF00u) >> 8);
    return (uint16_t)x;
}

static inline uint8_t sdPinsFramePin(uint32_t raw)
{
    const uint32_t low = ~raw;
    const int gp3 = (low & 0x55555555u) != 0;
    const int gp4 = (low & 0xAAAAAAAAu) != 0;
    if (gp3 == gp4) return 0;
    return gp3 ? SD_PIN_GP3 : SD_PIN_GP4;
}

static void setSdPin(uint8_t pin)
{
    g_sdPin = pin;
    gpio_set_oeover(SD_GBA_pin, (pin & SD_PIN_GP3) ? GPIO_OVERRIDE_NORMAL : GPIO_OVERRIDE_LOW);
    gpio_set_oeover(SD_GBC_pin, (pin & SD_PIN_GP4) ? GPIO_OVERRIDE_NORMAL : GPIO_OVERRIDE_LOW);
}

static void learnSdPin(uint32_t raw)
{
    const uint8_t pin = sdPinsFramePin(raw);
    if (pin == 0) return;

    if (pin == g_sdPin)
    {
        g_sdCandidate = 0;
        return;
    }

    if (pin == g_sdCandidate)
    {
        setSdPin(pin);
        g_sdCandidate = 0;
    }
    else
    {
        setSdPin(0);
        g_sdCandidate = pin;
    }
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// IRQ Watchdog
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void startRxWatchdog()
{
    k_timer_start(&g_rxWatchdog, rxWatchdogTimeout, K_NO_WAIT);
    pio_interrupt_clear(g_pio.device, START_WATCHDOG);
}

static void rxWatchdogExpired(struct k_timer* timer)
{
    (void)timer;

    const uint32_t instruction = g_pio.device->sm[g_pio.id].instr;

    // Master: polls both SD pins (`in pins, 2` / `mov x, ~isr` / `jmp !x`) while the ISR is all ones.
    // Clearing the ISR ends the poll at its next check; the receive then samples the idle line, 0xFFFF.
    const bool isPoll = instruction == pio_encode_in(pio_pins, 2)
                     || instruction == pio_encode_mov_not(pio_x, pio_isr)
                     || (instruction & 0xE0E0) == pio_encode_jmp_not_x(0);
    if (isPoll) pio_sm_exec(g_pio.device, g_pio.id, pio_encode_mov(pio_isr, pio_null));
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// IRQ
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void pioIsr_prepareTransmission()
{
    startRxWatchdog();
    if (g_transmitCallback) g_nextTransmission = g_transmitCallback(g_transmitUserData);

    // pio makes use of the fact that in master 0 is alwyas our seat
    pio_sm_put(g_pio.device, g_pio.id, g_nextTransmission.seatNumber - 1u);
    for (uint16_t value : g_nextTransmission.txValues)
    {
        pio_sm_put(g_pio.device, g_pio.id, sdPinsSpread(value));
    }
    
    pio_interrupt_clear(g_pio.device, PREP_NEXT_TRANSMISSION_FLAG);
}

static void pioIsr_done()
{
    pio_interrupt_clear(g_pio.device, RX_TX_DONE_FLAG);

    k_timer_stop(&g_rxWatchdog);

    // Gap and PREP window as two 16-bit counts. Slaves discard this word.
    const uint32_t gap = g_nextTransmission.timingUs > 0xFFFFu ? 0xFFFFu : g_nextTransmission.timingUs;
    pio_sm_put(g_pio.device, g_pio.id, gap | (prepWindowCycles << 16));

    // The master starts at its DONE step, so its first DONE has no word to read.
    if (pio_sm_is_rx_fifo_empty(g_pio.device, g_pio.id)) return;
    
    const uint32_t raw = pio_sm_get(g_pio.device, g_pio.id);
    if (g_sdLearning) learnSdPin(raw);
    g_wordCount++;
    const uint16_t rxData = sdPinsDecode(raw);
    if (g_receiveCallback) g_receiveCallback(rxData, g_receiveUserData);
    if (g_transiveDoneCallback) g_transiveDoneCallback(rxData, g_nextTransmission.txValues, g_transiveDoneUserdata);
}

static void pioIsr(const void* arg)
{
    (void)arg;

    if (pio_interrupt_get(g_pio.device, PREP_NEXT_TRANSMISSION_FLAG))
    {
        pioIsr_prepareTransmission();
    }

    if (pio_interrupt_get(g_pio.device, RX_TX_DONE_FLAG))
    {
        pioIsr_done();
    }
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

void link_disable()
{
    g_wordCount = 0;
    k_timer_stop(&g_rxWatchdog);
    pio_sm_set_enabled(g_pio.device, g_pio.id, false);
    pio_clear_instruction_memory(g_pio.device);
    pio_sm_restart(g_pio.device, g_pio.id);
    pio_sm_clear_fifos(g_pio.device, g_pio.id);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_enable()
{
    pio_sm_set_enabled(g_pio.device, g_pio.id, true);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_configureProgram(const pio_program_t* prgramm, uint32_t warp, uint32_t wrapTarget)
{
    g_wordCount = 0;
    uint32_t offset = pio_add_program(g_pio.device, prgramm);
    sm_config_set_wrap(&g_config, offset + wrapTarget, offset + warp);
    assignGpioToPio();

    // The programs wait for both SD pins to be high, so the one without a cable must not float.
    gpio_pull_up(SD_GBA_pin);
    gpio_pull_up(SD_GBC_pin);

    // The SD pin learned in an earlier session stays until link_forgetSdPin: a GBA already in its
    // handshake in seat 2 or 3 does not answer a master that leaves SD released. assignGpioToPio
    // cleared the pin overrides, so setSdPin applies them again.
    g_sdCandidate = 0;
    setSdPin(g_sdPin);

    sm_config_set_set_pins(&g_config, SC_pin, 5);
    sm_config_set_out_pins(&g_config, SD_GBA_pin, 2);
    sm_config_set_in_pins(&g_config, SD_GBA_pin);

    pio_sm_init(g_pio.device, g_pio.id, offset, &g_config);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

uint32_t link_getPin(enum LinkPin pin)
{
    switch(pin)
    {
        case SC: return SC_pin;
        case SI: return SI_pin;
        case SO: return SO_pin;
        case SD_GBA: return SD_GBA_pin;
        case SD_GBC: return SD_GBC_pin;
      break;
    }
    return 0;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

uint32_t link_receivedWordCount() { return g_wordCount; }

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_enableBothSdPins()
{
    g_sdLearning = false;
    setSdPin(SD_PIN_GP3 | SD_PIN_GP4);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_forgetSdPin()
{
    g_sdLearning = true;
    g_sdCandidate = 0;
    g_sdPin = 0;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

uint8_t link_readPartnerPins()
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

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void assignGpioToPio()
{
    pio_gpio_init(g_pio.device, SC_pin);
    pio_gpio_init(g_pio.device, SI_pin);
    pio_gpio_init(g_pio.device, SO_pin);
    pio_gpio_init(g_pio.device, SD_GBA_pin);
    pio_gpio_init(g_pio.device, SD_GBC_pin);
}

static int init()
{
    const struct device* dev = DEVICE_DT_GET(DT_PROP(DT_NODELABEL(pio_link), pio));

    const struct pinctrl_dev_config* config = linkPinctrl_getConfig();
    g_pio.device = pio_rpi_pico_get_pio(dev);

    pio_rpi_pico_allocate_sm(dev, &g_pio.id);

    k_timer_init(&g_rxWatchdog, rxWatchdogExpired, NULL);

    IRQ_CONNECT(PIO0_IRQ_0 , 0, pioIsr, NULL, 0);

    irq_enable(PIO0_IRQ_0);

    pio_set_irq0_source_enabled(g_pio.device, pis_interrupt0, true);
    pio_set_irq0_source_enabled(g_pio.device, pis_interrupt1, true);
    int ret = pinctrl_apply_state(config, PINCTRL_STATE_DEFAULT);

    link_disable();

    assignGpioToPio();

    g_config = pio_get_default_sm_config();

    sm_config_set_out_shift(&g_config, true, false, 32);
    sm_config_set_in_shift(&g_config, false, false, 32);
    sm_config_set_mov_status(&g_config, STATUS_TX_LESSTHAN, 1);

    sm_config_set_clkdiv(&g_config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200

    return ret;
}

SYS_INIT(init, APPLICATION, 1);
