extern "C"
{
    #include <zephyr/kernel.h>

    #include "hardware/pio.h"
    #include "hardware/gpio.h"

    #include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>
    #include <zephyr/drivers/pinctrl.h>
    #include "linkPinctrl.h"
}

#include "./linkLayer.hpp"


#include "./cableDetection/cableDetection.hpp"

#define PREP_NEXT_TRANSMISSION_FLAG 0
#define RX_TX_DONE_FLAG 1
#define START_WATCHDOG 2

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

static enum CableType g_configuredCable;

static uint32_t g_wordCount = 0;

uint16_t g_lastTxValue = 0x00;

static struct NextTransmit g_nextTransmission = {};

// releases the pio from waiting on a start bit that never comes, e.g. no GBA connected
static struct k_timer g_rxWatchdog;
static constexpr k_timeout_t rxWatchdogTimeout = K_MSEC(1);

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

static void configureGBA();

static void configureGBC();

void assignGpioToPio();

static uint16_t reverse16Bit(uint16_t x);

void link_disable();

void link_enable();

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// IRQ
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void pioIsr_prepareTransmission()
{
    if (g_transmitCallback) g_nextTransmission = g_transmitCallback(g_transmitUserData);
    pio_sm_put(g_pio.device, g_pio.id, g_nextTransmission.seatNumber);
    for (uint16_t value : g_nextTransmission.txValues)
    {
        pio_sm_put(g_pio.device, g_pio.id, value);
    }
    
    pio_interrupt_clear(g_pio.device, PREP_NEXT_TRANSMISSION_FLAG);
}

static void pioIsr_startWatchdog()
{
    k_timer_start(&g_rxWatchdog, rxWatchdogTimeout, K_NO_WAIT);
    pio_interrupt_clear(g_pio.device, START_WATCHDOG);
}

static void rxWatchdogExpired(struct k_timer* timer)
{
    (void)timer;

    const uint32_t currentInstruction = g_pio.device->sm[g_pio.id].instr;
    const bool isWait = (currentInstruction & 0xE000) == 0x2000;
    if (!isWait) return;

    const uint32_t pc = pio_sm_get_pc(g_pio.device, g_pio.id);
    pio_sm_exec(g_pio.device, g_pio.id, pio_encode_jmp(pc + 1));
}

static void pioIsr_done()
{
    k_timer_stop(&g_rxWatchdog);
    g_wordCount++;
    pio_sm_put(g_pio.device, g_pio.id, g_nextTransmission.timingUs);
    uint16_t rxData = pio_sm_get(g_pio.device, g_pio.id);
    rxData = reverse16Bit(rxData);
    if (g_receiveCallback) g_receiveCallback(rxData, g_receiveUserData);
    if (g_transiveDoneCallback) g_transiveDoneCallback(rxData, g_nextTransmission.txValues, g_transiveDoneUserdata);
    pio_interrupt_clear(g_pio.device, RX_TX_DONE_FLAG);
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

    if (pio_interrupt_get(g_pio.device, START_WATCHDOG))
    {
        pioIsr_startWatchdog();
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

void link_startTransive() {}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_disable()
{
    //FIXME breaches layers but good enoght for now, 
    // any mode change will disable it and mode that need it can enable it again
    cableDetection_enableSlaveMonitoring(false);

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
        case SD: return g_configuredCable == GBA ? SD_GBA_pin : SD_GBC_pin;
        case SD_GBA: return SD_GBA_pin;
        case SD_GBC: return SD_GBC_pin;
      break;
    }
    return 0;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

uint32_t link_receivedWordCount() { return g_wordCount; }

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_setPioPinDirs(uint32_t pin, enum LinkPinDir direction)
{
    bool isOut = (direction == PIN_DOR_OUT);
    pio_sm_set_consecutive_pindirs(g_pio.device, g_pio.id, pin, 1, isOut);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_configureCableType(enum CableType type)
{
    switch(type)
    {
        case GBA: configureGBA(); break;
        case GBC: configureGBC(); break;
    }
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

uint16_t reverse16Bit(uint16_t x)
{
	x = ((x & 0x5555) << 1) | ((x & 0xAAAA) >> 1);
	x = ((x & 0x3333) << 2) | ((x & 0xCCCC) >> 2);
	x = ((x & 0x0F0F) << 4) | ((x & 0xF0F0) >> 4);
	return (x << 8) | (x >> 8);
}

static void configureGBA()
{
    g_configuredCable = GBA;
    sm_config_set_set_pins(&g_config, SC_pin, 4);
    sm_config_set_out_pins(&g_config, SD_GBA_pin, 1);
    sm_config_set_in_pins(&g_config, SD_GBA_pin);
    sm_config_set_jmp_pin(&g_config, SD_GBA_pin);

    pio_sm_init(g_pio.device, g_pio.id, 0, &g_config);
}

static void configureGBC()
{
    g_configuredCable = GBC;
    sm_config_set_set_pins(&g_config, SC_pin, 5);
    sm_config_set_out_pins(&g_config, SD_GBC_pin, 1);
    sm_config_set_in_pins(&g_config, SD_GBC_pin);
    sm_config_set_jmp_pin(&g_config, SD_GBC_pin);

    pio_sm_init(g_pio.device, g_pio.id, 0, &g_config);
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
    pio_set_irq0_source_enabled(g_pio.device, pis_interrupt2, true);

    int ret = pinctrl_apply_state(config, PINCTRL_STATE_DEFAULT);

    link_disable();

    assignGpioToPio();
    
    configureGBC();

    gpio_pull_up(link_getPin(SD));

    g_config = pio_get_default_sm_config();

    sm_config_set_out_shift(&g_config, true, false, 16);
    sm_config_set_in_shift(&g_config, false, false, 16);
    sm_config_set_mov_status(&g_config, STATUS_TX_LESSTHAN, 1);

    sm_config_set_clkdiv(&g_config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200

    pio_sm_init(g_pio.device, g_pio.id, 0, &g_config);

    return ret;
}

SYS_INIT(init, APPLICATION, 1);
