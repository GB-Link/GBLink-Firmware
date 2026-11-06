#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/spinlock.h>

#include  <zephyr/drivers/dma.h>

#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_bus.h"
#include "zephyr/dt-bindings/gpio/gpio.h"

#include "linkLayer.h"

#define ARR_VALUE 416
#define GPIO_PIN_DEBUG 1

//SC -> always trasmission start -> master pulls, slaves listens
#define GPIO_SC_PIN_MASTER_START 4

//SD -> always Data -> master TX/RX, slave RX/TX half duplex
#define GPIO_SD_PIN_DATA_INPUT_OUTPUT 11 

//SI -> this is SI <- SO, nc for master since we know we are master when we are in master mode; in slave mode tells us when to TX
#define GPIO_SI_PIN_SLAVE_GETS_SELECTED 8 

//SO -> this is SO -> SI, tell slave when to TX in master mode; pull low to activate master mode of GBA
#define GPIO_SO_PIN_MASTER_SELECTS_SLAVE 12 

#define BSSR_SET (1 << GPIO_SD_PIN_DATA_INPUT_OUTPUT)
#define BSSR_RESET (1 << (GPIO_SD_PIN_DATA_INPUT_OUTPUT + 16))

#define GPIO_PORT_B 0x48000400

#define DMA_CHANNEL_RX 3
#define DMA_CHANNEL_TX 5

#define DMA_TIMER_RX TIM3
#define DMA_TIMER_TX TIM15

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void* g_receiveUserData = NULL;
static LegacyReceiveHandler g_receiveCallback = NULL;

void* g_transmitUserData = NULL;
static LegacyTransmitHandler g_transmitCallback = NULL;

void* g_transiveDoneUserdata = NULL;
static LegacyTransiveDoneHandler g_transiveDoneCallback = NULL;

static enum LinkMode g_mode = SLAVE;

struct k_timer g_timeout;

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static struct gpio_callback g_linkStartReceiveSlaveCallback;
static struct gpio_callback g_linkStartReceiveMasterCallback;

const struct device* gpioIO = DEVICE_DT_GET(DT_NODELABEL(gpiob));
const struct device* dma = DEVICE_DT_GET(DT_NODELABEL(dma1));

static uint32_t tx_data[18] = {};
static uint32_t rx_data[17] = {};

static uint16_t g_linkReceived = 0;
static uint16_t g_linkTransmit = 0;

static const uint32_t mask = 1 << GPIO_SD_PIN_DATA_INPUT_OUTPUT;

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// DMA
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static inline void link_transmitToDma()
{
    tx_data[1] = (g_linkTransmit & (1 << 0))  ? BSSR_SET : BSSR_RESET;
    tx_data[2] = (g_linkTransmit & (1 << 1))  ? BSSR_SET : BSSR_RESET;
    tx_data[3] = (g_linkTransmit & (1 << 2))  ? BSSR_SET : BSSR_RESET;
    tx_data[4] = (g_linkTransmit & (1 << 3))  ? BSSR_SET : BSSR_RESET;
    tx_data[5] = (g_linkTransmit & (1 << 4))  ? BSSR_SET : BSSR_RESET;
    tx_data[6] = (g_linkTransmit & (1 << 5))  ? BSSR_SET : BSSR_RESET;
    tx_data[7] = (g_linkTransmit & (1 << 6))  ? BSSR_SET : BSSR_RESET;
    tx_data[8] = (g_linkTransmit & (1 << 7))  ? BSSR_SET : BSSR_RESET;
    tx_data[9] = (g_linkTransmit & (1 << 8))  ? BSSR_SET : BSSR_RESET;
    tx_data[10] = (g_linkTransmit & (1 << 9))  ? BSSR_SET : BSSR_RESET;
    tx_data[11] = (g_linkTransmit & (1 << 10)) ? BSSR_SET : BSSR_RESET;
    tx_data[12] = (g_linkTransmit & (1 << 11)) ? BSSR_SET : BSSR_RESET;
    tx_data[13] = (g_linkTransmit & (1 << 12)) ? BSSR_SET : BSSR_RESET;
    tx_data[14] = (g_linkTransmit & (1 << 13)) ? BSSR_SET : BSSR_RESET;
    tx_data[15] = (g_linkTransmit & (1 << 14)) ? BSSR_SET : BSSR_RESET;
    tx_data[16] = (g_linkTransmit & (1 << 15)) ? BSSR_SET : BSSR_RESET;
}

static inline void link_dmaToReceived()
{
    if (rx_data[1] & mask) g_linkReceived |= (1 << 0);
    if (rx_data[2] & mask) g_linkReceived |= (1 << 1);
    if (rx_data[3] & mask) g_linkReceived |= (1 << 2);
    if (rx_data[4] & mask) g_linkReceived |= (1 << 3);

    if (rx_data[5] & mask) g_linkReceived |= (1 << 4);
    if (rx_data[6] & mask) g_linkReceived |= (1 << 5);
    if (rx_data[7] & mask) g_linkReceived |= (1 << 6);
    if (rx_data[8] & mask) g_linkReceived |= (1 << 7);

    if (rx_data[9] & mask) g_linkReceived |= (1 << 8);
    if (rx_data[10] & mask) g_linkReceived |= (1 << 9);
    if (rx_data[11] & mask) g_linkReceived |= (1 << 10);
    if (rx_data[12] & mask) g_linkReceived |= (1 << 11);

    if (rx_data[13] & mask) g_linkReceived |= (1 << 12);
    if (rx_data[14] & mask) g_linkReceived |= (1 << 13);
    if (rx_data[15] & mask) g_linkReceived |= (1 << 14);
    if (rx_data[16] & mask) g_linkReceived |= (1 << 15);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void dmaCompleteTransmit(const struct device *dev, void *user_data, uint32_t channel, int status);
static void dmaCompleteReceive(const struct device *dev, void *user_data, uint32_t channel, int status);

static struct dma_block_config tx_config = {
    .dest_address = (uint32_t)(GPIO_PORT_B | 0x18), // GPIOB->BSRR
    .source_address = (uint32_t)tx_data,
    .block_size = sizeof(tx_data),
    .source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
    .dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE
};

static struct dma_config dma_cfg_tx = {
    .channel_direction = MEMORY_TO_PERIPHERAL,
    .complete_callback_en = 0,
    .error_callback_dis = 1,
    .source_data_size = sizeof(uint32_t),
    .dest_data_size = sizeof(uint32_t),
    .source_burst_length = 1,
    .dest_burst_length = 1,
    .block_count = 1,
    .channel_priority = 0x03,
    .dma_callback = &dmaCompleteTransmit,
    .head_block = &tx_config
};

static struct dma_block_config rx_config = {
    .dest_address = (uint32_t)rx_data,
    .source_address = (uint32_t) (GPIO_PORT_B | 0x10), // GPIOBs->IDR
    .block_size = sizeof(rx_data),
    .source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
    .dest_addr_adj = DMA_ADDR_ADJ_INCREMENT
};

static struct dma_config dma_cfg_rx = {
    .channel_direction = PERIPHERAL_TO_MEMORY,
    .complete_callback_en = 0,
    .error_callback_dis = 1,
    .source_data_size = sizeof(uint32_t),
    .dest_data_size = sizeof(uint32_t),
    .source_burst_length = 1,
    .dest_burst_length = 1,
    .block_count = 1,
    .channel_priority = 0x00,
    .dma_callback = &dmaCompleteReceive,
    .head_block = &rx_config
};

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static void dmaCompleteTransmit(const struct device *dev, void *user_data, uint32_t channel, int status)
{
    g_linkReceived = 0;

    switch (g_mode)
    {
        case SLAVE:
            if (g_transiveDoneCallback != NULL) g_transiveDoneCallback(g_transiveDoneUserdata);
            
            k_timer_stop(&g_timeout);

            // Prepare state for next recive
            gpio_pin_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INPUT | GPIO_PULL_UP);
            gpio_pin_interrupt_configure(gpioIO, GPIO_SC_PIN_MASTER_START, GPIO_INT_EDGE_FALLING);

            LL_TIM_DisableCounter(DMA_TIMER_TX);
            LL_TIM_SetCounter(DMA_TIMER_TX, 0);

            LL_TIM_DisableCounter(DMA_TIMER_RX);
            LL_TIM_SetCounter(DMA_TIMER_RX, 0);

            dma_config(dma, DMA_CHANNEL_RX, &dma_cfg_rx);
            dma_start(dma, DMA_CHANNEL_RX);

            dma_config(dma, DMA_CHANNEL_TX, &dma_cfg_tx);
            break;

        case MASTER:
            gpio_pin_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INPUT | GPIO_PULL_UP);
            gpio_pin_set(gpioIO, GPIO_SO_PIN_MASTER_SELECTS_SLAVE, GPIO_ACTIVE_HIGH);
            gpio_pin_interrupt_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INT_EDGE_FALLING);
            break;
    }
}

static void dmaCompleteReceive(const struct device *dev, void *user_data, uint32_t channel, int status)
{
    int lock = irq_lock();

    link_dmaToReceived();
    if (g_receiveCallback != NULL) g_receiveCallback(g_linkReceived, g_receiveUserData);

    switch (g_mode)
    {
        case SLAVE:
            // normally we would have to check GPIO_SI_PIN_SLAVE_GETS_SELECTED here, but since the end callback of
            // DMA takes so long and MCU is slow, we can just start to transmit here
            // Will break on faster MCUs
            if (g_transmitCallback != NULL) g_linkTransmit = g_transmitCallback(g_transmitUserData);
            link_transmitToDma();
            gpio_pin_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_OUTPUT_HIGH);
            dma_start(dma, DMA_CHANNEL_TX);
            LL_TIM_EnableCounter(DMA_TIMER_TX);
            break;
            
        case MASTER:
            gpio_pin_interrupt_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INT_DISABLE);
            gpio_pin_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_OUTPUT_HIGH);

            k_timer_stop(&g_timeout);

            LL_TIM_DisableCounter(TIM15);
            LL_TIM_SetCounter(TIM15, 0);

            LL_TIM_DisableCounter(TIM3);
            LL_TIM_SetCounter(TIM3, 0);

            if (g_transiveDoneCallback != NULL) g_transiveDoneCallback(g_transiveDoneUserdata);

            gpio_pin_set(gpioIO, GPIO_SC_PIN_MASTER_START, GPIO_ACTIVE_LOW);
            gpio_pin_set(gpioIO, GPIO_SO_PIN_MASTER_SELECTS_SLAVE, GPIO_ACTIVE_LOW);
            break;
    }
    irq_unlock(lock);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// Interface
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_setLegacyTransmitCallback(LegacyTransmitHandler cb, void* userData) 
{
    g_transmitUserData = userData;
    g_transmitCallback = cb;
}

void link_setLegacyReceiveCallback(LegacyReceiveHandler cb, void* userData) 
{
    g_receiveUserData = userData;
    g_receiveCallback = cb;
}

void link_setLegacyTransiveDoneCallback(LegacyTransiveDoneHandler cb, void* user_data)
{
    g_transiveDoneUserdata = user_data;
    g_transiveDoneCallback = cb;
}

enum LinkMode link_getMode()
{
    return g_mode;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_startTransive()
{
    int lock = irq_lock();
    k_timer_start(&g_timeout, K_USEC(600), K_NO_WAIT);

    if (LL_TIM_IsActiveFlag_UPDATE(TIM16)) LL_TIM_ClearFlag_UPDATE(TIM16);
    if (g_mode != MASTER) return;

    g_linkTransmit = g_transmitCallback(g_transmitUserData);
    
    link_transmitToDma();

    dma_config(dma, DMA_CHANNEL_TX, &dma_cfg_tx);
    dma_start(dma, DMA_CHANNEL_TX);
    LL_TIM_EnableCounter(TIM15);

    gpio_pin_set(gpioIO, GPIO_SC_PIN_MASTER_START, GPIO_ACTIVE_HIGH);
    
    gpio_pin_toggle(gpioIO, GPIO_PIN_DEBUG);
    gpio_pin_toggle(gpioIO, GPIO_PIN_DEBUG);
    dma_config(dma, DMA_CHANNEL_RX, &dma_cfg_rx);
    dma_start(dma, DMA_CHANNEL_RX);

    irq_unlock(lock);
}

static void startReceiveSlave(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    LL_TIM_EnableCounter(DMA_TIMER_RX);
    k_timer_start(&g_timeout, K_USEC(500), K_NO_WAIT);
    gpio_pin_interrupt_configure(gpioIO, GPIO_SC_PIN_MASTER_START, GPIO_INT_DISABLE);
}

static void startReceiveMaster(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    LL_TIM_EnableCounter(DMA_TIMER_RX);
    gpio_pin_interrupt_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INT_DISABLE);
}

static void setUpDMATimer()
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM15);
    LL_TIM_SetClockSource(DMA_TIMER_TX, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetCounterMode(DMA_TIMER_TX, LL_TIM_COUNTERDIRECTION_UP);
    LL_TIM_SetUpdateSource(DMA_TIMER_TX, LL_TIM_UPDATESOURCE_COUNTER);
    LL_TIM_SetAutoReload(DMA_TIMER_TX, ARR_VALUE);
    LL_TIM_SetCounter(DMA_TIMER_TX, 0);
    LL_TIM_SetPrescaler(DMA_TIMER_TX, 0);
    LL_TIM_EnableDMAReq_UPDATE(DMA_TIMER_TX);
    LL_TIM_EnableIT_UPDATE(DMA_TIMER_TX);
    LL_TIM_GenerateEvent_UPDATE(DMA_TIMER_TX);
    LL_TIM_EnableAllOutputs(DMA_TIMER_TX);
    LL_TIM_SetTriggerOutput(DMA_TIMER_TX, LL_TIM_TRGO_UPDATE);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_TIM_SetClockSource(DMA_TIMER_RX, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetCounterMode(DMA_TIMER_RX, LL_TIM_COUNTERDIRECTION_UP);
    LL_TIM_SetUpdateSource(DMA_TIMER_RX, LL_TIM_UPDATESOURCE_COUNTER);
    LL_TIM_SetAutoReload(DMA_TIMER_RX, ARR_VALUE);
    LL_TIM_SetCounter(DMA_TIMER_RX, 0);
    LL_TIM_SetPrescaler(DMA_TIMER_RX, 0);
    LL_TIM_EnableDMAReq_UPDATE(DMA_TIMER_RX);
    LL_TIM_EnableIT_UPDATE(DMA_TIMER_RX);
    LL_TIM_GenerateEvent_UPDATE(DMA_TIMER_RX);
    LL_TIM_EnableAllOutputs(DMA_TIMER_RX);
    LL_TIM_SetTriggerOutput(DMA_TIMER_RX, LL_TIM_TRGO_UPDATE);
}

void link_changeMode(enum LinkMode mode)
{
    if (g_mode == mode) return;
    
    switch (mode)
    {
        case SLAVE:
            gpio_pin_configure(gpioIO, GPIO_SC_PIN_MASTER_START, GPIO_INPUT | GPIO_PULL_UP);
            gpio_pin_configure(gpioIO, GPIO_SI_PIN_SLAVE_GETS_SELECTED, GPIO_INPUT | GPIO_PULL_UP);
            gpio_pin_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INPUT | GPIO_PULL_UP);
            //This goes to SI of GBA, pull to GND to select GBA master mode
            gpio_pin_configure(gpioIO, GPIO_SO_PIN_MASTER_SELECTS_SLAVE, GPIO_OUTPUT_LOW);

            gpio_pin_interrupt_configure(gpioIO, GPIO_SC_PIN_MASTER_START, GPIO_INT_EDGE_FALLING);
            gpio_pin_interrupt_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INT_DISABLE);
            LL_TIM_DisableCounter(TIM16);
            break;

        case MASTER:
            gpio_pin_configure(gpioIO, GPIO_SC_PIN_MASTER_START, GPIO_OUTPUT_HIGH);
            gpio_pin_configure(gpioIO, GPIO_SO_PIN_MASTER_SELECTS_SLAVE, GPIO_OUTPUT_HIGH);
            gpio_pin_configure(gpioIO, GPIO_SI_PIN_SLAVE_GETS_SELECTED, GPIO_OUTPUT_HIGH);
            gpio_pin_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INPUT | GPIO_PULL_UP);

            gpio_pin_interrupt_configure(gpioIO, GPIO_SC_PIN_MASTER_START, GPIO_INT_DISABLE);
            gpio_pin_interrupt_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INT_DISABLE);

            break;
    }

    g_mode = mode;
}

static void link_timeout(struct k_timer* timer)
{
    switch(g_mode)
    {
        case MASTER:
        {
            gpio_pin_toggle(gpioIO, GPIO_PIN_DEBUG);
            gpio_pin_toggle(gpioIO, GPIO_PIN_DEBUG);
            return dmaCompleteReceive(NULL, NULL, 0, 0);
        } 
        case SLAVE: return dmaCompleteTransmit(NULL, NULL, 0, 0);

    }
}

static int link_init()
{
    tx_data[0] = BSSR_RESET;
    tx_data[17] = BSSR_SET;

    k_timer_init(&g_timeout, link_timeout, NULL);

    gpio_pin_configure(gpioIO, GPIO_SC_PIN_MASTER_START, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure(gpioIO, GPIO_SI_PIN_SLAVE_GETS_SELECTED, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INPUT | GPIO_PULL_UP);
    //This goes to SI of GBA, pull to GND to select GBA master mode
    gpio_pin_configure(gpioIO, GPIO_SO_PIN_MASTER_SELECTS_SLAVE, GPIO_OUTPUT_LOW);

    gpio_pin_configure(gpioIO, GPIO_PIN_DEBUG, GPIO_OUTPUT_HIGH);
    setUpDMATimer();

    gpio_init_callback(&g_linkStartReceiveSlaveCallback, startReceiveSlave, (1 << GPIO_SC_PIN_MASTER_START));
    gpio_add_callback(gpioIO, &g_linkStartReceiveSlaveCallback);
    gpio_pin_interrupt_configure(gpioIO, GPIO_SC_PIN_MASTER_START, GPIO_INT_EDGE_FALLING);

    gpio_init_callback(&g_linkStartReceiveMasterCallback, startReceiveMaster, (1 << GPIO_SD_PIN_DATA_INPUT_OUTPUT));
    gpio_add_callback(gpioIO, &g_linkStartReceiveMasterCallback);
    gpio_pin_interrupt_configure(gpioIO, GPIO_SD_PIN_DATA_INPUT_OUTPUT, GPIO_INT_DISABLE);

    return 0;
}

SYS_INIT(link_init, APPLICATION, 1);
