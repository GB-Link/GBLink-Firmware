#include "linkLayer.h"

#include "hardware/pio.h"
#include "hardware/gpio.h"

#include <zephyr/kernel.h>
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

// Child SD slots the SLAVE PIO drives per multiplayer frame. 1 = standard single
// child; 3 selects the multi-slot SLAVE PIO (one adapter impersonating 3 children
// = a 4-player frame). Set via link_setChildSlots() before link_changeMode(SLAVE).
static uint8_t g_childSlots = 1;

// Master-side multi-slot (joiner): when set, link_changeMode(MASTER) loads the
// master-multi PIO and the TX ISR feeds [count = txSlots-1, w0..w(txSlots-1)].
// txSlots = slots the adapter drives before the attached GBA's (last) slot = N-1.
static bool g_masterMulti = false;
static uint8_t g_masterN = 2;      // total players in the session
static uint8_t g_masterSeat = 1;   // the attached GBA's seat (1..N-1)

// Master-multi inter-frame pacing: the PIO blocks on its first `pull`, so the
// TX ISR stages the frame's words here and defers the FIFO fill by the timing
// the transmit callback asked for (NextTransmit.timingUs, in 540 ns PIO
// cycles, like the single-slot master's in-FIFO delay word). A timing of 0
// pushes immediately.
static struct k_timer g_masterMultiTimer;
static uint32_t g_mmPacked = 0;
static uint32_t g_mmWords[4] = { 0 };
static uint8_t g_mmCount = 0;

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

/* --- Multi-slot slave programs: receive the master word (slot 0), then transmit
 *     THREE child words back-to-back on SD (slots 1/2/3), holding SO low. A real
 *     master counts children by timing (GBATEK: "Transfer ends if next child does
 *     not output data after a certain time"), so one adapter emitting 3 UART words
 *     in succession presents as 3 children -> a 4-player frame. Same structure as
 *     pio_slave_*, with the transmit body wrapped in a Y=2 (3 iterations) loop.
 *     The TX FIFO is fed 3 words/frame by pioIsr_tx when g_childSlots == 3. */

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_multi_gba, 0, 20,
    (0xe000 | PIO_SD_GBA),            //  0: set    pins,    SD=HIGH (SO low -> GBA is master)
    0xe084,                           //  1: set    pindirs, SO=out  (SD in)
    0xe02f,                           //  2: set    x, 15
    0x2020,                           //  3: wait   0 pin 0  (master SC strobe)
    0xd701,                           //  4: irq    1 [23]   (stage 3 child words)
    0x4e01,                           //  5: in     pins, 1 [14]   (receive loop)
    0x0045,                           //  6: jmp    x--, 5
    0x8020,                           //  7: push   block          (master word)
    (0xe000 | PIO_SD_GBA),            //  8: set    pins,    SD=HIGH
    (0xe080 | PIO_SO | PIO_SD_GBA),   //  9: set    pindirs, SO|SD=out
    0xbf42,                           // 10: nop    [31]
    0xe042,                           // 11: set    y, 2           (3 child slots)
    0xe02f,                           // 12: set    x, 15          (child_loop)
    0x80a0,                           // 13: pull   block          (child word)
    0xf000,                           // 14: set    pins, 0 [16]   (UART start bit)
    0x6e01,                           // 15: out    pins, 1 [14]   (transmit loop)
    0x004f,                           // 16: jmp    x--, 15
    (0xf000 | PIO_SD_GBA),            // 17: set    pins,    SD=HIGH [16] (UART stop bit)
    0x008c,                           // 18: jmp    y--, 12        (next child slot)
    0xc000,                           // 19: irq    0              (frame done)
    0xbf42);                          // 20: nop    [31]

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_multi_gbc, 0, 20,
    (0xe000 | PIO_SD_GBC),
    0xe084,
    0xe02f,
    0x2020,
    0xd701,
    0x4e01,
    0x0045,
    0x8020,
    (0xe000 | PIO_SD_GBC),
    (0xe080 | PIO_SO | PIO_SD_GBC),
    0xbf42,
    0xe042,
    0xe02f,
    0x80a0,
    0xf000,
    0x6e01,
    0x004f,
    (0xf000 | PIO_SD_GBC),
    0x008c,
    0xc000,
    0xbf42);

/* 2-child variants (3-player): identical to pio_slave_multi_* but the child
 * loop runs twice (set y, 1 instead of set y, 2). */

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_multi2_gba, 0, 20,
    (0xe000 | PIO_SD_GBA),
    0xe084,
    0xe02f,
    0x2020,
    0xd701,
    0x4e01,
    0x0045,
    0x8020,
    (0xe000 | PIO_SD_GBA),
    (0xe080 | PIO_SO | PIO_SD_GBA),
    0xbf42,
    0xe041,                           // 11: set y, 1  (2 child slots)
    0xe02f,
    0x80a0,
    0xf000,
    0x6e01,
    0x004f,
    (0xf000 | PIO_SD_GBA),
    0x008c,
    0xc000,
    0xbf42);

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_multi2_gbc, 0, 20,
    (0xe000 | PIO_SD_GBC),
    0xe084,
    0xe02f,
    0x2020,
    0xd701,
    0x4e01,
    0x0045,
    0x8020,
    (0xe000 | PIO_SD_GBC),
    (0xe080 | PIO_SO | PIO_SD_GBC),
    0xbf42,
    0xe041,                           // 11: set y, 1  (2 child slots)
    0xe02f,
    0x80a0,
    0xf000,
    0x6e01,
    0x004f,
    (0xf000 | PIO_SD_GBC),
    0x008c,
    0xc000,
    0xbf42);

/* Master-side multi-slot, ARBITRARY SEAT: adapter is bus master, the attached
 * GBA is a child at seat cartSeat (1..N-1). One unified slot loop walks all N
 * slots (Y = slot countdown N-1..0; slot index = (N-1)-Y), transmitting a word
 * on SD at every slot EXCEPT the GBA's, where it forwards SO low and reads the
 * GBA's word (when Y == slotsAfter, held in X; X then set to a 31 sentinel).
 * Transmit counts 16 bits via OSR threshold ("jmp !osre") to free X. Assembled
 * from src/pio_firmware/pio_master_multi.pio via pioasm; SD-referencing SET ops
 * use the PIO_SD_* macro. ISR feeds [packed=(slotsAfter<<8)|(N-1), w0..w(N-2)]
 * (N entries, fits the 4-deep FIFO). configureMasterMulti sets out threshold=16. */

RPI_PICO_PIO_DEFINE_PROGRAM(pio_master_multi_gba, 0, 31,
    (0xe080 | PIO_SC | PIO_SO | PIO_SD_GBA),  //  0: set pindirs, SC|SO|SD out
    (0xe000 | PIO_SC | PIO_SO | PIO_SD_GBA),  //  1: set pins, SC|SO|SD high (idle)
    0xc001,                                   //  2: irq 1 (stage tx)
    0xe03f,                                   //  3: set x, 31
    0x1f44,                                   //  4: jmp x--, 4 [31] (wait/pace)
    0x80a0,                                   //  5: pull (packed)
    0x6048,                                   //  6: out y, 8  (Y = N-1)
    0x6028,                                   //  7: out x, 8  (X = slotsAfter)
    (0xe080 | PIO_SC | PIO_SO | PIO_SD_GBA),  //  8: set pindirs, SC|SO|SD out
    (0xe000 | PIO_SO | PIO_SD_GBA),           //  9: set pins, SO|SD (SC low)
    0x00b8,                                   // 10: jmp x!=y, 24 (do_transmit)
    (0xe000 | PIO_SD_GBA),                    // 11: set pins, SD (SO low = forward)
    (0xe080 | PIO_SC | PIO_SO),               // 12: set pindirs, SC|SO (SD input)
    0xe03f,                                   // 13: set x, 31
    0x074f,                                   // 14: jmp x--, 15 [7]
    0x0031,                                   // 15: jmp !x, 17
    0x00ce,                                   // 16: jmp pin, 14
    0xf62f,                                   // 17: set x, 15 [22]
    0x4e01,                                   // 18: in pins, 1 [14]
    0x0052,                                   // 19: jmp x--, 18
    0x9020,                                   // 20: push [16]
    (0xe080 | PIO_SC | PIO_SO | PIO_SD_GBA),  // 21: set pindirs, SC|SO|SD out
    0xe03f,                                   // 22: set x, 31 (sentinel)
    0x001d,                                   // 23: jmp 29
    0x80a0,                                   // 24: pull (slot word)
    (0xef00 | PIO_SO),                        // 25: set pins, SO [15] (start bit, SD low)
    0x6e01,                                   // 26: out pins, 1 [14]
    0x00fa,                                   // 27: jmp !osre, 26
    (0xef00 | PIO_SO | PIO_SD_GBA),           // 28: set pins, SO|SD [15] (stop bit)
    0x008a,                                   // 29: jmp y--, 10 (slot_loop)
    (0xfe00 | PIO_SO | PIO_SD_GBA),           // 30: set pins, SO|SD [30]
    0xc000);                                  // 31: irq 0

RPI_PICO_PIO_DEFINE_PROGRAM(pio_master_multi_gbc, 0, 31,
    (0xe080 | PIO_SC | PIO_SO | PIO_SD_GBC),
    (0xe000 | PIO_SC | PIO_SO | PIO_SD_GBC),
    0xc001,
    0xe03f,
    0x1f44,
    0x80a0,
    0x6048,
    0x6028,
    (0xe080 | PIO_SC | PIO_SO | PIO_SD_GBC),
    (0xe000 | PIO_SO | PIO_SD_GBC),
    0x00b8,
    (0xe000 | PIO_SD_GBC),
    (0xe080 | PIO_SC | PIO_SO),
    0xe03f,
    0x074f,
    0x0031,
    0x00ce,
    0xf62f,
    0x4e01,
    0x0052,
    0x9020,
    (0xe080 | PIO_SC | PIO_SO | PIO_SD_GBC),
    0xe03f,
    0x001d,
    0x80a0,
    (0xef00 | PIO_SO),
    0x6e01,
    0x00fa,
    (0xef00 | PIO_SO | PIO_SD_GBC),
    0x008a,
    (0xfe00 | PIO_SO | PIO_SD_GBC),
    0xc000);

/* Detect cable type by reading GP1 (SI pin).
 * GBA cable: GP1 hardwired to GND in cable — reads LOW
 * GBC cable: GP1 connected to GBA SO or floating — pull-up → reads HIGH
 *
 * Called once via link_detectCableType() when a GBA mode is selected
 * (before the GBA enters link mode and starts driving SO). */
static bool detect_gbc_cable(void)
{
    gpio_set_function(1, GPIO_FUNC_SIO);
    gpio_set_dir(1, GPIO_IN);
    gpio_pull_up(1);
    for (volatile int i = 0; i < 1000; i++);  /* settle ~10us */
    bool gbc = gpio_get(1);
    gpio_set_function(1, GPIO_FUNC_PIO0);
    return gbc;
}

static bool g_gbc_cable = false;

void link_detectCableType(void)
{
    g_gbc_cable = detect_gbc_cable();
}

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
static void pioIsr_done(const void* arg)
{
    (void)arg;
    uint16_t rxData = pio_sm_get(g_pio, g_sm);
    rxData = reverse_bit16(rxData);
    if (g_receiveCallback) g_receiveCallback(rxData, g_receiveUserData);
    if (g_transiveDoneCallback) g_transiveDoneCallback(rxData, g_lastTxValue, g_transiveDoneUserdata);
    pio_interrupt_clear(g_pio, TX_RX_DONE_IRQ);
}

static void masterMultiTimerFire(struct k_timer* timer)
{
    (void)timer;
    if (g_mode != MASTER || !g_masterMulti) return;
    pio_sm_put(g_pio, g_sm, g_mmPacked);
    for (uint8_t i = 0; i < g_mmCount; i++)
    {
        pio_sm_put(g_pio, g_sm, g_mmWords[i]);
    }
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

    if (g_mode == MASTER)
    {
        if (g_masterMulti)
        {
            // Stage [packed = (slotsAfter << 8) | (N-1), w0 .. w(N-2)] -- N-1
            // transmit words for the slots the adapter drives (all but the
            // GBA's seat) -- and fill the FIFO after the requested delay.
            const uint8_t nMinus1 = (uint8_t)(g_masterN - 1);
            const uint8_t slotsAfter = (uint8_t)(nMinus1 - g_masterSeat);
            g_mmPacked = ((uint32_t)slotsAfter << 8) | nMinus1;
            g_mmWords[0] = txValue.value;
            g_mmCount = nMinus1;
            g_lastTxValue = txValue.value;
            for (uint8_t slot = 1; slot < nMinus1; slot++)
            {
                struct NextTransmit extra = g_transmitCallback
                    ? g_transmitCallback(g_transmitUserData) : txValue;
                g_mmWords[slot] = extra.value;
            }

            if (txValue.timingUs == 0)
            {
                masterMultiTimerFire(NULL);
            }
            else
            {
                // timingUs is in 540 ns PIO cycles (see the single-slot master).
                k_timer_start(&g_masterMultiTimer,
                    K_USEC((txValue.timingUs * 54ULL) / 100ULL), K_NO_WAIT);
            }
        }
        else
        {
            pio_sm_put(g_pio, g_sm, txValue.timingUs);
            pio_sm_put(g_pio, g_sm, txValue.value);
            g_lastTxValue = txValue.value;
        }
    }
    else  // SLAVE (single or multi-slot)
    {
        pio_sm_put(g_pio, g_sm, txValue.value);
        g_lastTxValue = txValue.value;
        // Slave multi-slot (multi/multi2 PIO): stage the remaining child
        // words, one callback per slot.
        for (uint8_t slot = 1; slot < g_childSlots; slot++)
        {
            struct NextTransmit extra = g_transmitCallback
                ? g_transmitCallback(g_transmitUserData) : txValue;
            pio_sm_put(g_pio, g_sm, extra.value);
        }
    }
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
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);

    bool gbc = g_gbc_cable;

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

static void link_configureSlave()
{
    g_mode = SLAVE;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);

    bool gbc = g_gbc_cable;

	uint32_t offset = gbc
        ? pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_slave_gbc))
        : pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_slave_gba));

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
    pio_gpio_init(g_pio, SO_pin);
    pio_gpio_init(g_pio, SI_pin);

	sm_config_set_clkdiv(&sm_config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200

	sm_config_set_wrap(&sm_config,
			   offset + (gbc ? RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_gbc) : RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_gba)),
			   offset + (gbc ? RPI_PICO_PIO_GET_WRAP(pio_slave_gbc) : RPI_PICO_PIO_GET_WRAP(pio_slave_gba)));
    
	pio_sm_init(g_pio, g_sm, -1, &sm_config);
	pio_sm_set_enabled(g_pio, g_sm, true);
}

static void link_configureMasterMulti()
{
    g_mode = MASTER;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);

    bool gbc = g_gbc_cable;

    uint32_t offset = gbc
        ? pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_master_multi_gbc))
        : pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_master_multi_gba));

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

    // threshold 16: OSR empties (osre) after 16 data bits, which the master-multi
    // PIO uses ("jmp !osre") to count transmit bits without a scratch register.
    sm_config_set_out_shift(&sm_config, true, false, 16);
    sm_config_set_in_shift(&sm_config, false, false, 0);

    pio_gpio_init(g_pio, SD_pin);
    if (gbc) gpio_pull_up(SD_pin);
    pio_gpio_init(g_pio, SC_pin);
    pio_gpio_init(g_pio, SO_pin);
    pio_gpio_init(g_pio, SI_pin);

    sm_config_set_clkdiv(&sm_config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200

    sm_config_set_wrap(&sm_config,
        offset + (gbc ? RPI_PICO_PIO_GET_WRAP_TARGET(pio_master_multi_gbc) : RPI_PICO_PIO_GET_WRAP_TARGET(pio_master_multi_gba)),
        offset + (gbc ? RPI_PICO_PIO_GET_WRAP(pio_master_multi_gbc) : RPI_PICO_PIO_GET_WRAP(pio_master_multi_gba)));

    pio_sm_init(g_pio, g_sm, -1, &sm_config);
    pio_sm_set_enabled(g_pio, g_sm, true);
}

static void link_configureSlaveMulti()
{
    g_mode = SLAVE;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
    pio_sm_restart(g_pio, g_sm);
    pio_sm_clear_fifos(g_pio, g_sm);

    bool gbc = g_gbc_cable;

    // 3+ children (4-player) -> multi; 2 children (3-player) -> multi2.
    uint32_t offset;
    if (g_childSlots >= 3)
        offset = gbc ? pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_slave_multi_gbc))
                     : pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_slave_multi_gba));
    else
        offset = gbc ? pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_slave_multi2_gbc))
                     : pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(pio_slave_multi2_gba));

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
    pio_gpio_init(g_pio, SO_pin);
    pio_gpio_init(g_pio, SI_pin);

    sm_config_set_clkdiv(&sm_config, 67.816f); // ~540 ns per inst, 16 inst equal baud 115200

    // All four multi programs share the same length/wrap layout.
    sm_config_set_wrap(&sm_config,
        offset + RPI_PICO_PIO_GET_WRAP_TARGET(pio_slave_multi_gba),
        offset + RPI_PICO_PIO_GET_WRAP(pio_slave_multi_gba));

    pio_sm_init(g_pio, g_sm, -1, &sm_config);
    pio_sm_set_enabled(g_pio, g_sm, true);
}

static void link_disablePio()
{
    g_mode = DISABLED;
    k_timer_stop(&g_masterMultiTimer);
    g_childSlots = 1;  // reset so the next SLAVE mode defaults to a single child
    g_masterMulti = false;
    g_masterN = 2;
    g_masterSeat = 1;
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_clear_instruction_memory(g_pio);
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

void link_setChildSlots(uint8_t n)
{
    g_childSlots = n;
}

void link_setMasterMulti(uint8_t playerCount, uint8_t seat)
{
    g_masterMulti = true;
    g_masterN = playerCount;
    g_masterSeat = seat;
}

void link_changeMode(enum LinkMode mode)
{
    switch (mode)
    {
        case SLAVE:
            if (g_childSlots > 1) link_configureSlaveMulti();
            else link_configureSlave();
            break;
        case MASTER:
            if (g_masterMulti) link_configureMasterMulti();
            else link_configureMaster();
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

    k_timer_init(&g_masterMultiTimer, &masterMultiTimerFire, NULL);

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
