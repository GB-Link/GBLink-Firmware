#include "linkLayer_rfu.h"

#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

// Pin roles: GP0 = SC, GP1 = GBA SO (in), GP2 = GBA SI (out). SD (in, carries
// the game's AgbRFU_SoftReset pulse) lands on GP3 or GP4 depending on how the
// GBA end is wired; both are watched, so no cable selection applies here.
#define RFU_PIN_SC      0
#define RFU_PIN_RX      1
#define RFU_PIN_TX      2
#define RFU_PIN_SD_GBA  3
#define RFU_PIN_SD_GBC  4
#define RFU_PIN_NONE    0xFF

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// PIO programs (hand-assembled, see comments for the .pio equivalents).
// Shift config for both: left shift (MSB first, the GBA SIO wire order), no
// autopush/autopull, out_pins/set_pins = GP2 (count 1), in_pins = GP1.
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

/* rfu_slave32: GBA is bus master; edge-driven on SC so one program serves both
 * 256 kHz and 2 MHz (clkdiv 1.0). The idle-gap mirror (addr 3-4) holds SI(GP2) =
 * NOT SO(GP1) while SC(GP0) rests high, releasing librfu's command-phase
 * handshake_wait(1)/handshake_wait(0) busy-waits (the GBA polls SI between every
 * 32-bit word); it yields GP2 as soon as SC falls, and uses `mov` (OSR-safe) so
 * the staged response word survives into the bit loop.
 *   .wrap_target
 *   0: set  pins, 0        ; idle TX line low = "adapter ready" (one-shot)
 *   1: pull block          ; staged response (CPU keeps one word queued)
 *   2: set  x, 31
 *   3: mov  pins, ~pins    ; SI(GP2) = NOT SO(GP1) — continuous level mirror
 *   4: jmp  pin, 3         ; while SC(GP0) high keep mirroring; SC low -> fall
 *   5: wait 0 gpio 0       ; SC falling edge: both sides shift out   (bit:)
 *   6: out  pins, 1 [12]   ; drive at once, then ignore SC for ~100ns
 *   7: wait 1 gpio 0       ; SC rising edge: sample
 *   8: in   pins, 1 [6]    ; sample at once, then ignore SC for ~50ns
 *   9: jmp  x--, 5
 *  10: nop [31]           ; hold the last bit past the GBA's sample edge
 *  11: nop [31]
 *  12: push block
 *  13: irq  0
 *   .wrap
 * jmp_pin = SC (GP0), set in configureSlaveSm. SC idles high; the GBA-as-master
 * sets up on the falling edge and samples on the rising edge. The two nops after
 * the bit loop hold the last bit past that sample edge; SI is otherwise
 * released ~24ns after the final rising edge and the GBA reads the bit wrong.
 *
 * The delays on 6 and 8 blank SC after each edge. At clkdiv 1.0 the next `wait`
 * would otherwise be armed 16ns after an edge. Both ends change their data line
 * just after the falling edge, which a cable couples onto SC as a pulse, and a
 * slow or ringing edge crosses the input threshold more than once. Either is
 * taken for a whole clock: the word boundary slips by a bit and everything after
 * it is garbage until the state machine is next restarted. A half period at
 * 2 MHz is 238ns, and `wait` is level-sensitive, so a blanking window that
 * overruns an edge costs nothing: the level is still there when it ends.
 */
RPI_PICO_PIO_DEFINE_PROGRAM(rfu_slave32, 0, 13,
    0xE000,  //  0: set    pins, 0
    0x80A0,  //  1: pull   block
    0xE03F,  //  2: set    x, 31
    0xA008,  //  3: mov    pins, ~pins
    0x00C3,  //  4: jmp    pin, 3
    0x2000,  //  5: wait   0 gpio 0
    0x6C01,  //  6: out    pins, 1 [12]
    0x2080,  //  7: wait   1 gpio 0
    0x4601,  //  8: in     pins, 1 [6]
    0x0045,  //  9: jmp    x--, 5
    0xBF42,  // 10: nop    [31]
    0xBF42,  // 11: nop    [31]
    0x8020,  // 12: push   block
    0xC000); // 13: irq    0

/* rfu_master32: adapter clocks (wait-response delivery); side-set 1 = SC.
 * The side-set is non-optional: every instruction, including pio_sm_exec'd
 * ones, drives SC from bit 12, so only side-1 encodings may be forced here.
 * in_pins = GP1 (GBA SO), out_pins/set_pins = GP2 (SI). SC idles high.
 * clkdiv 54.25 -> 1 PIO cycle ≈ 434 ns; 15 cycles/bit ≈ 154 kHz.
 *
 * A 32-bit exchanger with no pin waits, so it cannot hang; all librfu
 * ready-handshaking happens in C between words (rfuProtocolSection), which
 * handles the word-1 / word-N asymmetry of the GBA's slave ISR.
 *
 * Bit geometry mirrors the command-phase slave loop: the GBA-as-slave shifts
 * its SO out on the falling edge (nothing is presented at idle; SO rests at the
 * SD level) and latches SI on the rising edge. Per bit: the falling edge and
 * our SI drive share a cycle (stable long before the rise), its SO is sampled
 * mid-low, and the rising edge latches our SI into the GBA.
 *   .side_set 1            ; SC: side 1 = HIGH (idle), side 0 = LOW
 *   .wrap_target
 *   0: pull block    side1        ; parked here between words (SC held high)
 *   1: set x,31      side1
 *   2: out pins,1    side0 [4]    ; FALLING edge + drive SI bit; GBA shifts SO
 *   3: in  pins,1    side0 [1]    ; mid-low: sample the SO bit it just shifted
 *   4: jmp x--,2     side1 [7]    ; RISING edge: GBA latches SI; hold high
 *   5: push block    side1        ; word read back from the GBA
 *   .wrap
 */
RPI_PICO_PIO_DEFINE_PROGRAM(rfu_master32, 0, 5,
    0x90A0,  //  0: pull   block        side 1
    0xF03F,  //  1: set    x, 31        side 1
    0x6401,  //  2: out    pins, 1      side 0 [4]
    0x4101,  //  3: in     pins, 1      side 0 [1]
    0x1742,  //  4: jmp    x--, 2       side 1 [7]
    0x9020); //  5: push   block        side 1

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

// io_bank0 raw interrupt latch bits for the edge events on a GPIO (4 status bits
// per GPIO in each INTR word; EDGE_LOW is bit 2 of the group, EDGE_HIGH bit 3).
#define RFU_EDGE_LOW_BIT(pin)   (1u << ((((pin) % 8) * 4) + 2))
#define RFU_EDGE_HIGH_BIT(pin)  (1u << ((((pin) % 8) * 4) + 3))

// How long SD has to stay high, without a falling edge, to count as the reset
// pulse. The pulse lasts about 1.1ms. What the neighbouring lines couple onto SD
// decays through the pad's pull-down (50-80k against a few hundred pF of cable,
// so 10-20us at the outside) or is cancelled by the aggressor's next edge.
#define RFU_SD_HOLD_US 40

// After the reset pulse the slave state machine starts over, once SD has been low
// for this long; the first clock of the ID exchange is more than 100us away when
// SD falls. The game raises the pulse from general-purpose mode, where its SC is
// an input, so the line floats on pull-ups while the pulse's own edges couple
// onto it, and a dip that reads as a clock shifts the word boundary by a bit.
// Nothing else would move it back: every ID word after it would arrive shifted
// and the game would reset and retry for good. A real adapter's shift register
// resets with the pulse.
#define RFU_SD_SETTLE_US 60
#define RFU_SD_PULSE_MAX_US 3000

static PIO g_pio = NULL;
static size_t g_smSlave = 0;
static size_t g_smMaster = 1;
static uint32_t g_offSlave = 0;
static uint32_t g_offMaster = 0;
static enum RfuRole g_role = RFU_ROLE_GBA_MASTER;
static bool g_enabled = false;
// Both candidate SD pins are watched: the one the cable does not use is left
// unconnected, and a pulled-down input with hysteresis stays quiet.
static const uint8_t kSdPins[2] = { RFU_PIN_SD_GBA, RFU_PIN_SD_GBC };
static bool g_sdArmed = false;
static bool g_sdHysteresis[2] = { false, false };  // each pin's setting before arming
static bool g_lineHysteresis[2] = { false, false }; // SC and RX, before the mode took them
static uint8_t g_sdLastResetPin = RFU_PIN_NONE;    // which pin last carried a pulse

// Transfer-path timing. After each word the slave SM blocks at `pull` and the
// GBA can clock the next one about 40us later, so the reply has to be staged
// inside that gap; anything longer is read as a stale word and librfu restarts
// the command 130ms later. Microseconds, from the free-running 1MHz timer.
#define RFU_ISR_BUDGET_US 30
static volatile uint32_t g_isrMaxUs = 0;
static volatile uint32_t g_isrLateCount = 0;
static volatile uint32_t g_isrGapMinUs = 0xFFFFFFFF;
static volatile uint32_t g_isrCount = 0;
static uint32_t g_isrLastStart = 0;

static RfuTransferDone g_doneCallback = NULL;
static void* g_doneUserData = NULL;

void rfuLink_setDoneCallback(RfuTransferDone cb, void* userData)
{
    g_doneUserData = userData;
    g_doneCallback = cb;
}

static void rfuIsr_done(const void* arg)
{
    (void)arg;
    const uint32_t startUs = timer_hw->timerawl;

    // Only the slave program raises PIO irq 0 (the adapter-master exchange is
    // polled synchronously via rfuLink_masterExchange).
    //
    // Clear first, then drain until empty: the callback stages the response and
    // the GBA can clock the next transfer within ~40us, before a long pass (the
    // SEND_DATAW one serializes a 104-byte relay frame) ends. A trailing clear
    // would wipe that transfer's flag without draining its word; the interrupt
    // would not re-fire and the slave SM would starve at `pull`. Clearing first
    // costs at worst one spurious empty re-fire.
    pio_interrupt_clear(g_pio, 0);
    while (!pio_sm_is_rx_fifo_empty(g_pio, g_smSlave))
    {
        const uint32_t rx = pio_sm_get(g_pio, g_smSlave);
        if (g_doneCallback && g_role == RFU_ROLE_GBA_MASTER)
            g_doneCallback(rx, g_doneUserData);
    }

    const uint32_t tookUs = timer_hw->timerawl - startUs;
    if (tookUs > g_isrMaxUs) g_isrMaxUs = tookUs;
    if (tookUs > RFU_ISR_BUDGET_US) g_isrLateCount++;
    if (g_isrCount != 0)
    {
        const uint32_t gapUs = startUs - g_isrLastStart;
        if (gapUs < g_isrGapMinUs) g_isrGapMinUs = gapUs;
    }
    g_isrLastStart = startUs;
    g_isrCount++;
}

static void configureSlaveSm(void)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, RFU_PIN_RX);
    sm_config_set_out_pins(&c, RFU_PIN_TX, 1);
    sm_config_set_set_pins(&c, RFU_PIN_TX, 1);
    sm_config_set_jmp_pin(&c, RFU_PIN_SC);  // 'jmp pin' (addr 4) tests SC=GP0 for the idle-gap SI mirror
    // MSB first: the GBA Normal-32 wire is MSB-first, so shift_right=false
    // shifts/reconstructs bit 31 first to match it (logical word values such as
    // 0x9966.. are unchanged). No autopush/autopull (explicit push/pull@32).
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_clkdiv(&c, 1.0f);
    sm_config_set_wrap(&c, g_offSlave + RPI_PICO_PIO_GET_WRAP_TARGET(rfu_slave32),
                       g_offSlave + RPI_PICO_PIO_GET_WRAP(rfu_slave32));
    pio_sm_init(g_pio, g_smSlave, g_offSlave, &c);
}

static void configureMasterSm(void)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, RFU_PIN_RX);
    sm_config_set_out_pins(&c, RFU_PIN_TX, 1);
    sm_config_set_set_pins(&c, RFU_PIN_TX, 1);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, RFU_PIN_SC);
    // MSB first (see configureSlaveSm).
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_in_shift(&c, false, false, 32);
    // ~154 kHz reverse clock (15 cycles/bit at clkdiv 54.25). The GBA-as-slave is
    // edge-driven and rate-tolerant; each word only has to land inside librfu's
    // 100.2 ms inter-word slave timer.
    sm_config_set_clkdiv(&c, 54.25f);
    sm_config_set_wrap(&c, g_offMaster + RPI_PICO_PIO_GET_WRAP_TARGET(rfu_master32),
                       g_offMaster + RPI_PICO_PIO_GET_WRAP(rfu_master32));
    pio_sm_init(g_pio, g_smMaster, g_offMaster, &c);
    // pio_sm_init's trailing forced jmp ran with this SM's side-set mapping live
    // and left GP0's output latch low (jmp encodes side 0). Re-seed it high while
    // GP0 is not yet output-enabled, so the first pindir flip to adapter-master
    // cannot emit a falling edge on an armed GBA.
    pio_sm_exec(g_pio, g_smMaster, 0xF000);  // set pins, 0 side 1
}

// Give the SD pins back as the link port's pinctrl configured them.
static void releaseSdPins(void)
{
    if (!g_sdArmed) return;
    for (int i = 0; i < 2; i++)
    {
        gpio_set_input_hysteresis_enabled(kSdPins[i], g_sdHysteresis[i]);
        gpio_pull_up(kSdPins[i]);
    }
    g_sdArmed = false;
}

// SD carries the game's soft-reset pulse: AgbRFU_SoftReset drives it high for
// about a millisecond before every librfu re-init. Plain SIO inputs, pulled
// down, read through the io_bank0 EDGE_HIGH latch so the pulse survives between
// polls of rfuLink_sdResetSeen(). The Schmitt trigger is required: the link
// port's pinctrl group disables hysteresis, and the weakly pulled-down line then
// chatters on crosstalk, which reads as a stream of soft resets. Callers hold
// irq_lock so g_enabled and the armed pins change together.
static void armSdPins(void)
{
    if (g_sdArmed) return;
    for (int i = 0; i < 2; i++)
    {
        const uint8_t pin = kSdPins[i];
        g_sdHysteresis[i] = gpio_is_input_hysteresis_enabled(pin);
        gpio_init(pin);
        gpio_pull_down(pin);
        gpio_set_input_hysteresis_enabled(pin, true);
        iobank0_hw->intr[pin / 8] = RFU_EDGE_HIGH_BIT(pin);  // clear stale latch
    }
    g_sdArmed = true;
}

void rfuLink_enable(void)
{
    if (g_pio == NULL || g_enabled) return;

    pio_sm_set_enabled(g_pio, g_smSlave, false);
    pio_sm_set_enabled(g_pio, g_smMaster, false);

    // pio1 is shared with the WS2812 status LED (hardware.cpp), so the two
    // programs are added beside it (WS2812 4 + slave 14 + master 6 = 24 of 32
    // slots); clearing instruction memory would wipe the LED program and freeze
    // the status LED. rfuLink_disable() removes these two again;
    // pio_add_program relocates the JMP targets.
    g_offSlave = pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(rfu_slave32));
    g_offMaster = pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(rfu_master32));

    pio_gpio_init(g_pio, RFU_PIN_SC);
    pio_gpio_init(g_pio, RFU_PIN_RX);
    pio_gpio_init(g_pio, RFU_PIN_TX);

    // The GBA SIO lines idle high (the master's SC clock rests high; the GBA's SO
    // is internally pulled up). Bias SC and RX high so the slave program's
    // `wait 0 gpio 0` blocks on a real falling edge instead of firing on a
    // floating-low line and slipping every word by one bit. Pad pulls are
    // independent of pindir, so this survives the slave<->master role swaps.
    gpio_pull_up(RFU_PIN_SC);
    gpio_pull_up(RFU_PIN_RX);

    // The clock is read through the pad's Schmitt trigger, the data is not. The
    // other modes never notice either: their state machines step every 540ns. This
    // one follows a 2 MHz clock edge by edge at 8ns a step, so noise on an edge
    // still inside the threshold region is seen, and counted. SO is sampled once
    // a bit and has 238ns to get there, which a cable's capacitance can use up:
    // hysteresis moves the falling threshold further from where the edge starts,
    // and with it on, every 1->0 on a slow cable arrived one bit late (a header
    // 0x99660010 read as 0x9df70018), so it is turned off even if pinctrl asks
    // for it.
    g_lineHysteresis[0] = gpio_is_input_hysteresis_enabled(RFU_PIN_SC);
    g_lineHysteresis[1] = gpio_is_input_hysteresis_enabled(RFU_PIN_RX);
    gpio_set_input_hysteresis_enabled(RFU_PIN_SC, true);
    gpio_set_input_hysteresis_enabled(RFU_PIN_RX, false);

    configureSlaveSm();
    configureMasterSm();

    // GBA-master role: SC and RX are inputs, TX output (low = ready)
    pio_sm_set_consecutive_pindirs(g_pio, g_smSlave, RFU_PIN_SC, 2, false);
    pio_sm_set_consecutive_pindirs(g_pio, g_smSlave, RFU_PIN_TX, 1, true);

    g_role = RFU_ROLE_GBA_MASTER;
    const unsigned int key = irq_lock();
    armSdPins();
    g_enabled = true;
    irq_unlock(key);

    pio_sm_clear_fifos(g_pio, g_smSlave);
    pio_sm_put(g_pio, g_smSlave, 0);  // RESET-state response
    pio_sm_set_enabled(g_pio, g_smSlave, true);
}

void rfuLink_disable(void)
{
    if (g_pio == NULL || !g_enabled) return;
    const unsigned int key = irq_lock();
    g_enabled = false;
    releaseSdPins();
    irq_unlock(key);
    pio_sm_set_enabled(g_pio, g_smSlave, false);
    pio_sm_set_enabled(g_pio, g_smMaster, false);
    // Free only our two slots; leave the WS2812 LED program (also on pio1)
    // resident so the status LED keeps working.
    pio_remove_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(rfu_slave32), g_offSlave);
    pio_remove_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(rfu_master32), g_offMaster);
    gpio_set_input_hysteresis_enabled(RFU_PIN_SC, g_lineHysteresis[0]);
    gpio_set_input_hysteresis_enabled(RFU_PIN_RX, g_lineHysteresis[1]);
}

void rfuLink_waitResetEnd(void)
{
    if (!g_enabled || g_sdLastResetPin == RFU_PIN_NONE) return;
    const uint32_t start = timer_hw->timerawl;
    while (gpio_get(g_sdLastResetPin) && (timer_hw->timerawl - start) < RFU_SD_PULSE_MAX_US) { }
    k_busy_wait(RFU_SD_SETTLE_US);
}

void rfuLink_realign(void)
{
    if (!g_enabled || g_role != RFU_ROLE_GBA_MASTER) return;
    pio_sm_set_enabled(g_pio, g_smSlave, false);
    pio_sm_clear_fifos(g_pio, g_smSlave);
    pio_sm_restart(g_pio, g_smSlave);
    // Past the `set pins, 0` at the wrap target: SC may still be floating, and an
    // edge on SI is one more thing for it to pick up.
    pio_sm_exec(g_pio, g_smSlave, 0x0000 | (g_offSlave + 1));  // jmp pull
    pio_interrupt_clear(g_pio, 0);
    pio_sm_set_enabled(g_pio, g_smSlave, true);
}

void rfuLink_setRole(enum RfuRole role)
{
    if (!g_enabled || role == g_role) return;

    pio_sm_set_enabled(g_pio, g_smSlave, false);
    pio_sm_set_enabled(g_pio, g_smMaster, false);
    pio_sm_clear_fifos(g_pio, g_smSlave);
    pio_sm_clear_fifos(g_pio, g_smMaster);

    if (role == RFU_ROLE_ADAPTER_MASTER)
    {
        // Take over SC without a low glitch: the GBA armed itself as an
        // external-clock slave right after the WAIT-class ACK and counts every
        // SC edge as a data bit. Required order:
        //   1. seed GP0's output latch high (and SI low, the ready lead-in)
        //      while GP0 is still an input, so the exec has no wire effect;
        //   2. flip GP0 to output, which drives the already-high latch;
        //   3. any forced jmp must carry side 1 (bit 12): a bare jmp encodes
        //      side 0 and pulls SC low, a falling edge the GBA would count.
        pio_sm_restart(g_pio, g_smMaster);
        pio_sm_exec(g_pio, g_smMaster, 0xF000);                 // set pins, 0 side 1 (SC latch high, SI low)
        pio_sm_set_consecutive_pindirs(g_pio, g_smMaster, RFU_PIN_SC, 1, true);
        pio_sm_set_consecutive_pindirs(g_pio, g_smMaster, RFU_PIN_TX, 1, true);
        pio_sm_exec(g_pio, g_smMaster, 0x1000 | g_offMaster);   // jmp wrap_target side 1 (addr 0 = pull)
        pio_sm_set_enabled(g_pio, g_smMaster, true);
    }
    else
    {
        // Release SC back to the GBA (it re-drives SC as bus master within tens
        // of microseconds of the final event handshake). The latch is high from
        // the master program's side-1 park, so the OE drop hands over at the
        // same level, without an edge.
        pio_sm_set_consecutive_pindirs(g_pio, g_smSlave, RFU_PIN_SC, 1, false);
        pio_sm_set_consecutive_pindirs(g_pio, g_smSlave, RFU_PIN_TX, 1, true);
        pio_sm_restart(g_pio, g_smSlave);
        pio_sm_exec(g_pio, g_smSlave, 0x0000 | g_offSlave);    // jmp wrap_target (no side-set on this SM)
        pio_sm_set_enabled(g_pio, g_smSlave, true);
    }

    g_role = role;
}

enum RfuRole rfuLink_getRole(void)
{
    return g_role;
}

bool rfuLink_masterExchange(uint32_t word, uint32_t* rx, uint32_t timeoutUs)
{
    if (!g_enabled || g_role != RFU_ROLE_ADAPTER_MASTER) return false;

    pio_sm_put(g_pio, g_smMaster, word);
    for (uint32_t t = 0;; t += 4)
    {
        if (!pio_sm_is_rx_fifo_empty(g_pio, g_smMaster))
        {
            *rx = pio_sm_get(g_pio, g_smMaster);
            return true;
        }
        if (t >= timeoutUs) return false;
        k_busy_wait(4);
    }
}

void rfuLink_masterDriveSi(bool high)
{
    if (!g_enabled || g_role != RFU_ROLE_ADAPTER_MASTER) return;
    // The master SM's side-set is non-optional, so forced instructions drive SC
    // too; these encodings carry side 1 (SC stays high). The SM is parked on
    // `pull block side 1` between words, so the forced set does not disturb it.
    pio_sm_exec(g_pio, g_smMaster, high ? 0xF001u : 0xF000u);  // set pins, <v> side 1
}

void rfuLink_pushTx(uint32_t word)
{
    if (!g_enabled) return;
    const size_t sm = (g_role == RFU_ROLE_GBA_MASTER) ? g_smSlave : g_smMaster;
    pio_sm_put(g_pio, sm, word);
}

bool rfuLink_gbaLineHigh(void)
{
    return gpio_get(RFU_PIN_RX);
}

void rfuLink_isrStats(uint32_t out[5])
{
    out[0] = g_isrMaxUs;
    out[1] = g_isrLateCount;
    out[2] = (g_isrGapMinUs == 0xFFFFFFFF) ? 0 : g_isrGapMinUs;
    out[3] = g_isrCount;
    out[4] = g_sdLastResetPin;
}

void rfuLink_debugSnapshot(uint8_t* smPc, uint8_t* txLvl, uint8_t* rxLvl,
                           uint8_t* lines)
{
    // Instantaneous view of the active state machine + the raw wires. smPc is
    // relative to the active program's load offset.
    const bool master = (g_role == RFU_ROLE_ADAPTER_MASTER);
    const size_t sm = master ? g_smMaster : g_smSlave;
    const uint32_t off = master ? g_offMaster : g_offSlave;
    *smPc = (uint8_t)(pio_sm_get_pc(g_pio, sm) - off);
    *txLvl = (uint8_t)pio_sm_get_tx_fifo_level(g_pio, sm);
    *rxLvl = (uint8_t)pio_sm_get_rx_fifo_level(g_pio, sm);
    *lines = (uint8_t)((gpio_get(RFU_PIN_RX) ? 1 : 0) |      // GBA SO
                       (gpio_get(RFU_PIN_TX) ? 2 : 0) |      // our SI
                       (gpio_get(RFU_PIN_SC) ? 4 : 0) |      // SC
                       (master ? 8 : 0));
}

bool rfuLink_sdResetSeen(void)
{
    // Latched rising edge on SD = the game may have run AgbRFU_SoftReset since the
    // last call. Reading clears the latch (write-1-to-clear).
    //
    // The SoftReset pulse holds SD high for ~1ms and this poll runs every ~1ms, so
    // it lands inside the pulse. The edge alone proves nothing: the 2MHz SC/SI/SO
    // bursts couple spikes onto the weakly pulled-down line, and through some cables
    // a spike is still above the threshold microseconds later, so neither does the
    // level at one instant. What only the pulse does is stay high: the line has to
    // hold for RFU_SD_HOLD_US with no falling edge latched. Unqualified, spikes read
    // as resets and wipe live link state.
    if (!g_sdArmed) return false;
    for (int i = 0; i < 2; i++)
    {
        const uint8_t pin = kSdPins[i];
        if (!(iobank0_hw->intr[pin / 8] & RFU_EDGE_HIGH_BIT(pin))) continue;
        iobank0_hw->intr[pin / 8] = RFU_EDGE_HIGH_BIT(pin) | RFU_EDGE_LOW_BIT(pin);
        if (!gpio_get(pin)) continue;
        k_busy_wait(RFU_SD_HOLD_US);
        if (!gpio_get(pin) || (iobank0_hw->intr[pin / 8] & RFU_EDGE_LOW_BIT(pin))) continue;
        g_sdLastResetPin = pin;
        return true;
    }
    return false;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static int rfuLink_init(void)
{
    // The pico-sdk defines pio1 as a macro; shadow it so the devicetree
    // node label resolves (as linkLayer_pio.c does for pio0).
    #pragma push_macro("pio1")
    #undef pio1
    const struct device* dev = DEVICE_DT_GET(DT_NODELABEL(pio1));
    #pragma pop_macro("pio1")
    g_pio = pio_rpi_pico_get_pio(dev);

    pio_rpi_pico_allocate_sm(dev, &g_smSlave);
    pio_rpi_pico_allocate_sm(dev, &g_smMaster);

    IRQ_CONNECT(PIO1_IRQ_0, 0, rfuIsr_done, NULL, 0);
    irq_enable(PIO1_IRQ_0);
    pio_set_irq0_source_enabled(g_pio, pis_interrupt0, true);

    return 0;
}

SYS_INIT(rfuLink_init, APPLICATION, 2);
