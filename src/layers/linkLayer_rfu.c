#include "linkLayer_rfu.h"

#include "hardware/pio.h"
#include "hardware/gpio.h"

#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

// Pin roles (GBC cable): GP0 = SC, GP1 = GBA SO (in), GP2 = GBA SI (out),
// GP4 = SD (in — carries the game's AgbRFU_SoftReset pulse).
#define RFU_PIN_SC  0
#define RFU_PIN_RX  1
#define RFU_PIN_TX  2
#define RFU_PIN_SD  4

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//
// PIO programs (hand-assembled, see comments for the .pio equivalents).
// Shift config for both: left shift (MSB first — the GBA SIO wire order, proven
// on HW test #4), no autopush/autopull,
// out_pins/set_pins = GP2 (count 1), in_pins = GP1.
//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

/* rfu_slave32 — GBA is bus master; edge-driven on SC so one program serves
 * both 256 kHz and 2 MHz (clkdiv 1.0). The idle-gap mirror (addr 3-4) holds
 * SI(GP2) = NOT SO(GP1) while SC(GP0) rests high, so librfu's command-phase
 * handshake_wait(1)/handshake_wait(0) busy-waits release (the GBA polls SI
 * between every 32-bit word). It yields GP2 to the data shifter the instant SC
 * falls. The mirror uses `mov` (OSR-safe), so the staged response word survives
 * into the bit loop. Detection (Sio32IDIntr) never reads SI and ignores GP2 in
 * the gap, so its 32 data bits are byte-identical to before.
 *   .wrap_target
 *   0: set  pins, 0        ; idle TX line low = "adapter ready" (one-shot)
 *   1: pull block          ; staged response (CPU keeps one word queued)
 *   2: set  x, 31
 *   3: mov  pins, ~pins    ; SI(GP2) = NOT SO(GP1) — continuous level mirror
 *   4: jmp  pin, 3         ; while SC(GP0) high keep mirroring; SC low -> fall
 *   5: wait 0 gpio 0       ; SC falling edge: both sides shift out   (bit:)
 *   6: out  pins, 1
 *   7: wait 1 gpio 0       ; SC rising edge: sample
 *   8: in   pins, 1
 *   9: jmp  x--, 5
 *  10: push block
 *  11: irq  0
 *   .wrap
 * jmp_pin = SC (GP0), set in configureSlaveSm. SC idles high, the GBA-as-master
 * sets up on the falling edge and samples on the rising edge (HW-proven).
 */
RPI_PICO_PIO_DEFINE_PROGRAM(rfu_slave32, 0, 11,
    0xE000,  //  0: set    pins, 0
    0x80A0,  //  1: pull   block
    0xE03F,  //  2: set    x, 31
    0xA008,  //  3: mov    pins, ~pins
    0x00C3,  //  4: jmp    pin, 3
    0x2000,  //  5: wait   0 gpio 0
    0x6001,  //  6: out    pins, 1
    0x2080,  //  7: wait   1 gpio 0
    0x4001,  //  8: in     pins, 1
    0x0045,  //  9: jmp    x--, 5
    0x8020,  // 10: push   block
    0xC000); // 11: irq    0

/* rfu_master32 — adapter clocks (wait-response delivery); side-set 1 = SC
 * (non-optional: EVERY instruction, including pio_sm_exec'd ones, drives SC
 * from bit 12 — only side-1 encodings may ever be forced on this SM).
 * in_pins = GP1 (GBA SO), out_pins/set_pins = GP2 (SI). SC idles HIGH.
 * clkdiv 54.25 -> 1 PIO cycle ≈ 434 ns; 15 cycles/bit ≈ 154 kHz.
 *
 * A pure 32-bit exchanger with NO pin waits — it cannot hang, and all librfu
 * ready-handshaking happens in C between words (rfuProtocolSection), where the
 * word-1 / word-N asymmetry of the GBA's slave ISR is a trivial branch.
 *
 * Bit geometry = the MIRROR of the HW-proven command-phase slave loop (HW test
 * #31 wire evidence: reading the GBA's 0x80000000 pre-load one bit late =
 * 0x40000000, plus the librfu word-1 reject signature): the GBA-as-slave
 * SHIFTS ITS SO OUT ON THE FALLING EDGE (nothing is presented at idle — SO
 * rests at the SD level, HW test #25) and LATCHES SI ON THE RISING EDGE,
 * exactly like the GBA-as-master does in reverse. Hence per bit: the falling
 * edge and our SI drive share a cycle (stable long before the rise), sample
 * its SO mid-low (it shifted at the fall), and the rising edge latches our SI
 * into the GBA.
 *   .side_set 1            ; SC: side 1 = HIGH (idle), side 0 = LOW
 *   .wrap_target
 *   0: pull block    side1        ; parked here between words (SC held high)
 *   1: set x,31      side1
 *   2: out pins,1    side0 [4]    ; FALLING edge + drive SI bit; GBA shifts SO
 *   3: in  pins,1    side0 [1]    ; mid-low: sample the SO bit it just shifted
 *   4: jmp x--,2     side1 [7]    ; RISING edge: GBA latches SI; hold high
 *   5: push block    side1        ; word read back from the GBA
 *   .wrap
 * 15 cycles/bit at clkdiv 54.25 ≈ 154 kHz. Slot budget pio1: WS2812 4 +
 * slave 12 + master 6 = 22/32.
 */
RPI_PICO_PIO_DEFINE_PROGRAM(rfu_master32, 0, 5,
    0x90A0,  //  0: pull   block        side 1
    0xF03F,  //  1: set    x, 31        side 1
    0x6401,  //  2: out    pins, 1      side 0 [4]
    0x4101,  //  3: in     pins, 1      side 0 [1]
    0x1742,  //  4: jmp    x--, 2       side 1 [7]
    0x9020); //  5: push   block        side 1

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

// io_bank0 raw interrupt latch bit for an EDGE_HIGH event on the SD pin
// (4 status bits per GPIO in each INTR word; EDGE_HIGH is bit 3 of the group).
#define RFU_SD_EDGE_HIGH_BITS  (1u << (((RFU_PIN_SD % 8) * 4) + 3))

static PIO g_pio = NULL;
static size_t g_smSlave = 0;
static size_t g_smMaster = 1;
static uint32_t g_offSlave = 0;
static uint32_t g_offMaster = 0;
static enum RfuRole g_role = RFU_ROLE_GBA_MASTER;
static bool g_enabled = false;

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
    // Only the slave program raises PIO irq 0 (the adapter-master exchange is
    // polled synchronously via rfuLink_masterExchange).
    //
    // Clear FIRST, then drain until empty. The callback stages the response
    // and the GBA can complete its ready-handshake + clock the NEXT transfer
    // within ~40us — before a long ISR pass (the SEND_DATAW one serializes a
    // 104-byte relay frame) reaches its end. A trailing clear would wipe that
    // fresh transfer's flag without draining its word: no edge, no re-fire,
    // the word strands in the RX FIFO and the slave SM starves at `pull`
    // forever (the stall probe's smPc=1/tx=0/rx=1 signature). With
    // clear-first, a push after the clear re-raises the flag (re-fire), and a
    // push before the drain loop is simply drained here — worst case one
    // spurious empty re-fire.
    pio_interrupt_clear(g_pio, 0);
    while (!pio_sm_is_rx_fifo_empty(g_pio, g_smSlave))
    {
        const uint32_t rx = pio_sm_get(g_pio, g_smSlave);
        if (g_doneCallback && g_role == RFU_ROLE_GBA_MASTER)
            g_doneCallback(rx, g_doneUserData);
    }
}

static void configureSlaveSm(void)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, RFU_PIN_RX);
    sm_config_set_out_pins(&c, RFU_PIN_TX, 1);
    sm_config_set_set_pins(&c, RFU_PIN_TX, 1);
    sm_config_set_jmp_pin(&c, RFU_PIN_SC);  // 'jmp pin' (addr 4) tests SC=GP0 for the idle-gap SI mirror
    // MSB first: HW test #4 proved the GBA Normal-32 wire is MSB-first — with
    // shift_right=true the adapter read 0x72920000, the exact 32-bit bit-reversal
    // of the GBA's first checkID word 0x0000494E. shift_right=false shifts/
    // reconstructs bit 31 first, matching the wire (the logical word values, e.g.
    // ID_DANCE/0x9966.., are unchanged). No autopush/autopull (explicit push/pull@32).
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
    // pio_sm_init's trailing forced jmp ran with this SM's side-set mapping live,
    // leaving GP0's output latch LOW (jmp encodes side 0). Re-seed it HIGH now,
    // while GP0 is not yet output-enabled, so the first pindir flip to
    // adapter-master cannot emit a falling edge on an armed GBA.
    pio_sm_exec(g_pio, g_smMaster, 0xF000);  // set pins, 0 side 1
}

void rfuLink_enable(void)
{
    if (g_pio == NULL || g_enabled) return;

    pio_sm_set_enabled(g_pio, g_smSlave, false);
    pio_sm_set_enabled(g_pio, g_smMaster, false);

    // pio1 is SHARED with the WS2812 status LED (hardware.cpp). Add our two
    // programs ALONGSIDE the LED program (WS2812 4 + slave 12 + master 6 = 22 of
    // 32 instruction slots) — do NOT clear the whole instruction memory here, or
    // the LED program is wiped and the status LED freezes mid-session. disable()
    // removes exactly these two again. pio_add_program relocates the JMP targets.
    g_offSlave = pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(rfu_slave32));
    g_offMaster = pio_add_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(rfu_master32));

    pio_gpio_init(g_pio, RFU_PIN_SC);
    pio_gpio_init(g_pio, RFU_PIN_RX);
    pio_gpio_init(g_pio, RFU_PIN_TX);

    // The GBA SIO lines idle high (the master's SC clock rests high; the GBA's
    // SO is internally pulled up). Bias SC and the GBA-driven input RX high so
    // the slave program's `wait 0 gpio 0` blocks on a real falling edge instead
    // of firing on a floating-low line — without this the first bit clocks with
    // no edge and every word is slipped by one bit. Pad pulls are independent
    // of pindir, so this one call survives the slave<->master role swaps.
    gpio_pull_up(RFU_PIN_SC);
    gpio_pull_up(RFU_PIN_RX);

    // SD carries the game's soft-reset pulse (AgbRFU_SoftReset drives it high
    // for well under a millisecond before every librfu re-init — the signal a
    // real adapter and gpsp reset on). Plain SIO input, pulled down, with the
    // io_bank0 EDGE_HIGH latch used so the pulse is caught even between polls
    // of rfuLink_sdResetSeen().
    gpio_init(RFU_PIN_SD);
    gpio_pull_down(RFU_PIN_SD);
    iobank0_hw->intr[RFU_PIN_SD / 8] = RFU_SD_EDGE_HIGH_BITS;  // clear stale latch

    configureSlaveSm();
    configureMasterSm();

    // GBA-master role: SC and RX are inputs, TX output (low = ready)
    pio_sm_set_consecutive_pindirs(g_pio, g_smSlave, RFU_PIN_SC, 2, false);
    pio_sm_set_consecutive_pindirs(g_pio, g_smSlave, RFU_PIN_TX, 1, true);

    g_role = RFU_ROLE_GBA_MASTER;
    g_enabled = true;

    pio_sm_clear_fifos(g_pio, g_smSlave);
    pio_sm_put(g_pio, g_smSlave, 0);  // RESET-state response
    pio_sm_set_enabled(g_pio, g_smSlave, true);
}

void rfuLink_disable(void)
{
    if (g_pio == NULL || !g_enabled) return;
    g_enabled = false;
    pio_sm_set_enabled(g_pio, g_smSlave, false);
    pio_sm_set_enabled(g_pio, g_smMaster, false);
    // Free only our two slots; leave the WS2812 LED program (also on pio1)
    // resident so the status LED keeps working.
    pio_remove_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(rfu_slave32), g_offSlave);
    pio_remove_program(g_pio, RPI_PICO_PIO_GET_PROGRAM(rfu_master32), g_offMaster);
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
        // Take over SC without ever letting it dip: the GBA armed itself as
        // an external-clock slave right after the WAIT-class ACK and counts
        // EVERY SC edge as a data bit, so the handoff must be glitch-free.
        // Order matters:
        //   1. seed GP0's output latch HIGH (and SI low, the ready lead-in)
        //      while GP0 is still an input — the exec has no wire effect yet;
        //   2. only then flip GP0 to output (drives the already-high latch);
        //   3. any forced jmp must carry side 1 (bit 12) — a bare jmp encodes
        //      side 0 and yanks SC low for a phantom falling edge.
        pio_sm_restart(g_pio, g_smMaster);
        pio_sm_exec(g_pio, g_smMaster, 0xF000);                 // set pins, 0 side 1 (SC latch high, SI low)
        pio_sm_set_consecutive_pindirs(g_pio, g_smMaster, RFU_PIN_SC, 1, true);
        pio_sm_set_consecutive_pindirs(g_pio, g_smMaster, RFU_PIN_TX, 1, true);
        pio_sm_exec(g_pio, g_smMaster, 0x1000 | g_offMaster);   // jmp wrap_target side 1 (addr 0 = pull)
        pio_sm_set_enabled(g_pio, g_smMaster, true);
    }
    else
    {
        // Release SC back to the GBA (it re-drives SC as bus master within
        // tens of microseconds of the final event handshake). The latch is
        // high from the master program's side-1 park, so the OE drop hands
        // over at the same level — no edge.
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
    // too — these encodings carry side 1 (SC stays high). The SM is parked on
    // `pull block side 1` between words; the forced set interleaves cleanly.
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

void rfuLink_debugSnapshot(uint8_t* smPc, uint8_t* txLvl, uint8_t* rxLvl,
                           uint8_t* lines)
{
    // Instantaneous view of the ACTIVE state machine + the raw wires, for
    // stall forensics. smPc is relative to the active program's load offset.
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
    // Latched rising edge on SD = the game ran AgbRFU_SoftReset since the
    // last call. Reading clears the latch (write-1-to-clear).
    //
    // Qualify the edge with the LEVEL: the real SoftReset pulse holds SD high
    // for ~1ms (and this poll runs every ~1ms, so it lands inside the pulse),
    // while crosstalk from the neighboring 2MHz SC/SI/SO bursts onto the
    // weakly pulled-down SD pin is gone within nanoseconds. An unqualified
    // edge-latch treated such spikes as resets and silently wiped live link
    // state mid-session (HW test #40).
    if (iobank0_hw->intr[RFU_PIN_SD / 8] & RFU_SD_EDGE_HIGH_BITS)
    {
        iobank0_hw->intr[RFU_PIN_SD / 8] = RFU_SD_EDGE_HIGH_BITS;
        k_busy_wait(2);
        return gpio_get(RFU_PIN_SD);
    }
    return false;
}

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//

static int rfuLink_init(void)
{
    // The pico-sdk defines pio1 as a macro; shadow it so the devicetree
    // node label resolves (same trick as linkLayer_pio.c uses for pio0).
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
