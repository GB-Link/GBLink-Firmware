#include <zephyr/kernel.h>

#pragma once

// 32-bit SIO NORMAL-mode link layer for the Wireless Adapter (RFU) mode,
// on PIO1 (the 16-bit MULTI-mode layer keeps PIO0 untouched).
//
// GP0 = SC, GP1 carries the GBA's SO (our receive) and GP2 is crossed to the
// GBA's SI (our transmit); SD arrives on GP3 or GP4 and both are watched, so no
// cable selection applies. A GBA link cable grounds GP1 and cannot carry the
// receive line, so this mode needs an adapter plugged straight into the GBA or
// a GBC cable.

enum RfuRole
{
    RFU_ROLE_GBA_MASTER,     // GBA clocks (command phase); slave program
    RFU_ROLE_ADAPTER_MASTER  // adapter clocks (wait-response delivery)
};

// Called from PIO1 IRQ0 context with the 32-bit word received this transfer.
// Only fires in GBA-master role (the adapter-master exchange is synchronous).
typedef void (*RfuTransferDone)(uint32_t rx, void* userData);

void rfuLink_setDoneCallback(RfuTransferDone cb, void* userData);

// Claim GP0-GP2 for PIO1, load both programs, start in GBA-master role with
// the TX FIFO primed (value 0, the RESET-state response).
void rfuLink_enable(void);

// Stop both state machines. Pins are reclaimed by the next mode's
// link_changeMode()/pio_gpio_init.
void rfuLink_disable(void);

// Swap roles: disables the other state machine, clears FIFOs and flips the SC
// pin direction with SC held high throughout (an armed GBA-as-slave counts
// any SC edge as a data bit, so the handoff must be glitch-free).
// The caller pushes the first TX word after a swap to GBA-master.
void rfuLink_setRole(enum RfuRole role);

enum RfuRole rfuLink_getRole(void);

// Adapter-master role only: clock one 32-bit word into the GBA-as-slave and
// return what it shifted out. Synchronous (~222us at the reverse clock rate);
// the timeout only guards a wedged state machine, since the PIO program never
// waits on a pin. The inter-word ready handshake is left to the caller
// (rfuLink_masterDriveSi + polling rfuLink_gbaLineHigh).
bool rfuLink_masterExchange(uint32_t word, uint32_t* rx, uint32_t timeoutUs);

// Adapter-master role only: drive the SI line (GP2) level for the inter-word
// handshake while the state machine is parked between words.
void rfuLink_masterDriveSi(bool high);

// Stage the next 32-bit transmit word on the active state machine. The
// protocol layer keeps exactly one word staged at all times in GBA-master
// role. (Adapter-master delivery uses rfuLink_masterExchange instead.)
void rfuLink_pushTx(uint32_t word);

// Raw level of the GBA's SO line (GP1).
bool rfuLink_gbaLineHigh(void);

// Transfer-path timing: worst interrupt pass (us), passes longer than
// RFU_ISR_BUDGET_US, tightest gap the GBA left between words (us), passes
// counted, and the GPIO the last soft-reset pulse arrived on (RFU_PIN_NONE if
// none has).
void rfuLink_isrStats(uint32_t out[5]);

// The active state machine's program counter (relative to its program), FIFO
// levels, and the raw line levels (bit0 = GBA SO, bit1 = our SI, bit2 = SC,
// bit3 = adapter-master role).
void rfuLink_debugSnapshot(uint8_t* smPc, uint8_t* txLvl, uint8_t* rxLvl,
                           uint8_t* lines);

// After rfuLink_sdResetSeen() returned true: blocks until the pulse is over and the
// lines have settled (bounded, a few milliseconds at most).
void rfuLink_waitResetEnd(void);

// Starts the slave state machine over: bit counter, shift registers and FIFOs. The
// caller stages the next response with rfuLink_pushTx(). Only in the GBA-master role.
void rfuLink_realign(void);

// True once per soft reset: a rising edge was latched on the SD line since the
// last call. The game ran AgbRFU_SoftReset, so the adapter must drop all link
// state and re-enter detection (the same signal gpsp's rfu_reset keys on).
// Edge-latched in hardware; polling cannot miss it.
bool rfuLink_sdResetSeen(void);
