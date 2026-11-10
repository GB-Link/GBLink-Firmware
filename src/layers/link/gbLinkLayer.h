#pragma once

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the GB SPI PIO program on pio0.
 * This disables any existing PIO program on pio0 first (e.g., GBA link).
 * Must be called before gb_link_transfer().
 */
void gb_link_init(void);

/**
 * Deinitialize the GB SPI PIO, freeing pio0 for other uses (e.g., GBA link).
 */
void gb_link_deinit(void);

/**
 * Perform a full-duplex SPI transfer.
 * For each byte, clocks tx out while reading rx in simultaneously.
 * 
 * @param tx   Transmit buffer (len bytes)
 * @param rx   Receive buffer (len bytes)
 * @param len  Number of bytes to transfer
 * @param us_between  Microseconds to wait between individual byte transfers
 */
void gb_link_transfer(const uint8_t* tx, uint8_t* rx, uint32_t len, uint32_t us_between);

/**
 * Called in the PIO interrupt after every byte while the Game Boy clocks the link.
 *
 * @param received  The Game Boy's byte
 * @return          The byte for the next transfer
 */
typedef uint8_t (*gb_link_passive_handler_t)(uint8_t received);

void gb_link_set_passive_handler(gb_link_passive_handler_t handler);

/**
 * Switch between the adapter clocking the Game Boy (active, the state after
 * gb_link_init()) and the Game Boy clocking the adapter (passive). Also leaves
 * hybrid.
 *
 * @param passive  true: the Game Boy clocks
 * @param first    Passive: the byte for the first transfer
 */
void gb_link_set_passive(bool passive, uint8_t first);

/**
 * Start the passive program over at a byte boundary, answering `first` next.
 * Also switches from active or hybrid.
 */
void gb_link_restart_passive(uint8_t first);

/**
 * Called in the PIO interrupt after every byte in hybrid.
 *
 * @param received  The Game Boy's byte
 * @param clocked   true when the adapter clocked it, false when the Game Boy did
 * @return          The byte for the next transfer. After a byte the Game Boy
 *                  clocked it is the adapter's turn, and -1 means there is
 *                  nothing to clock yet (see gb_link_hybrid_clock()).
 */
typedef int (*gb_link_hybrid_handler_t)(uint8_t received, bool clocked);

void gb_link_set_hybrid_handler(gb_link_hybrid_handler_t handler);

/**
 * Hybrid: the Game Boy and the adapter take turns clocking, one byte each. The
 * adapter clocks its byte no sooner than `gap_us` after the Game Boy's byte
 * ended (for a first turn, after this call), and SCK is driven only while it
 * does. Also switches from active or passive.
 *
 * @param game_boy_first  true: the Game Boy clocks the first byte
 * @param first           The Game Boy first: the answer to its first byte
 */
void gb_link_start_hybrid(uint32_t gap_us, bool game_boy_first, uint8_t first);

/**
 * Hybrid: clock `byte` on the adapter's turn the handler had nothing for. Call
 * with the PIO interrupt locked out.
 */
void gb_link_hybrid_clock(uint8_t byte);

/**
 * Passive and hybrid: call regularly. A byte the Game Boy started but has not
 * finished within `stall_ms` leaves the bits out of step, so the passive
 * program starts over, keeping the byte it was about to send.
 */
void gb_link_check_passive_stall(uint32_t stall_ms);

#ifdef __cplusplus
}
#endif
