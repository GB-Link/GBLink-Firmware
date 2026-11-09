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

#ifdef __cplusplus
}
#endif
