#include <zephyr/kernel.h>
#include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>

#pragma once

/* SET-instruction pin values: bit 0 SC (GP0), bit 1 SI (GP1, always input), bit 2 SO (GP2),
 * bit 3 SD on GP3, bit 4 SD on GP4. OUT and IN cover GP3..GP4.
 *
 * Depending on the cable, SD arrives on GP3 or GP4 and the other pin is open or carries the partner's
 * VCC. Both programs read the two pins together and drive SD only on the pin the GBA's start bit came
 * in on, so one program per role serves every cable and a pin wired to VCC is never driven. */

/* 2-4 players, the adapter as the parent. TX FIFO per transfer, queued from the done interrupt of the previous one:
 * a control word (bits 0-1 SD output mask, bits 2-3 the GBA's slot, bits 4-31 the gap before SC low), then one
 * word per other player in slot order, each bit doubled. The mask is the pin the GBA answered on, 0 until it has
 * answered. Execution starts at 0, whose done interrupt queues the first transfer.
 * Source: pio_firmware/pio_master_multi_mode.pio. */
RPI_PICO_PIO_DEFINE_PROGRAM(pio_master_multi, 9, 23,
    0xdf00, //  0: irq    nowait 0               [31]
    0x82a0, //  1: pull   block                  [2]
    0xe01d, //  2: set    pins, 29
    0xa0c7, //  3: mov    isr, osr
    0x6082, //  4: out    pindirs, 2
    0x6042, //  5: out    y, 2
    0x603c, //  6: out    x, 28
    0x0047, //  7: jmp    x--, 7
    0xe21c, //  8: set    pins, 28               [2]
    0x0098, //  9: jmp    y--, 24
    0xe085, // 10: set    pindirs, 5
    0xe018, // 11: set    pins, 24
    0xa0e6, // 12: mov    osr, isr
    0xc002, // 13: irq    nowait 2
    0xa0cb, // 14: mov    isr, !null
    0x4002, // 15: in     pins, 2
    0xa02e, // 16: mov    x, !isr
    0x002f, // 17: jmp    !x, 15
    0xf32f, // 18: set    x, 15                  [19]
    0x4e02, // 19: in     pins, 2                [14]
    0x0053, // 20: jmp    x--, 19
    0x9020, // 21: push   block                  [16]
    0x6082, // 22: out    pindirs, 2
    0xfe1c, // 23: set    pins, 28               [30]
    0xa025, // 24: mov    x, status
    0x0040, // 25: jmp    x--, 0
    0x8080, // 26: pull   noblock
    0xef04, // 27: set    pins, 4                [15]
    0x6e02, // 28: out    pins, 2                [14]
    0x00fc, // 29: jmp    !osre, 28
    0xef1c, // 30: set    pins, 28               [15]
    0x0009);// 31: jmp    9

/* pio_master_multi: the poll for the GBA's start bit, and the receive the reply watchdog jumps to when none comes. */
#define MULTI_POLL_FIRST 15
#define MULTI_POLL_LAST 17
#define MULTI_RECEIVE 18

/* SC stays high for the control word's gap + 6 cycles, so this keeps it at gapCycles + 314. */
#define MULTI_GAP_OFFSET 308

/* 2-4 players, the GBA as the parent in slot 0. TX FIFO per transfer: one word per other player in slot order,
 * each bit doubled, queued from the tx interrupt while the GBA's word arrives.
 * Source: pio_firmware/pio_slave_multi_mode.pio. */
RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_multi, 0, 31,
    0xe018, //  0: set    pins, 24
    0xe084, //  1: set    pindirs, 4
    0xe02f, //  2: set    x, 15
    0xa0cb, //  3: mov    isr, !null
    0x4002, //  4: in     pins, 2
    0xa04e, //  5: mov    y, !isr
    0x0083, //  6: jmp    y--, 3
    0x4002, //  7: in     pins, 2
    0xa04e, //  8: mov    y, !isr
    0x0067, //  9: jmp    !y, 7
    0xd401, // 10: irq    nowait 1               [20]
    0x4e02, // 11: in     pins, 2                [14]
    0x004b, // 12: jmp    x--, 11
    0x8020, // 13: push   block
    0xa0cb, // 14: mov    isr, !null
    0x4002, // 15: in     pins, 2
    0xa02e, // 16: mov    x, !isr
    0x0033, // 17: jmp    !x, 19
    0xa043, // 18: mov    y, null
    0xa0e2, // 19: mov    osr, y
    0x6082, // 20: out    pindirs, 2
    0xbc42, // 21: nop                           [28]
    0xe02f, // 22: set    x, 15
    0x80a0, // 23: pull   block
    0xf000, // 24: set    pins, 0                [16]
    0x6e02, // 25: out    pins, 2                [14]
    0x0059, // 26: jmp    x--, 25
    0xf018, // 27: set    pins, 24               [16]
    0xbf42, // 28: nop                           [31]
    0xa025, // 29: mov    x, status
    0x0036, // 30: jmp    !x, 22
    0xdf00);// 31: irq    nowait 0               [31]
