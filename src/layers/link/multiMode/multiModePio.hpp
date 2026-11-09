#include <zephyr/kernel.h>
#include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>

#pragma once

/* From pio_firmware/combined/pio_master_multi_mode.pio */
RPI_PICO_PIO_DEFINE_PROGRAM(pio_master, 19, 31,
    0xdf01, //  0: irq    nowait 1               [31]
    0x82a0, //  1: pull   block                  [2]
    0xe01d, //  2: set    pins, 29
    0xe085, //  3: set    pindirs, 5
    0x6030, //  4: out    x, 16
    0x0045, //  5: jmp    x--, 5
    0xc000, //  6: irq    nowait 0
    0xa027, //  7: mov    x, osr
    0x0048, //  8: jmp    x--, 8
    0x80a0, //  9: pull   block
    0xa047, // 10: mov    y, osr
    0xe09d, // 11: set    pindirs, 29
    0xe41c, // 12: set    pins, 28               [4]
    0xa025, // 13: mov    x, status
    0x0040, // 14: jmp    x--, 0
    0x8080, // 15: pull   noblock
    0xef04, // 16: set    pins, 4                [15]
    0x6e02, // 17: out    pins, 2                [14]
    0x00f1, // 18: jmp    !osre, 17
            //     .wrap_target
    0xf01c, // 19: set    pins, 28               [16]
    0x008d, // 20: jmp    y--, 13
    0xe085, // 21: set    pindirs, 5
    0xe018, // 22: set    pins, 24
    0xa0cb, // 23: mov    isr, !null
    0x4002, // 24: in     pins, 2
    0xa02e, // 25: mov    x, !isr
    0x0038, // 26: jmp    !x, 24
    0xf32f, // 27: set    x, 15                  [19]
    0x4e02, // 28: in     pins, 2                [14]
    0x005c, // 29: jmp    x--, 28
    0x9020, // 30: push   block                  [16]
    0xee9d, // 31: set    pindirs, 29            [14]
            //     .wrap
);

/* From pio_firmware/combined/pio_slave_multi_mode.pio. Drives SD only on the pin the GBA's start bit came in on. */
RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave, 0, 31,
            //     .wrap_target
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
    0xd400, // 10: irq    nowait 0               [20]
    0x4e02, // 11: in     pins, 2                [14]
    0x004b, // 12: jmp    x--, 11
    0x8020, // 13: push   block
    0xa0cb, // 14: mov    isr, !null
    0x4002, // 15: in     pins, 2
    0xa02e, // 16: mov    x, !isr
    0x0033, // 17: jmp    !x, 19
    0xa043, // 18: mov    y, null
    0x80a0, // 19: pull   block
    0xa0e2, // 20: mov    osr, y
    0x7b82, // 21: out    pindirs, 2             [27]
    0xa025, // 22: mov    x, status
    0x005e, // 23: jmp    x--, 30
    0x8080, // 24: pull   noblock
    0xef00, // 25: set    pins, 0                [15]
    0x6e02, // 26: out    pins, 2                [14]
    0x00fa, // 27: jmp    !osre, 26
    0xf118, // 28: set    pins, 24               [17]
    0x1f16, // 29: jmp    22                     [31]
    0xdf01, // 30: irq    nowait 1               [31]
    0x80a0, // 31: pull   block
            //     .wrap
);