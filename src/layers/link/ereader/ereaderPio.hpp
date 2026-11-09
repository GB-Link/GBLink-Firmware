#include <zephyr/kernel.h>
#include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>

#pragma once

/* From pio_firmware/combined/pio_slave_ereader.pio: the multi-mode slave, but SO stays input.
 * The CPU holds SO low on the GPIO, so it stays low across program reloads. */
RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_ereader, 0, 31,
            //     .wrap_target
    0xe018, //  0: set    pins, 24
    0xe080, //  1: set    pindirs, 0
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
