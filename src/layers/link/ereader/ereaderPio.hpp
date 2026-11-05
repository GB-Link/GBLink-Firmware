#include <zephyr/kernel.h>
#include <zephyr/drivers//misc/pio_rpi_pico/pio_rpi_pico.h>

#pragma once

/* SET-instruction pin values.
 *   bit 0 = SC  (GP0)       bit 1 = SI  (GP1, always input)
 *   bit 2 = SO  (GP2)       bit 3/4 = SD (GP3 or GP4)       */
#define PIO_SC          1   /* bit 0 */
#define PIO_SO          4   /* bit 2 */
#define PIO_SD_GBA      8   /* bit 3 — GBA cable, SD on GP3, set_count=4 */
#define PIO_SD_GBC      16  /* bit 4 — GBC cable, SD on GP4, set_count=5 */

/* --- GBA cable programs (SD on GP3) --- */

/* e-reader child: SO stays on GPIO LOW — no PIO_SO in set ops. */
RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_ereader_gba, 0, 23,
            //     .wrap_target
    0xe008, //  0: set    pins, 8                    
    0xe080, //  1: set    pindirs, 0                 
    0xe02f, //  2: set    x, 15                      
    0x2020, //  3: wait   0 pin, 0                   
    0xd700, //  4: irq    nowait 0               [23]
    0x4e01, //  5: in     pins, 1                [14]
    0x0045, //  6: jmp    x--, 5                     
    0x8020, //  7: push   block                      
    0x80a0, //  8: pull   block                      
    0xe008, //  9: set    pins, 8                    
    0xe088, // 10: set    pindirs, 8                 
    0xbf42, // 11: nop                           [31]
    0xe02f, // 12: set    x, 15                      
    0x80a0, // 13: pull   block                      
    0xf000, // 14: set    pins, 0                [16]
    0x6e01, // 15: out    pins, 1                [14]
    0x004f, // 16: jmp    x--, 15                    
    0xf008, // 17: set    pins, 8                [16]
    0xbf42, // 18: nop                           [31]
    0xa025, // 19: mov    x, status                  
    0x002c, // 20: jmp    !x, 12                     
    0xdf01, // 21: irq    nowait 1               [31]
    0xbf42, // 22: nop                           [31]
    0x80a0, // 23: pull   block                      
            //     .wrap
);

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_ereader_gbc, 0, 23,
    //     .wrap_target
    0xe010, //  0: set    pins, 16                   
    0xe080, //  1: set    pindirs, 0                 
    0xe02f, //  2: set    x, 15                      
    0x2020, //  3: wait   0 pin, 0                   
    0xd700, //  4: irq    nowait 0               [23]
    0x4e01, //  5: in     pins, 1                [14]
    0x0045, //  6: jmp    x--, 5                     
    0x8020, //  7: push   block                      
    0x80a0, //  8: pull   block                      
    0xe010, //  9: set    pins, 16                   
    0xe090, // 10: set    pindirs, 16                
    0xbf42, // 11: nop                           [31]
    0xe02f, // 12: set    x, 15                      
    0x80a0, // 13: pull   block                      
    0xf000, // 14: set    pins, 0                [16]
    0x6e01, // 15: out    pins, 1                [14]
    0x004f, // 16: jmp    x--, 15                    
    0xf010, // 17: set    pins, 16               [16]
    0xbf42, // 18: nop                           [31]
    0xa025, // 19: mov    x, status                  
    0x002c, // 20: jmp    !x, 12                     
    0xdf01, // 21: irq    nowait 1               [31]
    0xbf42, // 22: nop                           [31]
    0x80a0, // 23: pull   block                      
            //     .wrap
);

//-////////////////////////////////////////////////////////////////////////////////////////////////////////-//