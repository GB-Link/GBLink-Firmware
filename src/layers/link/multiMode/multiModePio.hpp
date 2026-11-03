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

RPI_PICO_PIO_DEFINE_PROGRAM(pio_master_gba, 0, 31,
    //     .wrap_target
    0xe08d, //  0: set    pindirs, 13                
    0xe00d, //  1: set    pins, 13                   
    0x0082, //  2: jmp    y--, 2                     
    0xc000, //  3: irq    nowait 0                   
    0xe03e, //  4: set    x, 30                      
    0x0945, //  5: jmp    x--, 5                 [9] 
    0xe00c, //  6: set    pins, 12                   
    0x8080, //  7: pull   noblock                    
    0xa047, //  8: mov    y, osr                     
    0x0095, //  9: jmp    y--, 21                    
    0xe085, // 10: set    pindirs, 5                 
    0xe008, // 11: set    pins, 8                    
    0xc002, // 12: irq    nowait 2                   
    0x2020, // 13: wait   0 pin, 0                   
    0xf62f, // 14: set    x, 15                  [22]
    0x4e01, // 15: in     pins, 1                [14]
    0x004f, // 16: jmp    x--, 15                    
    0x9020, // 17: push   block                  [16]
    0xe08d, // 18: set    pindirs, 13                
    0xfd0c, // 19: set    pins, 12               [29]
    0x0009, // 20: jmp    9                          
    0xa025, // 21: mov    x, status                  
    0x005d, // 22: jmp    x--, 29                    
    0x8080, // 23: pull   noblock                    
    0xef04, // 24: set    pins, 4                [15]
    0x6e01, // 25: out    pins, 1                [14]
    0x00f9, // 26: jmp    !osre, 25                  
    0xef0c, // 27: set    pins, 12               [15]
    0x0009, // 28: jmp    9                          
    0xdf01, // 29: irq    nowait 1               [31]
    0x80a0, // 30: pull   block                      
    0xa047, // 31: mov    y, osr                     
            //     .wrap
);

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_gba, 0, 23,
    //     .wrap_target
    0xe008, //  0: set    pins, 8                    
    0xe084, //  1: set    pindirs, 4                 
    0xe02f, //  2: set    x, 15                      
    0x2020, //  3: wait   0 pin, 0                   
    0xd700, //  4: irq    nowait 0               [23]
    0x4e01, //  5: in     pins, 1                [14]
    0x0045, //  6: jmp    x--, 5                     
    0x8020, //  7: push   block                      
    0x80a0, //  8: pull   block                      
    0xe008, //  9: set    pins, 8                    
    0xe08c, // 10: set    pindirs, 12                
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
            //     .wrap            [31]
);

/* --- GBC cable programs (SD on GP4) --- */

RPI_PICO_PIO_DEFINE_PROGRAM(pio_master_gbc, 0, 31,
    //     .wrap_target
    0xe095, //  0: set    pindirs, 21                
    0xe015, //  1: set    pins, 21                   
    0x0082, //  2: jmp    y--, 2                     
    0xc000, //  3: irq    nowait 0                   
    0xe03e, //  4: set    x, 30                      
    0x0945, //  5: jmp    x--, 5                 [9] 
    0xe014, //  6: set    pins, 20                   
    0x8080, //  7: pull   noblock                    
    0xa047, //  8: mov    y, osr                     
    0x0095, //  9: jmp    y--, 21                    
    0xe085, // 10: set    pindirs, 5                 
    0xe010, // 11: set    pins, 16                   
    0xc002, // 12: irq    nowait 2                   
    0x2020, // 13: wait   0 pin, 0                   
    0xf62f, // 14: set    x, 15                  [22]
    0x4e01, // 15: in     pins, 1                [14]
    0x004f, // 16: jmp    x--, 15                    
    0x9020, // 17: push   block                  [16]
    0xe095, // 18: set    pindirs, 21                
    0xfd14, // 19: set    pins, 20               [29]
    0x0009, // 20: jmp    9                          
    0xa025, // 21: mov    x, status                  
    0x005d, // 22: jmp    x--, 29                    
    0x8080, // 23: pull   noblock                    
    0xef04, // 24: set    pins, 4                [15]
    0x6e01, // 25: out    pins, 1                [14]
    0x00f9, // 26: jmp    !osre, 25                  
    0xef14, // 27: set    pins, 20               [15]
    0x0009, // 28: jmp    9                          
    0xdf01, // 29: irq    nowait 1               [31]
    0x80a0, // 30: pull   block                      
    0xa047, // 31: mov    y, osr                     
            //     .wrap
);

RPI_PICO_PIO_DEFINE_PROGRAM(pio_slave_gbc, 0, 23,
    //     .wrap_target
    0xe010, //  0: set    pins, 16                   
    0xe084, //  1: set    pindirs, 4                 
    0xe02f, //  2: set    x, 15                      
    0x2020, //  3: wait   0 pin, 0                   
    0xd700, //  4: irq    nowait 0               [23]
    0x4e01, //  5: in     pins, 1                [14]
    0x0045, //  6: jmp    x--, 5                     
    0x8020, //  7: push   block                      
    0x80a0, //  8: pull   block                      
    0xe010, //  9: set    pins, 16                   
    0xe094, // 10: set    pindirs, 20                
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