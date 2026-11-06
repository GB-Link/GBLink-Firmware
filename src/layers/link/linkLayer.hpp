
#include <zephyr/kernel.h>
#include "hardware/pio.h"

#include "../../common.hpp"

extern "C"
{
    #include "../linkLayer.h"
}

#pragma once

// The 2-4 player link of the GBA link mode (0x01). It borrows the state machine of the 2 player programs in
// linkLayer_pio.c, which take it back on their next configure.

/**
 * @brief Data for the next link transfer, returned by the TransmitHandler.
 */
struct NextTransmit
{
    uint8_t seatNumber;             ///< Index where connect GBA word is received [0-3]
    TxValues txValues;              ///< Words sent to the GBA;
    uint32_t timingUs;              ///< Master only: delay before the next transfer, in PIO cycles (~0.54 us)
};

typedef void (*ReceiveHandler)(uint16_t rx, void* userData);
typedef struct NextTransmit (*TransmitHandler)(void* userData);
typedef void (*TransiveDoneHandler)(uint16_t rx, TxValues, void* userData);

void link_setTransmitCallback(TransmitHandler cb, void* userData);

void link_setReceiveCallback(ReceiveHandler cb, void* userData);

void link_setTransiveDoneCallback(TransiveDoneHandler cb, void* userData);

void link_disable();

void link_enable();

// The master keeps the SD pin its GBA answered on across sessions. Called when another mode takes the link port,
// which may come with another cable.
void link_forgetSdPin();

// role: MASTER (the adapter clocks the link) or SLAVE (the GBA in slot 0 does).
void link_configureProgram(const pio_program_t* program, uint32_t wrap, uint32_t wrapTarget, enum LinkMode role);
