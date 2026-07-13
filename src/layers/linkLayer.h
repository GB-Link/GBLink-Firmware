#include <zephyr/kernel.h>

#pragma once

enum LinkMode
{
    MASTER,
    SLAVE,
    DISABLED
};

struct NextTransmit
{
    uint16_t value;
    uint32_t timingUs;
};

typedef void (*ReceiveHandler)(uint16_t rx, void* userData);
typedef struct NextTransmit (*TransmitHandler)(void* userData);
typedef void (*TransiveDoneHandler)(uint16_t rx, uint16_t tx, void* userData);

void link_setTransmitCallback(TransmitHandler cb, void* userData);

void link_setReceiveCallback(ReceiveHandler cb, void* userData);

void link_setTransiveDoneCallback(TransiveDoneHandler cb, void* userData);

void link_startTransive();

enum LinkMode link_getMode();

void link_changeMode(enum LinkMode mode);

void link_detectCableType(void);

// Number of child SD slots the SLAVE PIO presents per multiplayer frame.
// 1 = the standard single-child slave (default). >1 selects the multi-slot
// SLAVE PIO so one adapter impersonates several children (3-4 player). Must be
// set before link_changeMode(SLAVE); reset to 1 on link_changeMode(DISABLED).
void link_setChildSlots(uint8_t n);

// Joiner-side multi-slot: the attached GBA is the LAST child (seat N-1) and the
// adapter is the bus master, driving txSlots = N-1 earlier slots before reading
// the GBA's word. Set before link_changeMode(MASTER); reset on DISABLED.
void link_setMasterMulti(uint8_t playerCount, uint8_t seat);
