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

// True when the last detection saw a GBA cable (GP1 grounded). The GBC cable
// is the one whose GP1 carries the GBA's SO — required by full-duplex modes
// (RFU wireless adapter emulation).
bool link_isGbaCable(void);
