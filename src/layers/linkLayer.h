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

uint8_t link_getDetectedCableType(void);
void link_setCableOverride(uint8_t mode);
void link_flipSdPath(void);
uint32_t link_getReceivedWordCount(void);

void link_configurePartnerPresence(void);
uint8_t link_readPartnerPins(void);
void link_releasePartnerPins(void);

void link_configurePokemonSlave(void);
void link_configureMultiSlave(void);
