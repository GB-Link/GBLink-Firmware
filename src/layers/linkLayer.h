#include <zephyr/kernel.h>

#pragma once

enum LinkMode
{
    MASTER,
    SLAVE,
    DISABLED
};

enum CableType
{
    GBA,
    GBC
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

/* GBA when SD last arrived on GP3 (port pin 4), GBC when on GP4 (port pin 1). Before any word in the
 * current session it reports the stored cable selection. */
enum CableType link_getDetectedCableType(void);

/* Cable selection as sent by SetCableOverride / SetCableSelection (wire encoding, distinct from CableType).
 * Stored and reported only: SD is read on both pins whatever the selection. */
enum CableSelection
{
    CABLE_AUTO = 0,
    CABLE_FORCE_GBA = 1,
    CABLE_FORCE_GBC = 2
};
void link_setCableOverride(uint8_t mode);
uint32_t link_getReceivedWordCount(void);

void link_configurePartnerPresence(void);
uint8_t link_readPartnerPins(void);
void link_releasePartnerPins(void);

void link_configurePokemonSlave(void);
void link_configureEreaderSlave(void);
void link_configureBattleChipGateSlave(void);
