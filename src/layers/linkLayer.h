#include <zephyr/kernel.h>

#pragma once

enum LinkMode
{
    MASTER,
    SLAVE,
    DISABLED
};

/* 2 player programs, one word per transfer. The GBA link mode (0x01) uses the 2-4 player programs in layers/link. */
struct LegacyTransmit
{
    uint16_t value;
    uint32_t timingUs;
};

typedef void (*LegacyReceiveHandler)(uint16_t rx, void* userData);
typedef struct LegacyTransmit (*LegacyTransmitHandler)(void* userData);
typedef void (*LegacyTransiveDoneHandler)(uint16_t rx, uint16_t tx, void* userData);

void link_setLegacyTransmitCallback(LegacyTransmitHandler cb, void* userData);

void link_setLegacyReceiveCallback(LegacyReceiveHandler cb, void* userData);

void link_setLegacyTransiveDoneCallback(LegacyTransiveDoneHandler cb, void* userData);

void link_startTransive();

enum LinkMode link_getMode();

void link_changeMode(enum LinkMode mode);

uint32_t link_getReceivedWordCount(void);

void link_configurePartnerPresence(void);
uint8_t link_readPartnerPins(void);
void link_releasePartnerPins(void);

void link_configurePokemonSlave(void);
void link_configureEreaderSlave(void);
void link_configureBattleChipGateSlave(void);

/* The 2-4 player programs (layers/link) run on the same state machine. link_lendStateMachine stops the program
 * running and returns the state machine. Until the next link_changeMode or link_configure* call its interrupts go
 * to irq0 (PIO irq 0) and irq1 (PIO irqs 1 and 2); that call runs revoke once the state machine has stopped. */
typedef void (*LinkIrqHandler)(void);
uint32_t link_lendStateMachine(LinkIrqHandler irq0, LinkIrqHandler irq1, LinkIrqHandler revoke);
