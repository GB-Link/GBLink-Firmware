
#include <zephyr/kernel.h>
#include "hardware/pio.h"

#include "../../common.hpp"

#pragma once

/**
 * @brief Which SD pin the cable uses: GBA cable = GP3, GBC cable = GP4.
 */
enum CableType
{
    GBA,
    GBC
};

/**
 * @brief Data for the next link transfer, returned by the TransmitHandler.
 */
struct NextTransmit
{
    uint8_t seatNumber;             ///< Index where connect GBA word is received [0-3]
    TxValues txValues;              ///< Words sent to the GBA;
    uint32_t timingUs;              ///< Master only: delay before the next transfer, in PIO cycles (~0.54 us)
};

enum LinkPin
{
    SC,
    SI,
    SO,
    SD,
    SD_GBA,
    SD_GBC
};

enum LinkPinDir
{
    PIN_DIR_IN,
    PIN_DOR_OUT
};

typedef void (*ReceiveHandler)(uint16_t rx, void* userData);
typedef struct NextTransmit (*TransmitHandler)(void* userData);
typedef void (*TransiveDoneHandler)(uint16_t rx, TxValues, void* userData);

void link_setTransmitCallback(TransmitHandler cb, void* userData);

void link_setReceiveCallback(ReceiveHandler cb, void* userData);

void link_setTransiveDoneCallback(TransiveDoneHandler cb, void* userData);

void link_startTransive();

void link_disable();

void link_enable();

void link_setPioPinDirs(uint32_t pin, enum LinkPinDir direction);

uint32_t link_getPin(enum LinkPin pin);

void link_configureProgram(const pio_program_t* prgramm, uint32_t warp, uint32_t wrapTarget);

void link_configureCableType(enum CableType type);

/** Cable the modes should configure for; set from the persisted selection. */
void link_selectCableType(enum CableType type);

enum CableType link_getSelectedCableType();

uint32_t link_receivedWordCount();

uint8_t link_readPartnerPins();