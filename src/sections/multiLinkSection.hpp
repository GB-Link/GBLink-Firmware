#pragma once

#include <cstdint>
#include <span>

extern "C"
{
    #include "../layers/linkLayer.h"
}

// 3-4 player gen-3 link relay. Each player's adapter runs a private MULTI bus
// to its own cart and the session is stitched together over the network.
//
// The gen-3 link driver frames the wire into 9-transfer command windows
// (1 checksum word + 8 command words) and checksums all slots across the
// window, so raw word relaying cannot survive network latency. Instead, each
// adapter mirrors the game's own link state machine on its bus:
//
//  - Handshake phase: seats' presence ("cart is handshake-ready") is relayed
//    and the adapter synthesizes the constant handshake words for present
//    seats. When the game on seat 0's cart advances the link, that cart emits
//    a one-shot 0x8FFF; seat 0's adapter establishes and broadcasts GO, on
//    which every joiner bus presents 0x8FFF and establishes the same way its
//    cart does.
//
//  - Established phase: the relay unit is a whole 8-word command, delivered
//    exactly once and in order via per-seat queues (all-zero idle windows
//    when starved, which is the protocol's true idle). Each window's checksum
//    word is forged locally as the sum of everything on this bus, so the
//    game's cross-slot checksum holds on every bus by construction.
//
// Sessions are numbered by a generation that seat 0 mints at each
// establishment; frames from dead generations are dropped so one activity's
// teardown cannot leak into the next.
//
// Host (seat 0): the cart is the bus parent -> slave-multi PIO presenting the
// other seats as children; the cart paces the wire. Joiner: the cart is a
// child -> master-multi PIO with the adapter as bus master, pacing like a
// real gen-3 master with a drift servo against seat 0's window counter.
class MultiLinkSection
{
public:
    MultiLinkSection(uint8_t seat, uint8_t playerCount);
    ~MultiLinkSection();

    void process();

    void cancel() { m_cancel = true; }

private:
    static struct NextTransmit transmitCallback(void* userData);
    static void receiveCallback(uint16_t rx, void* userData);
    static void transiveDoneCallback(uint16_t rx, uint16_t tx, void* userData);

    bool m_cancel = false;
};

// Inbound network frames (byte 0 = tag, low 2 bits = seat):
//   0xA0|seat  presence      [tag, handshakeReady]                 2 bytes
//   0xA4|seat  command       [tag, gen, 8 x u16 LE]               18 bytes
//   0xA8|seat  window count  [tag, gen, u32 LE]                    6 bytes
//   0xAC       GO            [tag, playerCount, gen]               3 bytes
// Registered as the transport data handler while the section runs.
void multiLink_receiveHandler(std::span<const uint8_t> data, void*);
