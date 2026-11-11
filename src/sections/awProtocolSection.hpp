
#include <cstdint>
#include <span>
#include <zephyr/kernel.h>

#include "../layers/link/multiMode/multiMode.hpp"
#include "awProtocol.hpp"

#pragma once

// Zephyr glue around awproto::AwProxy: PIO link callbacks on the GBA side,
// the players' MAW1 frame streams over the 51-byte data transport on the
// network side. Every transport packet starts with the seat it comes from.
//
// The proxy, stream parser, and outbound ring live in file-scope statics
// (like the section message queues elsewhere in this codebase) so the ISR
// and transport-receive paths never dereference the stack-allocated section
// — a late callback after teardown writes into static state instead of
// freed memory.
//
// Context map:
//   transmitCallback  PIO tx-ISR    O(1) read of the staged response
//   receiveCallback   PIO done-ISR  FSM step + stage next response
//   emit (via proxy)  PIO done-ISR  serialize frame into the outbound ring
//   awProto_receiveHandler  USB thread / UART ISR  parse frames, push queue
//   process()         module thread drain ring → 51-byte chunks → transport
class AwProtocolSection
{
public:
    // seat: the GBA's, 0 for the bus master (player 1). playerCount: 2 to 4.
    AwProtocolSection(awproto::GameVariant variant, uint8_t seat, uint8_t playerCount);
    ~AwProtocolSection();

    void process();

    void cancel() { m_cancel = true; }

private:
    void pumpOutbound();

    bool m_cancel = false;
    bool m_reportedReconnecting = false;

    // Transport chunk being filled from the outbound ring: the seat, then the
    // frame stream. Frames may span chunks; a chunk's unused tail is
    // zero-padded (the remote stream parser skips padding while seeking the
    // frame magic). 51 bytes: the Celio client and server carry data in
    // packets of that size.
    static constexpr size_t chunkHeader = 1;
    uint8_t m_chunk[51];
    size_t m_chunkLen = chunkHeader;
};

void awProto_receiveHandler(std::span<const uint8_t> data, void*);
