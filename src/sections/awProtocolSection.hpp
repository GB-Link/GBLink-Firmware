
#include <cstdint>
#include <span>
#include <zephyr/kernel.h>

#include "../layers/link/multiMode/multiMode.hpp"
#include "awProtocol.hpp"

#pragma once

// Zephyr glue around awproto::AwProxy: PIO link callbacks on the GBA side,
// MAW1 frame stream over the 64-byte data transport on the network side.
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
//   process()         module thread drain ring → 64-byte chunks → transport
class AwProtocolSection
{
public:
    AwProtocolSection(awproto::GameVariant variant, enum MultiMode mode);
    ~AwProtocolSection();

    void process();

    void cancel() { m_cancel = true; }

private:
    void pumpOutbound();

    bool m_cancel = false;
    bool m_reportedReconnecting = false;

    // Transport chunk being filled from the outbound ring. Frames may span
    // chunks; a chunk's unused tail is zero-padded (the remote stream parser
    // skips padding while seeking the frame magic).
    uint8_t m_chunk[64];
    size_t m_chunkLen = 0;
};

void awProto_receiveHandler(std::span<const uint8_t> data, void*);
