
#include <cstdint>
#include <span>

extern "C"
{
    #include "../layers/linkLayer_rfu.h"
}

#include <zephyr/kernel.h>

#include "rfuProtocol.hpp"

#pragma once

// Zephyr glue around rfuproto::RfuCore: PIO1 link callbacks on the GBA side,
// RFU1 frame stream over the 64-byte data transport on the network side.
//
// Like the AW section, all protocol state lives in file-scope statics so a
// late ISR or transport callback never dereferences the stack-allocated
// section after teardown.
//
// Context map:
//   PIO done callback   PIO1 IRQ0   slave role: FSM step + stage next response
//   emit (via core)     PIO ISR / transport ctx / section thread —
//                       serialized into the outbound ring under irq_lock
//   awRfu receive       USB thread / UART ISR  parse frames, apply to core
//   process()           module thread: pump ring, wait events + synchronous
//                       reverse delivery, broadcast tick, link supervision
class RfuProtocolSection
{
public:
    explicit RfuProtocolSection(uint8_t role = 0);
    ~RfuProtocolSection();

    void process();

    void cancel() { m_cancel = true; }

    void feedNetworkBytes(std::span<const uint8_t> data);

private:
    void runDelivery();
    void pumpOutbound();
    void superviseLink(uint32_t nowMs);
    void updateDiagnosticLed();
    void reportDiagnostics(uint32_t nowMs);

    bool m_cancel = false;
    bool m_reportedReconnecting = false;
    int m_lastDiag = -1;
    uint32_t m_lastDiagReportMs = 0;

    // Transport chunk being filled from the outbound ring (frames span
    // chunks; chunk tails are zero padded — the parser skips padding).
    uint8_t m_chunk[64];
    size_t m_chunkLen = 0;
};

void rfuProto_receiveHandler(std::span<const uint8_t> data, void*);
