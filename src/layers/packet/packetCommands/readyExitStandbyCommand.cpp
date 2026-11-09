#include "readyExitStandbyCommand.hpp"
#include "../../../link_defines.h"

struct SendExitStandbyContext : TransmitContext
{
    uint8_t index = 0;
};

TxValues readyExitStandbyCommandTransive(TransmitContext* ctx)
{
    auto* usbCtx = static_cast<SendExitStandbyContext*>(ctx);
    TxValues txValues;

    switch(usbCtx->index)
    {
        case 0:
            usbCtx->index++;
            txValues.push(LINKCMD_READY_EXIT_STANDBY);
            break;
        case 7:
            usbCtx->index = 0;
            txValues.push(0x00);
            break;
        default:
            usbCtx->index++;
            txValues.push(0x00);
            break;
    }

    return txValues;
}

TransmitBehaviour readyExitStandbyCommand()
{
    std::unique_ptr<SendExitStandbyContext> c = std::make_unique<SendExitStandbyContext>();
    c->index = 0;

    TransmitBehaviour transive
    {
        .context = std::move(c),
        .transmitCallback = readyExitStandbyCommandTransive,
        .transmitDoneCallback = [](TransmitContext* ctx){ (void)ctx; return CommandState::done; }
    };

    return transive;
}