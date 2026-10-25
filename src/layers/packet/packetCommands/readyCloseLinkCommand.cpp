#include "readyCloseLinkCommand.hpp"
#include "../../../link_defines.h"

struct SendReadyExitStandbyContext : TransmitContext
{
    uint8_t index = 0;
};

TxValues readyCloseLinkTransive(TransmitContext* ctx)
{
    auto* usbCtx = static_cast<SendReadyExitStandbyContext*>(ctx);
    TxValues txValues;

    switch(usbCtx->index)
    {
        case 0:
            usbCtx->index++;
            txValues.push(LINKCMD_READY_CLOSE_LINK);
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

TransmitBehaviour readyCloseLinkCommand()
{
    std::unique_ptr<SendReadyExitStandbyContext> c = std::make_unique<SendReadyExitStandbyContext>();
    c->index = 0;

    TransmitBehaviour transive
    {
        .context = std::move(c),
        .transmitCallback = readyCloseLinkTransive,
        .transmitDoneCallback = [](TransmitContext* ctx){ return CommandState::done; }
    };

    return transive;
}