#include "readyExitStandbyCommand.hpp"
#include "../../../link_defines.h"

struct SendLinkTypeContext : TransmitContext
{
    uint8_t index = 0;
    uint16_t type = 0;
};

TxValues sendLinkTypeCommandTransive(TransmitContext* ctx)
{
    auto* usbCtx = static_cast<SendLinkTypeContext*>(ctx);
    TxValues txValues;

    switch(usbCtx->index)
    {
        case 0:
            usbCtx->index++;
            txValues.push(LINKCMD_SEND_LINK_TYPE);
            break;
        case 1:
            usbCtx->index++;
            txValues.push(usbCtx->type);
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

TransmitBehaviour sendLinkTypeCommand(uint16_t type)
{
    std::unique_ptr<SendLinkTypeContext> c = std::make_unique<SendLinkTypeContext>();
    c->index = 0;
    c->type = type;

    TransmitBehaviour transive
    {
        .context = std::move(c),
        .transmitCallback = sendLinkTypeCommandTransive,
        .transmitDoneCallback = [](TransmitContext* ctx){ (void)ctx; return CommandState::done; }
    };

    return transive;
}