#include "blockCommandRequestCommand.hpp"
#include "../../../link_defines.h"

struct BloclCommandRequestContext : TransmitContext
{
    uint8_t index = 0;
    uint16_t type = 0;
};

TxValues blockCommandRequestTransive(TransmitContext* ctx)
{
    auto* blkCxt = static_cast<BloclCommandRequestContext*>(ctx);
    TxValues txValues;

    switch(blkCxt->index)
    {
        case 0:
            blkCxt->index++;
            txValues.push(LINKCMD_SEND_LINK_TYPE);
            break;
        case 1:
            blkCxt->index++;
            txValues.push(blkCxt->type);
            break;
        case 7:
            blkCxt->index = 0;
            txValues.push(0x00);
            break;
        default:
            blkCxt->index++;
            txValues.push(0x00);
            break;
    }

    return txValues;
}

TransmitBehaviour sendBlockCommandRequestCommand(uint16_t type)
{
    std::unique_ptr<BloclCommandRequestContext> c = std::make_unique<BloclCommandRequestContext>();
    c->index = 0;
    c->type = type;

    TransmitBehaviour transive
    {
        .context = std::move(c),
        .transmitCallback = blockCommandRequestTransive,
        .transmitDoneCallback = [](TransmitContext* ctx){ return CommandState::done; }
    };

    return transive;
}