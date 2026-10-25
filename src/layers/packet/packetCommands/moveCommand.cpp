#include "moveCommand.hpp"

struct MoveContext : TransmitContext
{
    uint8_t index = 0;
    size_t repeats = 0;
    uint16_t move;
};

TxValues moveCommandTransive(TransmitContext* ctx)
{
    auto* moveCtx = static_cast<MoveContext*>(ctx);
    TxValues txValues;

    switch(moveCtx->index)
    {
        case 0:
            moveCtx->index++;
            txValues.push(0xCAFE);
            break;
        case 1:
            moveCtx->index++;
            txValues.push(moveCtx->move);
            break;
        case 7:
            moveCtx->index = 0;
            txValues.push(0x00);
            break;
        default:
            moveCtx->index++;
            txValues.push(0x00);
            break;
    }

    return txValues;
}

TransmitBehaviour moveCommand(int16_t data)
{
    std::unique_ptr<MoveContext> c = std::make_unique<MoveContext>();
    c->move = data;

    TransmitBehaviour transive
    {
        .context = std::move(c),
        .transmitCallback = moveCommandTransive,
        .transmitDoneCallback = [](TransmitContext* ctx)
        { 
            auto* moveCtx = static_cast<MoveContext*>(ctx);
            if (moveCtx->repeats != 20)
            {
                moveCtx->repeats++;
                return CommandState::resume;   
            }
            return CommandState::done; 
        }
    };

    return transive;
}