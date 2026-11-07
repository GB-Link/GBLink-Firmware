#include "emptyCommand.hpp"
#include "TransiveStruct.hpp"

static TxValues emptyCommandTransive(TransmitContext* ctx)
{
    (void)ctx;
    TxValues txValues;
    txValues.push(0x00);
    return txValues;
}

TransmitBehaviour emptyCommand()
{
    TransmitBehaviour transive
    {
        .context = nullptr,
        .transmitCallback = emptyCommandTransive,
        .transmitDoneCallback = [](TransmitContext* ctx){ (void)ctx; return CommandState::done; }
    };

    return transive;
}