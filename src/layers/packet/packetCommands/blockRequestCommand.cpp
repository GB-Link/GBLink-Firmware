#include "blockRequestCommand.hpp"
#include "TransiveStruct.hpp"

struct BlockRequestContext : TransmitContext
{
    uint8_t index = 0;
    uint16_t blockCommandContent[8] = {0xCCCC, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
};


TxValues blockRequestTransive(TransmitContext* ctx)
{
    auto* blockCtx = static_cast<BlockRequestContext*>(ctx);
    TxValues txValues;
    txValues.push(blockCtx->blockCommandContent[blockCtx->index]);

    blockCtx->index++;
    if (blockCtx->index == 8)
    {
        blockCtx->index = 0;
    } 

    return txValues;
}

TransmitBehaviour blockRequest()
{
    std::unique_ptr<BlockRequestContext> c = std::make_unique<BlockRequestContext>();
    c->index = 0;

    TransmitBehaviour transive
    {
        .context = std::move(c),
        .transmitCallback = blockRequestTransive,
        .transmitDoneCallback = [](TransmitContext* ctx){ (void)ctx; return CommandState::done; }
    };

    return transive;
}