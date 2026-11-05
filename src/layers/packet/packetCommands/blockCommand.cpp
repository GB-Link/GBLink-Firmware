#include "blockCommand.hpp"
#include "../../../link_defines.h"
#include "TransiveStruct.hpp"

#include <string.h>

#define MAX_CHUNK 14
#define BLOCK_SIZE_INDEX 1

struct BlockCommandContext : TransmitContext
{
    const void* src;
    uint16_t srcSize;
    uint16_t pos;

    uint16_t index = 0;
    bool initSend = false;
    uint16_t blockMaxSize = 0;

    uint16_t blockCommandInit[8] = { LINKCMD_INIT_BLOCK, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint16_t blockCommandContent[8] = {LINKCMD_CONT_BLOCK, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
};

static CommandState blockCommandChunk(TransmitContext* ctx)
{
    auto* blockCtx = static_cast<BlockCommandContext*>(ctx);
    if (blockCtx->blockMaxSize == 0) return CommandState::done;

    if (!blockCtx->initSend) return CommandState::resume;

    memset((uint8_t*)blockCtx->blockCommandContent + 2, 0x00, MAX_CHUNK);

    uint16_t chunkSize = blockCtx->srcSize < MAX_CHUNK ? blockCtx->srcSize : MAX_CHUNK;
    uint16_t maxChunkSize = blockCtx->blockMaxSize < MAX_CHUNK ? blockCtx->blockMaxSize : MAX_CHUNK;

    if (blockCtx->blockMaxSize > 0)
    {
        memcpy((uint8_t*)blockCtx->blockCommandContent + 2, (uint8_t*)blockCtx->src + blockCtx->pos, chunkSize);
    }
    
    blockCtx->srcSize -= chunkSize;
    blockCtx->blockMaxSize -= maxChunkSize;
    blockCtx->pos += chunkSize;
    return CommandState::resume;
}

static TxValues blockCommandTransive(TransmitContext* ctx)
{
    auto* blockCtx = static_cast<BlockCommandContext*>(ctx);
    TxValues txValues;

    txValues.push(blockCtx->initSend ? blockCtx->blockCommandContent[blockCtx->index] : blockCtx->blockCommandInit[blockCtx->index]);
    
    blockCtx->index++;
    if (blockCtx->index == 8)
    {
        blockCtx->index = 0;
        blockCtx->initSend = true;
    } 

    return txValues;
}

TransmitBehaviour blockCommand(const void* src, uint16_t size, uint16_t blockMaxSize) 
{
    std::unique_ptr<BlockCommandContext> c = std::make_unique<BlockCommandContext>();

    c->src = src;
    c->srcSize = size;
    c->index = 0;
    c->initSend = false;
    c->pos = 0;
    c->blockMaxSize = blockMaxSize;
    c->blockCommandInit[BLOCK_SIZE_INDEX] = blockMaxSize;

    TransmitBehaviour transive
    {
        .context = std::move(c),
        .transmitCallback = blockCommandTransive,
        .transmitDoneCallback = blockCommandChunk
    };

    return transive;
}
