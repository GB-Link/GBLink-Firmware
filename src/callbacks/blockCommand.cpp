#include "blockCommand.hpp"
#include "../link_defines.h"

#include <string.h>

#define MAX_CHUNK 14
#define BLOCK_SIZE_INDEX 1

static const void* g_src;
static uint16_t g_srcSize;
static uint16_t g_pos;

static uint16_t g_index = 0;
static bool g_initSend = false;
static uint16_t g_blockMaxSize = 0;
static bool g_transferComplete = false;

static uint16_t g_blockCommandInit[8] = {
    LINKCMD_INIT_BLOCK, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint16_t g_blockCommandContent[8] = {LINKCMD_CONT_BLOCK, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


CommandState blockCommandChunk()
{
    if (g_blockMaxSize == 0) { g_transferComplete = true; return CommandState::done; }

    if (!g_initSend) return CommandState::resume;

    memset((uint8_t*)g_blockCommandContent + 2, 0x00, MAX_CHUNK);

    uint16_t chunkSize = g_srcSize < MAX_CHUNK ? g_srcSize : MAX_CHUNK;
    uint16_t maxChunkSize = g_blockMaxSize < MAX_CHUNK ? g_blockMaxSize : MAX_CHUNK;

    if (g_blockMaxSize > 0)
    {
        memcpy((uint8_t*)g_blockCommandContent + 2, (uint8_t*)g_src + g_pos, chunkSize);
    }
    
    g_srcSize -= chunkSize;
    g_blockMaxSize -= maxChunkSize;
    g_pos += chunkSize;
    return CommandState::resume;
}

void blockCommandSetup(const void* src, uint16_t size, uint16_t blockMaxSize)
{
    g_src = src;
    g_srcSize = size;
    g_index = 0;
    g_initSend = false;
    g_pos = 0;
    g_blockMaxSize = blockMaxSize;
    g_transferComplete = false;
    g_blockCommandInit[BLOCK_SIZE_INDEX] = blockMaxSize;
}

bool blockCommandTransferComplete() { return g_transferComplete; }
uint16_t blockCommandBytesSent() { return g_pos; }
void blockCommandConsumeComplete() { g_transferComplete = false; }
void blockCommandReset()
{
    g_src = nullptr;
    g_srcSize = 0;
    g_pos = 0;
    g_index = 0;
    g_initSend = false;
    g_blockMaxSize = 0;
    g_transferComplete = false;
}

uint16_t blockCommandTransive()
{
    uint16_t ret = g_initSend ? g_blockCommandContent[g_index] : g_blockCommandInit[g_index];
    
    g_index++;
    if (g_index == 8)
    {
        g_index = 0;
        g_initSend = true;
    } 

    return ret;
}

TransiveStruct blockCommand() 
{
    static TransiveStruct transive
    {
        .init = nullptr,
        .transive = blockCommandTransive,
        .transiveDone = blockCommandChunk
    };

    return transive;
}
