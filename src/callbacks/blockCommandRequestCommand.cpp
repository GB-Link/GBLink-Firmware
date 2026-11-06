#include "blockCommandRequestCommand.hpp"
#include "../link_defines.h"

namespace legacy
{

static uint8_t index = 0;
static uint16_t g_type = 0;

uint16_t blockCommandRequestTransive()
{
    switch(index)
    {
        case 0:
            index++;
            return LINKCMD_SEND_BLOCK_REQ;
        case 1:
            index++;
            return g_type;
        case 7:
            index = 0;
            return 0x00;
        default:
            index++;
         return 0x00;
    }
}

TransiveStruct sendBlockCommandRequestCommand(uint16_t type)
{
    g_type = type;
    static TransiveStruct transive
    {
        .init = nullptr,
        .transive = blockCommandRequestTransive,
        .transiveDone = [](){ return CommandState::done; }
    };

    return transive;
}

}
