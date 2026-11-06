#include "readyExitStandbyCommand.hpp"
#include "../link_defines.h"

namespace legacy
{

static uint8_t index = 0;
static uint16_t g_type = 0;

uint16_t sendLinkTypeCommandTransive()
{
    switch(index)
    {
        case 0:
            index++;
            return LINKCMD_SEND_LINK_TYPE;
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

TransiveStruct sendLinkTypeCommand(uint16_t type)
{
    g_type = type;
    static TransiveStruct transive
    {
        .init = nullptr,
        .transive = sendLinkTypeCommandTransive,
        .transiveDone = [](){ return CommandState::done; }
    };

    return transive;
}

}
