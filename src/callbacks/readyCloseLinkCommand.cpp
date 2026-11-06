#include "readyCloseLinkCommand.hpp"
#include "../link_defines.h"

namespace legacy
{

static uint8_t index = 0;

uint16_t readyCloseLinkTransive()
{
    switch(index)
    {
        case 0:
            index++;
            return LINKCMD_READY_CLOSE_LINK;
        case 7:
            index = 0;
            return 0x00;
        default:
            index++;
         return 0x00;
    }
}

TransiveStruct readyCloseLinkCommand()
{
    static TransiveStruct transive
    {
        .init = nullptr,
        .transive = readyCloseLinkTransive,
        .transiveDone = [](){ return CommandState::done; }
    };

    return transive;
}

}
