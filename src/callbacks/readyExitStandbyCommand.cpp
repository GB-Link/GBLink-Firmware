#include "readyExitStandbyCommand.hpp"
#include "../link_defines.h"

namespace legacy
{

static uint8_t index = 0;

uint16_t readyExitStandbyCommandTransive()
{
    switch(index)
    {
        case 0:
            index++;
            return LINKCMD_READY_EXIT_STANDBY;
        case 7:
            index = 0;
            return 0x00;
        default:
            index++;
         return 0x00;
    }
}

TransiveStruct readyExitStandbyCommand()
{
    static TransiveStruct transive
    {
        .init = nullptr,
        .transive = readyExitStandbyCommandTransive,
        .transiveDone = [](){ return CommandState::done; }
    };

    return transive;
}

}
