#include "readyExitStandbyCommand.hpp"
#include "../link_defines.h"

namespace legacy
{

static uint8_t index = 0;

uint16_t dummyTransive()
{
    switch(index)
    {
        case 0:
            index++;
            return LINKCMD_DUMMY_1;
        case 7:
            index = 0;
            return 0x00;
        default:
            index++;
         return 0x00;
    }
}

TransiveStruct dummyCommand()
{
    static TransiveStruct transive
    {
        .init = nullptr,
        .transive = dummyTransive,
        .transiveDone = [](){ return CommandState::done; }
    };

    return transive;
}

}
