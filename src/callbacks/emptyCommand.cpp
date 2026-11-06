#include "emptyCommand.hpp"

namespace legacy
{

uint16_t emptyCommandTransive()
{
    return 0x00;
}

TransiveStruct emptyCommand()
{
    static TransiveStruct transive
    {
        .init = nullptr,
        .transive = emptyCommandTransive,
        .transiveDone = [](){ return CommandState::done; }
    };

    return transive;
}

}
