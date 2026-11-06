#include "moveCommand.hpp"

namespace legacy
{

static uint8_t g_index = 0;
static size_t g_repeats = 0;
static uint16_t g_move;

void moveCommandInit(uint16_t data)
{
    g_index = 0;
    g_repeats = 0;
    g_move = data;
}

uint16_t moveCommandTransive()
{
    switch(g_index)
    {
        case 0:
            g_index++;
            return 0xCAFE;
        case 1:
            g_index++;
            return g_move;
        case 7:
            g_index = 0;
            return 0x00;
        default:
            g_index++;
         return 0x00;
    }
}

TransiveStruct moveCommand()
{
    static TransiveStruct transive
    {
        .init = nullptr,
        .transive = moveCommandTransive,
        .transiveDone = []()
        { 
            if (g_repeats != 20)
            {
                g_repeats++;
                return CommandState::resume;   
            }
            return CommandState::done; 
        }
    };

    return transive;
}

}
