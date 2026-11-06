
#include <span>
#pragma once

namespace legacy
{

enum class CommandState
{
    resume,
    done
};

struct TransiveStruct
{
    using InitCallback = void(*)(std::span<const uint16_t>);
    using TransiveCallback = uint16_t(*)();
    using TransiveDoneCallback = CommandState(*)();

    std::span<const uint16_t> userData;

    InitCallback init;
    TransiveCallback transive;
    TransiveDoneCallback transiveDone;
};

}
