
#include <memory>
#include <array>
#include "../../../common.hpp"

#pragma once

enum class CommandState
{
    resume,
    done
};

struct TransmitContext
{
    virtual ~TransmitContext() = default;
};

class TransmitBehaviour
{
public:
    using TransiveCallback = TxValues(*)(TransmitContext*);
    using TransiveDoneCallback = CommandState(*)(TransmitContext*);

    std::unique_ptr<TransmitContext> context;

    TransiveCallback transmitCallback;
    TransiveDoneCallback transmitDoneCallback;

    TxValues transmit()
    {
        return transmitCallback(context.get());
    }

    CommandState transmitDone()
    {
        return transmitDoneCallback(context.get());
    }

};