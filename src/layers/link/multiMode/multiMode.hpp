
#pragma once

extern "C"
{
    #include "../../linkLayer.h"
}

// MASTER, SLAVE and DISABLED are enum LinkMode of linkLayer.h, which the 2 player programs use too.

void multiMode_selectMode(enum LinkMode mode);
