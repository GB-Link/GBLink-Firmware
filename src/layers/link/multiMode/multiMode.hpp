
#pragma once

enum MultiMode
{
    MASTER,
    SLAVE,
    DISABLED
};

void multiMode_selectMode(enum MultiMode mode);
