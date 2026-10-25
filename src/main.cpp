#include <zephyr/kernel.h>

#include "control.hpp"

#include "link_defines.h"

int main(void)
{
    Control g_control = Control();
    while (true)
    {
        g_control.executeMode();
    }
    
    // PacketLayer p;
    // p.configurePlayer(1, 2);
    // p.enableHandshake();
    // while (true)
    // {

    // }

    return 0;
}