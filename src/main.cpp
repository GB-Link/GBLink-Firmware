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

    return 0;
}