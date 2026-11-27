#include "linkPinctrl.h"

#include <zephyr/devicetree.h>

PINCTRL_DT_DEFINE(DT_NODELABEL(pio_link));

const struct pinctrl_dev_config* linkPinctrl_getConfig(void)
{
    return PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(pio_link));
}
