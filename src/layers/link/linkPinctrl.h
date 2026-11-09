#pragma once

#include <zephyr/drivers/pinctrl.h>

#ifdef __cplusplus
extern "C"
{
#endif

const struct pinctrl_dev_config* linkPinctrl_getConfig(void);

#ifdef __cplusplus
}
#endif
