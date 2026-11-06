#include <zephyr/kernel.h>
#include "TransiveStruct.hpp"

#pragma once 

namespace legacy
{

void moveCommandInit(uint16_t data);

uint16_t moveCommandTransive();

TransiveStruct moveCommand();

}
