#include <zephyr/kernel.h>
#include "TransiveStruct.hpp"

#pragma once

TransmitBehaviour blockCommand(const void* src, uint16_t size, uint16_t blockMaxSize);
