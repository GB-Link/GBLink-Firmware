#include <zephyr/kernel.h>
#include "TransiveStruct.hpp"

#pragma once

void blockCommandSetup(const void* src, uint16_t size, uint16_t blockMaxSize);

CommandState blockCommandChunk();

uint16_t blockCommandTransive();

TransiveStruct blockCommand();

bool blockCommandTransferComplete();
uint16_t blockCommandBytesSent();
void blockCommandConsumeComplete();
void blockCommandReset();


