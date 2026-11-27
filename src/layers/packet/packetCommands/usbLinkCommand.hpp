#include <zephyr/kernel.h>
#include "TransiveStruct.hpp"

TransmitBehaviour usbLinkCommand(uint8_t ownSeat, uint8_t playerCount);

void usbLink_receiveHandler(std::span<const uint8_t> data, void*);