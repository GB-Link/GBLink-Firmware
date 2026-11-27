#include "usbLinkCommand.hpp"
#include "../../../link_defines.h"
#include "TransiveStruct.hpp"
#include "zephyr/kernel.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <span>

static constexpr uint8_t commandBytesSize = 16;
static constexpr uint8_t metadataSize = 1;
static constexpr uint8_t packetSize = metadataSize + commandBytesSize;

// Queues should be synced by zephyr internally, so no need for atomics or mutexes.
// Besides, due to current structurings, usbLink_receiveHandler and loadTransivePacket
// run in ISR context, maybe improve later
K_MSGQ_DEFINE(g_packetQueue0, commandBytesSize, 200, 1);
K_MSGQ_DEFINE(g_packetQueue1, commandBytesSize, 200, 1);
K_MSGQ_DEFINE(g_packetQueue2, commandBytesSize, 200, 1);

static constexpr uint8_t unassignedSeat = 0xFF;

struct SeatQueueStatus
{
    uint8_t seat = unassignedSeat;  // absolute seat (0-3) whose packets this queue holds
    bool packetAvailable = false;
    uint16_t currentPacket[commandBytesSize / sizeof(uint16_t)] = {};
    k_msgq* queue;
};

std::array<SeatQueueStatus, 3> g_seatPacketQueues = {
    SeatQueueStatus
    {
        .packetAvailable = false,
        .currentPacket = {},
        .queue = &g_packetQueue0
    },
    SeatQueueStatus
    {
        .packetAvailable = false,
        .currentPacket = {},
        .queue = &g_packetQueue1
    },
    SeatQueueStatus
    {
        .packetAvailable = false,
        .currentPacket = {},
        .queue = &g_packetQueue2
    }
};

struct UsbLinkContext : TransmitContext
{
    uint8_t index = 0;
    uint8_t simulatedPartnereCount = 0;
    uint8_t seatNumber = 0;
    std::array<SeatQueueStatus, 3>* seatQueues = &g_seatPacketQueues;
};

void usbLink_receiveHandler(std::span<const uint8_t> data, void*)
{
    for (size_t packetOffset = 0; packetOffset + packetSize <= data.size(); packetOffset += packetSize)
    {
        const uint8_t seat = data[packetOffset];
        auto seatQueue = std::ranges::find(g_seatPacketQueues, seat, &SeatQueueStatus::seat);

        // own seat or invalid seat, no queue to put it in
        if (seatQueue == g_seatPacketQueues.end()) continue;

        auto commandPacket = data.subspan(packetOffset + metadataSize, commandBytesSize);
        k_msgq_put(seatQueue->queue, commandPacket.data(), K_NO_WAIT);
    }
}

static void loadNextPacket(TransmitContext* ctx)
{
    (void)ctx;
    for (size_t i = 0; i < sizeof(g_seatPacketQueues)/sizeof(SeatQueueStatus); i++)
    {
        SeatQueueStatus* seatQueue = &g_seatPacketQueues[i];
        seatQueue->packetAvailable = (k_msgq_get(seatQueue->queue, seatQueue->currentPacket, K_NO_WAIT) == 0);
    }
}

TxValues usbLinkTransmit(TransmitContext* ctx)
{   
    auto* usbCtx = static_cast<UsbLinkContext*>(ctx);
    TxValues txValues;

    bool packedShifted = false;
    for (uint8_t i = 0; i < usbCtx->simulatedPartnereCount; i++)
    {
        SeatQueueStatus* seatQueue = &g_seatPacketQueues[i];
        if (!seatQueue->packetAvailable)
        {
            txValues.push(0x00);
            continue;
        }
        
        uint16_t seatValue = (seatQueue->currentPacket[usbCtx->index]);
        packedShifted = true;

        // TODO Why does this happen? Only observed on Reconnect and only from slaves -> master and is concistent, so no random flip
        if (usbCtx->index == 0 && (seatValue == 0xFF02 || seatValue == 0xFF06 || seatValue == 0xFF07))
        {
            txValues.push(0x5FFF);
        }
        else 
        {
            txValues.push(seatValue);
        }
        
        if (usbCtx->index == 7)
        {
            seatQueue->packetAvailable = false;
        }
    }

    if (packedShifted) usbCtx->index++; 
    if (usbCtx->index == 8) usbCtx->index = 0;
    return txValues;
}

TransmitBehaviour usbLinkCommand(uint8_t ownSeat, uint8_t playerCount)
{
    std::unique_ptr<UsbLinkContext> p = std::make_unique<UsbLinkContext>();
    p->seatNumber = ownSeat;
    p->index = 0;
    p->simulatedPartnereCount = playerCount - 1;

    // queues hold the other seats in ascending order, which is the order the pio transmits them in
    uint8_t seat = 0;
    for (size_t i = 0; i < p->seatQueues->size(); i++)
    {
        if (seat == ownSeat) seat++;

        SeatQueueStatus* seatQueue = &p->seatQueues->at(i);
        seatQueue->seat = seat++;
        k_msgq_purge(seatQueue->queue);
        memset(seatQueue->currentPacket, 0, commandBytesSize);
        seatQueue->packetAvailable = false;
    }

    TransmitBehaviour transive
    {
        .context = std::move(p),
        .transmitCallback = usbLinkTransmit,
        .transmitDoneCallback = [](TransmitContext* ctx) 
        {
            loadNextPacket(ctx);
            return CommandState::resume; 
        }
    };

    return transive;
}