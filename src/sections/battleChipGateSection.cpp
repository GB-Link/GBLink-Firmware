#include "battleChipGateSection.hpp"

#include <cstring>
#include <new>

#include "../layers/transport.hpp"

extern "C"
{
    #include "../layers/linkLayer.h"
}

static bcgproto::BcgProxy g_proxy;
static volatile bool g_sessionActive = false;

static constexpr int kStatusIntervalLoops = 50;
static constexpr int kLoopSleepMs = 10;
static constexpr uint32_t kSlaveTimingUs = 15370;

static struct LegacyTransmit transmitCallback(void*)
{
    return { g_proxy.stagedTx, kSlaveTimingUs };
}

static void receiveCallback(uint16_t rx, void*)
{
    g_proxy.onRound(rx);
}

void bcgProto_receiveHandler(std::span<const uint8_t> data, void*)
{
    if (!g_sessionActive || data.size() < 3) return;

    const bcgproto::GateType gateType = bcgproto::gateTypeFromVariant(data[0]);
    const uint16_t chipId =
        static_cast<uint16_t>(data[1]) |
        (static_cast<uint16_t>(data[2]) << 8);

    const unsigned int key = irq_lock();
    g_proxy.applyHostPacket(gateType, chipId);
    irq_unlock(key);
}

static void emitStatus()
{
    uint8_t pkt[64] = {};
    std::memcpy(pkt, bcgproto::BCGS_MAGIC, 4);
    const uint32_t rx = link_getReceivedWordCount();
    pkt[4] = static_cast<uint8_t>(rx);
    pkt[5] = static_cast<uint8_t>(rx >> 8);
    pkt[6] = static_cast<uint8_t>(rx >> 16);
    pkt[7] = static_cast<uint8_t>(rx >> 24);
    const uint16_t chip = g_proxy.chipId;
    pkt[8] = static_cast<uint8_t>(chip);
    pkt[9] = static_cast<uint8_t>(chip >> 8);
    pkt[10] = static_cast<uint8_t>(static_cast<int8_t>(g_proxy.stage));
    // pkt[11] stays 0: clients read it as an SD-path flag, and SD is read on both pins.
    const uint16_t staged = g_proxy.stagedTx;
    pkt[12] = static_cast<uint8_t>(staged);
    pkt[13] = static_cast<uint8_t>(staged >> 8);
    Transport::sendData(std::span<const uint8_t>(pkt, sizeof(pkt)));
}

BattleChipGateSection::BattleChipGateSection(bcgproto::GateType initialGateType)
{
    new (&g_proxy) bcgproto::BcgProxy();
    g_proxy.reset(initialGateType);

    link_setLegacyTransmitCallback(&transmitCallback, nullptr);
    link_setLegacyReceiveCallback(&receiveCallback, nullptr);

    g_sessionActive = true;
}

BattleChipGateSection::~BattleChipGateSection()
{
    g_sessionActive = false;
    link_setLegacyTransmitCallback(nullptr, nullptr);
    link_setLegacyReceiveCallback(nullptr, nullptr);
}

void BattleChipGateSection::armLink()
{
    link_configureBattleChipGateSlave();
}

void BattleChipGateSection::process()
{
    int statusLoops = 0;
    uint32_t lastReportedRx = 0;

    while (!m_cancel)
    {
        if (++statusLoops >= kStatusIntervalLoops)
        {
            statusLoops = 0;
            const uint32_t rx = link_getReceivedWordCount();
            if (rx != lastReportedRx || rx == 0)
            {
                lastReportedRx = rx;
                emitStatus();
            }
        }

        k_sleep(K_MSEC(kLoopSleepMs));
    }
}
