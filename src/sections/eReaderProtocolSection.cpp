#include "eReaderProtocolSection.hpp"

#include <cstring>
#include <new>

#include <zephyr/sys/ring_buffer.h>

#include "../callbacks/blockCommand.hpp"
#include "../callbacks/commands.hpp"
#include "../layers/packetLayer.hpp"
#include "../layers/transport.hpp"
#include "../link_defines.h"

extern "C"
{
    #include "../layers/linkLayer.h"
}

RING_BUF_DECLARE(g_erOutRing, 2048);

static erproto::EReaderProxy g_proxy;
static volatile bool g_sessionActive = false;
static bool g_multiPioActive = false;

// PIO (re)configuration sleeps, so the transport RX handler only flags it and
// the section thread performs it from process().
static volatile bool g_armRequested = false;

// Wrong-pin watchdog: cable detection is a heuristic (a GBA parked on a link
// screen, or a cable plugged in after detection, makes it pick the wrong SD
// pin). The partner clocking SC while zero words arrive is proof of the wrong
// pin — SC is the same pin for both cable types — so after sustained evidence
// the section flips the SD path once per scan and re-arms.
static constexpr int kWrongPinIntervals = 4;
static bool g_sdPathFlipped = false;
static int g_wrongPinIntervals = 0;

// Raw storage instead of a static PacketLayer: its constructor self-registers
// the link ISR callbacks, which must not happen at boot for a layer that only
// lives during Pokemon e-reader sessions.
alignas(PacketLayer) static uint8_t g_pokemonPacketStorage[sizeof(PacketLayer)];
static bool g_pokemonPacketLive = false;

static PacketLayer& pokemonPacket()
{
    return *std::launder(reinterpret_cast<PacketLayer*>(g_pokemonPacketStorage));
}

static struct NextTransmit sma4TransmitCallback(void*);
static void sma4ReceiveCallback(uint16_t rx, void*);

static bool g_pokeLinkPlayerPending = false;
static bool g_pokePayloadPending = false;
static uint8_t g_pokeLinkPlayerBuf[60];
static bool g_lpFinished = false;

static constexpr uint16_t kLpPostWaitRounds = 300;
static uint16_t g_lpPostWaitCount = 0;

static constexpr size_t kPokeLinkPlayerBytes = 60;

static void erProto_shutdownPokemonPacketLayer();
static void erProto_initPokemonPacketLayer();

static void ensurePioLink()
{
    if (g_multiPioActive) return;

    // Recycle the packet layer BEFORE touching the PIO (the wrong-pin flip
    // re-arms with it live): ~PacketLayer disables the link, so running it
    // after would kill the mode configured just below. The cable type is NOT
    // re-sampled here — it was captured at mode entry, before the GBA's link
    // screens start driving SO low.
    erProto_shutdownPokemonPacketLayer();

    if (erproto::isSma4Profile(g_proxy.profile))
        link_configureEreaderSlave();
    else
    {
        link_configurePartnerPresence();
        k_sleep(K_MSEC(200));
        link_configurePokemonSlave();
    }
    g_multiPioActive = true;

    if (!erproto::isSma4Profile(g_proxy.profile))
        erProto_initPokemonPacketLayer();

    g_proxy.emitWire(erproto::WIRE_FLAG_PIO_ARMED, 0, erproto::poke::kSlaveHandshake);
}

static void erProto_shutdownPokemonPacketLayer()
{
    if (!g_pokemonPacketLive) return;
    pokemonPacket().cancel();
    pokemonPacket().~PacketLayer();
    // The constructor self-registered the link ISR callbacks; nothing may keep
    // pointing at the destroyed instance.
    link_setTransmitCallback(nullptr, nullptr);
    link_setReceiveCallback(nullptr, nullptr);
    link_setTransiveDoneCallback(nullptr, nullptr);
    g_pokemonPacketLive = false;
}

static void erProto_initPokemonPacketLayer()
{
    erProto_shutdownPokemonPacketLayer();
    new (g_pokemonPacketStorage) PacketLayer();
    g_pokemonPacketLive = true;
    pokemonPacket().enableHandshake();
    pokemonPacket().setTransiveHandler(emptyCommand());
    g_pokeLinkPlayerPending = false;
    g_pokePayloadPending = false;
    g_lpFinished = false;
    blockCommandReset();
}

static void logPacketFrame(erproto::EReaderProxy& proxy,
                           const PacketLayer::TransiveResult& result)
{
    if (!proxy.wireLogEnabled) return;
    proxy.noteWireRound(result.received[0], result.transmitted[0]);
}

static uint8_t g_pokeIdleIdx = 0;

static void pokeSendKeysInit(std::span<const uint16_t>)
{
    g_pokeIdleIdx = 0;
}

static uint16_t pokeSendKeysTransive()
{
    if (g_pokeIdleIdx == 0)
    {
        g_pokeIdleIdx++;
        return erproto::poke::kSendKeys;
    }
    g_pokeIdleIdx++;
    if (g_pokeIdleIdx >= 8) g_pokeIdleIdx = 0;
    return 0;
}

static CommandState pokeSendKeysDone()
{
    return CommandState::done;
}

static TransiveStruct pokeSendKeysIdle()
{
    static TransiveStruct transive
    {
        .init = pokeSendKeysInit,
        .transive = pokeSendKeysTransive,
        .transiveDone = pokeSendKeysDone
    };
    return transive;
}

enum class LpPhase : uint8_t { preDelay, initFrame, postDelay, cont };
static LpPhase  g_lpPhase         = LpPhase::preDelay;
static uint8_t  g_lpPreDelayLeft  = 0;
static uint8_t  g_lpPostDelayLeft = 0;
static uint8_t  g_lpContFramesSent = 0;
static uint8_t  g_lpFrameIdx      = 0;
static uint16_t g_lpTxFrame[8]    = {};

static void lpPrepareZeroFrame()
{
    for (int i = 0; i < 8; i++)
        g_lpTxFrame[i] = 0;
}

static void lpPrepareInitFrame()
{
    g_lpTxFrame[0] = LINKCMD_INIT_BLOCK;
    g_lpTxFrame[1] = static_cast<uint16_t>(kPokeLinkPlayerBytes);
    g_lpTxFrame[2] = static_cast<uint16_t>(LINK_PLAYER_ID);
    for (int i = 3; i < 8; i++)
        g_lpTxFrame[i] = 0;
}

static void lpPrepareContFrame(size_t offset)
{
    const size_t remaining = kPokeLinkPlayerBytes - offset;
    const size_t chunkLen = remaining < 14 ? remaining : 14;

    g_lpTxFrame[0] = LINKCMD_CONT_BLOCK;
    for (int i = 0; i < 7; i++)
        g_lpTxFrame[i + 1] = 0;

    for (size_t i = 0; i + 1 < chunkLen; i += 2)
    {
        g_lpTxFrame[1 + (i / 2)] = static_cast<uint16_t>(g_pokeLinkPlayerBuf[offset + i])
                               | (static_cast<uint16_t>(g_pokeLinkPlayerBuf[offset + i + 1]) << 8);
    }
    if (chunkLen % 2 == 1)
        g_lpTxFrame[1 + (chunkLen / 2)] = g_pokeLinkPlayerBuf[offset + chunkLen - 1];
}

static void pokeLinkPlayerContInit(std::span<const uint16_t>)
{
    g_lpFrameIdx = 0;
}

static uint16_t pokeLinkPlayerContTransive()
{
    return g_lpTxFrame[g_lpFrameIdx++];
}

static CommandState pokeLinkPlayerContDone()
{
    g_lpFrameIdx = 0;

    switch (g_lpPhase)
    {
        case LpPhase::preDelay:
            if (--g_lpPreDelayLeft > 0)
                lpPrepareZeroFrame();
            else
            {
                g_lpPhase = LpPhase::initFrame;
                lpPrepareInitFrame();
            }
            break;

        case LpPhase::initFrame:
            g_lpPhase = LpPhase::postDelay;
            g_lpPostDelayLeft = 2;
            lpPrepareZeroFrame();
            break;

        case LpPhase::postDelay:
            if (--g_lpPostDelayLeft > 0)
                lpPrepareZeroFrame();
            else
            {
                g_lpPhase = LpPhase::cont;
                g_lpContFramesSent = 0;
                lpPrepareContFrame(0);
            }
            break;

        case LpPhase::cont:
            g_lpContFramesSent++;
            if (g_lpContFramesSent >= 5)
            {
                g_lpFinished = true;
                return CommandState::done;
            }
            lpPrepareContFrame(static_cast<size_t>(g_lpContFramesSent) * 14);
            break;
    }

    return CommandState::resume;
}

static TransiveStruct pokeLinkPlayerContCommand()
{
    static TransiveStruct transive
    {
        .init = pokeLinkPlayerContInit,
        .transive = pokeLinkPlayerContTransive,
        .transiveDone = pokeLinkPlayerContDone
    };
    return transive;
}

static void pokemonArmLinkPlayerFull(PacketLayer& layer)
{
    erproto::buildLinkPlayerBlock(g_pokeLinkPlayerBuf, g_proxy.pokeCfg);
    g_lpPhase         = LpPhase::preDelay;
    g_lpPreDelayLeft  = 2;
    g_lpPostDelayLeft = 2;
    g_lpContFramesSent = 0;
    g_lpFinished      = false;
    g_lpFrameIdx      = 0;
    lpPrepareZeroFrame();
    layer.setTransiveHandler(pokeLinkPlayerContCommand());
    g_pokeLinkPlayerPending = true;
}

static void pokemonPollHandshakes(erproto::EReaderProxy& proxy, PacketLayer& layer)
{
    while (auto rx = layer.getReceivedHandshake(K_NO_WAIT))
    {
        if (*rx == LINK_MASTER_HANDSHAKE)
            proxy.emitPhase(erproto::Phase::handshaking, *rx);
    }
}

static void pokemonHandleTransiveFrame(erproto::EReaderProxy& proxy,
                                       PacketLayer& layer,
                                       const PacketLayer::TransiveResult& result)
{
    logPacketFrame(proxy, result);

    const uint16_t cmd = result.received[0];

    if (!proxy.pokeSentLinkPlayer && !g_pokeLinkPlayerPending
        && cmd == erproto::poke::kSendLinkType)
    {
        pokemonArmLinkPlayerFull(layer);
    }
    else if (proxy.pokeSentLinkPlayer && !proxy.pokeSentPayload && !g_pokePayloadPending
             && proxy.pokeWaitingPayload)
    {
        const bool readyAdvance = (cmd == erproto::poke::kReadyAdvance);
        if (proxy.scanArmed && proxy.cardLoaded && proxy.cardSize > 0
            && (readyAdvance || g_lpPostWaitCount >= kLpPostWaitRounds))
        {
            g_lpPostWaitCount = 0;
            proxy.pokeWaitingPayload = false;
            blockCommandSetup(proxy.card,
                              static_cast<uint16_t>(proxy.cardSize),
                              static_cast<uint16_t>(proxy.cardSize));
            layer.setTransiveHandler(blockCommand());
            g_pokePayloadPending = true;
            proxy.emitPhase(erproto::Phase::sending);
        }
        else
        {
            g_lpPostWaitCount++;
            if (layer.idle())
                layer.setTransiveHandler(emptyCommand());
        }
    }
    else if (cmd == LINKCMD_READY_EXIT_STANDBY && !g_pokePayloadPending)
    {
        layer.setTransiveHandler(readyExitStandbyCommand());
    }
    else if (cmd == LINKCMD_READY_CLOSE_LINK && proxy.pokeSentPayload && layer.idle())
    {
        layer.setTransiveHandler(readyCloseLinkCommand());
        proxy.emitComplete(1);
    }
    else if (!g_pokeLinkPlayerPending && !g_pokePayloadPending
             && cmd == LINKCMD_SEND_HELD_KEYS
             && layer.idle())
    {
        layer.setTransiveHandler(pokeSendKeysIdle());
    }

    if (g_pokeLinkPlayerPending && g_lpFinished)
    {
        g_lpFinished = false;
        g_pokeLinkPlayerPending = false;
        proxy.pokeSentLinkPlayer = true;
        g_lpPostWaitCount = 0;
        proxy.pokeWaitingPayload = true;
        layer.setTransiveHandler(emptyCommand());
    }

    if (g_pokePayloadPending && blockCommandTransferComplete()
        && blockCommandBytesSent() >= static_cast<uint16_t>(proxy.cardSize)
        && layer.idle())
    {
        blockCommandConsumeComplete();
        g_pokePayloadPending = false;
        proxy.pokeSentPayload = true;
        proxy.pokeWaitingPayload = false;
        proxy.scanArmed = false;
        layer.setTransiveHandler(emptyCommand());
    }
}

static void pokemonPumpPacketLayer()
{
    if (!g_pokemonPacketLive) return;

    erproto::EReaderProxy& proxy = g_proxy;
    PacketLayer& layer = pokemonPacket();

    pokemonPollHandshakes(proxy, layer);

    while (auto result = layer.awaitTransiveResults(K_NO_WAIT))
        pokemonHandleTransiveFrame(proxy, layer, *result);
}

static struct NextTransmit sma4TransmitCallback(void*)
{
    return { g_proxy.stagedTx, 15370 };
}

static void sma4ReceiveCallback(uint16_t rx, void*)
{
    g_proxy.onRound(rx);
}

static void emitTrampoline(void*, const uint8_t* frame, size_t len)
{
    // Producers span the SMA4 PIO ISR, the transport RX thread, and the section
    // thread; Zephyr ring_buf is only SPSC-safe, so serialize the puts.
    const unsigned int key = irq_lock();
    if (ring_buf_space_get(&g_erOutRing) >= len)
        ring_buf_put(&g_erOutRing, frame, len);
    irq_unlock(key);
}

void erProto_receiveHandler(std::span<const uint8_t> data, void*)
{
    if (!g_sessionActive || data.size() < 5) return;
    if (std::memcmp(data.data(), erproto::ERDR_MAGIC, 4) != 0) return;

    const uint8_t cmd = data[4];
    const uint8_t* payload = data.data() + 5;
    const size_t payloadLen = data.size() - 5;

    const unsigned int key = irq_lock();
    if (cmd == erproto::HOST_START_SCAN)
    {
        // Each scan gets a fresh wrong-pin recovery budget.
        g_sdPathFlipped = false;
        g_wrongPinIntervals = 0;
        if (!g_multiPioActive)
            g_armRequested = true;
    }

    if (cmd == erproto::HOST_START_SCAN
        && !erproto::isSma4Profile(g_proxy.profile))
    {
        g_pokeLinkPlayerPending = false;
        g_pokePayloadPending = false;
        g_lpFinished = false;
        g_lpPhase = LpPhase::preDelay;
        g_lpPreDelayLeft = 0;
        g_lpPostDelayLeft = 0;
        g_lpContFramesSent = 0;
        g_lpPostWaitCount = 0;
        blockCommandReset();
        if (g_pokemonPacketLive)
            pokemonPacket().setTransiveHandler(emptyCommand());
    }

    g_proxy.applyHostFrame(cmd, payload, payloadLen);
    irq_unlock(key);
}

EReaderProtocolSection::EReaderProtocolSection(erproto::Profile profile)
{
    ring_buf_reset(&g_erOutRing);
    g_multiPioActive = false;
    g_armRequested = false;
    g_sdPathFlipped = false;
    g_wrongPinIntervals = 0;

    new (&g_proxy) erproto::EReaderProxy();
    g_proxy.profile = profile;
    g_proxy.pokeCfg = erproto::pokemonConfigFor(profile);
    g_proxy.emit = &emitTrampoline;
    g_proxy.emitCtx = nullptr;
    g_proxy.wireLogEnabled = true;

    if (erproto::isSma4Profile(profile))
    {
        g_proxy.stagedTx = erproto::sma4::ID;
        link_setTransmitCallback(&sma4TransmitCallback, nullptr);
        link_setReceiveCallback(&sma4ReceiveCallback, nullptr);
    }
    else
    {
        g_proxy.emitWireSession();
    }

    g_sessionActive = true;
}

EReaderProtocolSection::~EReaderProtocolSection()
{
    g_sessionActive = false;
    erProto_shutdownPokemonPacketLayer();
    link_setTransmitCallback(nullptr, nullptr);
    link_setReceiveCallback(nullptr, nullptr);
    link_setTransiveDoneCallback(nullptr, nullptr);
}

void EReaderProtocolSection::armLink()
{
    ensurePioLink();
}

void EReaderProtocolSection::process()
{
    uint32_t lastWireTotal = 0;
    int intervalLoops = 0;
    bool scSeenLow = false;
    bool scSeenHigh = false;

    while (!m_cancel)
    {
        if (g_armRequested)
        {
            g_armRequested = false;
            ensurePioLink();
        }

        pumpOutbound();

        if (g_pokemonPacketLive)
            pokemonPumpPacketLayer();

        // Accumulate SC edges across the whole interval — a single sample
        // would miss the GBA's brief clock bursts. Requiring both levels
        // (a real toggle) rejects a stuck-low line.
        if (link_readPartnerPins() & 0x01) scSeenLow = true;
        else scSeenHigh = true;

        if (++intervalLoops >= 2500)
        {
            intervalLoops = 0;
            const bool scToggled = scSeenLow && scSeenHigh;
            scSeenLow = false;
            scSeenHigh = false;

            if (g_proxy.scanArmed && scToggled
                && link_getReceivedWordCount() == 0)
                g_wrongPinIntervals++;

            if (!g_sdPathFlipped && g_wrongPinIntervals >= kWrongPinIntervals)
            {
                g_sdPathFlipped = true;
                link_flipSdPinPath();
                g_multiPioActive = false;
                ensurePioLink();
                g_proxy.emitWire(erproto::WIRE_FLAG_SD_FLIPPED,
                                 link_getDetectedCableType(), 0);
            }

            if (g_proxy.wireLogEnabled)
            {
                if (g_proxy.wireTotal == lastWireTotal)
                    g_proxy.emitWireNoTraffic(scToggled);
                else
                    lastWireTotal = g_proxy.wireTotal;
            }
        }

        if (g_proxy.currentPhase == erproto::Phase::complete
            || g_proxy.currentPhase == erproto::Phase::error)
        {
            pumpOutbound();
            k_sleep(K_MSEC(50));
            pumpOutbound();
            if (g_proxy.currentPhase == erproto::Phase::complete)
                break;
        }

        k_sleep(g_proxy.scanArmed ? K_USEC(200) : K_MSEC(2));
    }
}

void EReaderProtocolSection::pumpOutbound()
{
    for (;;)
    {
        // Move whole ERDR frames only: the web client parses each 64-byte
        // packet independently, so a frame split across chunks would be lost.
        for (;;)
        {
            uint8_t hdr[5];
            if (ring_buf_peek(&g_erOutRing, hdr, sizeof(hdr)) < sizeof(hdr)) break;
            const size_t frameLen = 5 + erproto::erdrPayloadLen(hdr[4]);
            if (m_chunkLen + frameLen > sizeof(m_chunk)) break;
            ring_buf_get(&g_erOutRing, &m_chunk[m_chunkLen], frameLen);
            m_chunkLen += frameLen;
        }

        if (m_chunkLen == sizeof(m_chunk))
        {
            if (!Transport::sendData(std::span(m_chunk, sizeof(m_chunk))))
                return;
            m_chunkLen = 0;
            continue;
        }

        if (m_chunkLen > 0)
        {
            std::memset(&m_chunk[m_chunkLen], 0, sizeof(m_chunk) - m_chunkLen);
            if (Transport::sendData(std::span(m_chunk, sizeof(m_chunk))))
                m_chunkLen = 0;
        }
        return;
    }
}
