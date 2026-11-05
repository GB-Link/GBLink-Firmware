#include "eReaderProtocolSection.hpp"

#include <cstring>
#include <new>

#include <zephyr/sys/ring_buffer.h>
#include "../link_defines.h"

#include "../layers/packet/packetCommands/commands.hpp"
#include "../layers/packet/packetLayer.hpp"
#include "../layers/transport.hpp"
#include "../layers/link/ereader/ereader.hpp"
#include "../layers/link/gbaLinkLayer.hpp"

RING_BUF_DECLARE(g_erOutRing, 2048);

static erproto::EReaderProxy g_proxy;
static volatile bool g_sessionActive = false;
static bool g_multiPioActive = false;

// PIO (re)configuration sleeps, so the transport RX handler only flags it and
// the section thread performs it from process().
static volatile bool g_armRequested = false;

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

    // Recycle the packet layer BEFORE touching the PIO: ~PacketLayer disables
    // the link, so running it after would kill the mode configured just below.
    erProto_shutdownPokemonPacketLayer();

    if (erproto::isSma4Profile(g_proxy.profile))
        ereader_configureEreaderSlave();
    else
    {
        ereader_configurePartnerPresence();
        k_sleep(K_MSEC(200));
        multiMode_selectMode(SLAVE);
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
    pokemonPacket().sendCommand(emptyCommand());
    g_pokeLinkPlayerPending = false;
    g_pokePayloadPending = false;
    g_lpFinished = false;
}

static void logPacketFrame(erproto::EReaderProxy& proxy,
                           const PacketLayer::TransiveResult& result)
{
    if (!proxy.wireLogEnabled) return;
    TxValues txValues = result.transmitted[0];
    proxy.noteWireRound(result.received[0], txValues[0]);
}

struct PokeSendContext : TransmitContext
{
    uint8_t pokeIdleIdx = 0;
};

static TxValues pokeSendKeysTransive(TransmitContext* ctx)
{
    auto* pokeCtx = static_cast<PokeSendContext*>(ctx);

    const uint16_t value = (pokeCtx->pokeIdleIdx == 0) ? erproto::poke::kSendKeys : 0x00;

    pokeCtx->pokeIdleIdx++;
    if (pokeCtx->pokeIdleIdx >= 8) pokeCtx->pokeIdleIdx = 0;

    return TxValues(value);
}


static CommandState pokeSendKeysDone(TransmitContext* ctx)
{
    (void)ctx;
    return CommandState::done;
}

TransmitBehaviour pokeSendKeysIdle()
{
    std::unique_ptr<PokeSendContext> c = std::make_unique<PokeSendContext>();
    c->pokeIdleIdx = 0;

    TransmitBehaviour transive
    {
        .context = std::move(c),
        .transmitCallback = pokeSendKeysTransive,
        .transmitDoneCallback = pokeSendKeysDone
    };
    return transive;
}

enum class LpPhase : uint8_t { preDelay, initFrame, postDelay, cont };


struct PokeLinkPlayerContext : TransmitContext
{
    uint8_t  lpFrameIdx = 0;

    LpPhase  lpPhase = LpPhase::preDelay;
    uint8_t  lpPreDelayLeft  = 0;
    uint8_t  lpPostDelayLeft = 0;
    uint8_t  lpContFramesSent = 0;
    uint16_t lpTxFrame[8]    = {};
};

static void lpPrepareZeroFrame(PokeLinkPlayerContext* ctx)
{
    for (int i = 0; i < 8; i++)
    {
        ctx->lpTxFrame[i] = 0;
    }
}

static void lpPrepareInitFrame(PokeLinkPlayerContext* ctx)
{
    ctx->lpTxFrame[0] = LINKCMD_INIT_BLOCK;
    ctx->lpTxFrame[1] = static_cast<uint16_t>(kPokeLinkPlayerBytes);
    ctx->lpTxFrame[2] = static_cast<uint16_t>(LINK_PLAYER_ID);
    for (int i = 3; i < 8; i++)
    {
        ctx->lpTxFrame[i] = 0;
    }
}

static void lpPrepareContFrame(size_t offset, PokeLinkPlayerContext* ctx)
{
    const size_t remaining = kPokeLinkPlayerBytes - offset;
    const size_t chunkLen = remaining < 14 ? remaining : 14;

    ctx->lpTxFrame[0] = LINKCMD_CONT_BLOCK;
    for (int i = 0; i < 7; i++)
        ctx->lpTxFrame[i + 1] = 0;

    for (size_t i = 0; i + 1 < chunkLen; i += 2)
    {
        ctx->lpTxFrame[1 + (i / 2)] = static_cast<uint16_t>(g_pokeLinkPlayerBuf[offset + i])
                               | (static_cast<uint16_t>(g_pokeLinkPlayerBuf[offset + i + 1]) << 8);
    }
    if (chunkLen % 2 == 1)
        ctx->lpTxFrame[1 + (chunkLen / 2)] = g_pokeLinkPlayerBuf[offset + chunkLen - 1];
}

static TxValues pokeLinkPlayerContTransive(TransmitContext* ctx)
{
    auto* pokeCtx = static_cast<PokeLinkPlayerContext*>(ctx);
    TxValues txValues;
    txValues.push(pokeCtx->lpTxFrame[pokeCtx->lpFrameIdx++]);
    return txValues;
}

static CommandState pokeLinkPlayerContDone(TransmitContext* ctx)
{
    auto* pokeCtx = static_cast<PokeLinkPlayerContext*>(ctx);
    pokeCtx->lpFrameIdx = 0;

    switch (pokeCtx->lpPhase)
    {
        case LpPhase::preDelay:
            if (--pokeCtx->lpPreDelayLeft > 0)
                lpPrepareZeroFrame(pokeCtx);
            else
            {
                pokeCtx->lpPhase = LpPhase::initFrame;
                lpPrepareInitFrame(pokeCtx);
            }
            break;

        case LpPhase::initFrame:
            pokeCtx->lpPhase = LpPhase::postDelay;
            pokeCtx->lpPostDelayLeft = 2;
            lpPrepareZeroFrame(pokeCtx);
            break;

        case LpPhase::postDelay:
            if (--pokeCtx->lpPostDelayLeft > 0)
                lpPrepareZeroFrame(pokeCtx);
            else
            {
                pokeCtx->lpPhase = LpPhase::cont;
                pokeCtx->lpContFramesSent = 0;
                lpPrepareContFrame(0, pokeCtx);
            }
            break;

        case LpPhase::cont:
            pokeCtx->lpContFramesSent++;
            if (pokeCtx->lpContFramesSent >= 5)
            {
                g_lpFinished = true;
                return CommandState::done;
            }
            lpPrepareContFrame(static_cast<size_t>(pokeCtx->lpContFramesSent) * 14, pokeCtx);
            break;
    }

    return CommandState::resume;
}

TransmitBehaviour pokeLinkPlayerContCommand()
{
    std::unique_ptr<PokeLinkPlayerContext> c = std::make_unique<PokeLinkPlayerContext>();
    c->lpPhase         = LpPhase::preDelay;
    c->lpPreDelayLeft  = 2;
    c->lpPostDelayLeft = 2;
    c->lpContFramesSent = 0;
    c->lpFrameIdx      = 0;
    lpPrepareZeroFrame(c.get());

    TransmitBehaviour transive
    {
        .context = std::move(c),
        .transmitCallback = pokeLinkPlayerContTransive,
        .transmitDoneCallback = pokeLinkPlayerContDone
    };
    return transive;
}

static void pokemonArmLinkPlayerFull(PacketLayer& layer)
{
    erproto::buildLinkPlayerBlock(g_pokeLinkPlayerBuf, g_proxy.pokeCfg);

    
    layer.sendCommand(pokeLinkPlayerContCommand());
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
            layer.sendCommand(blockCommand(proxy.card,
                              static_cast<uint16_t>(proxy.cardSize),
                              static_cast<uint16_t>(proxy.cardSize)));
            g_pokePayloadPending = true;
            proxy.emitPhase(erproto::Phase::sending);
        }
        else
        {
            g_lpPostWaitCount++;
            if (layer.idle())
                layer.sendCommand(emptyCommand());
        }
    }
    else if (cmd == LINKCMD_READY_EXIT_STANDBY && !g_pokePayloadPending)
    {
        layer.sendCommand(readyExitStandbyCommand());
    }
    else if (cmd == LINKCMD_READY_CLOSE_LINK && proxy.pokeSentPayload && layer.idle())
    {
        layer.sendCommand(readyCloseLinkCommand());
        proxy.emitComplete(1);
    }
    else if (!g_pokeLinkPlayerPending && !g_pokePayloadPending
             && cmd == LINKCMD_SEND_HELD_KEYS
             && layer.idle())
    {
        layer.sendCommand(pokeSendKeysIdle());
    }

    if (g_pokeLinkPlayerPending && g_lpFinished)
    {
        g_lpFinished = false;
        g_pokeLinkPlayerPending = false;
        proxy.pokeSentLinkPlayer = true;
        g_lpPostWaitCount = 0;
        proxy.pokeWaitingPayload = true;
        layer.sendCommand(emptyCommand());
    }

    if (g_pokePayloadPending && layer.idle())
    {
        g_pokePayloadPending = false;
        proxy.pokeSentPayload = true;
        proxy.pokeWaitingPayload = false;
        proxy.scanArmed = false;
        layer.sendCommand(emptyCommand());
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
    TxValues txValues;
    txValues.push(g_proxy.stagedTx);
    return {1, txValues, 15370 };
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
        if (!g_multiPioActive)
            g_armRequested = true;
    }

    if (cmd == erproto::HOST_START_SCAN
        && !erproto::isSma4Profile(g_proxy.profile))
    {
        g_pokeLinkPlayerPending = false;
        g_pokePayloadPending = false;
        g_lpFinished = false;
        g_lpPostWaitCount = 0;
        
        if (g_pokemonPacketLive)
            pokemonPacket().sendCommand(emptyCommand());
    }

    g_proxy.applyHostFrame(cmd, payload, payloadLen);
    irq_unlock(key);
}

EReaderProtocolSection::EReaderProtocolSection(erproto::Profile profile)
{
    ring_buf_reset(&g_erOutRing);
    g_multiPioActive = false;
    g_armRequested = false;

    new (&g_proxy) erproto::EReaderProxy();
    g_proxy.profile = profile;
    g_proxy.pokeCfg = erproto::pokemonConfigFor(profile);
    g_proxy.emit = &emitTrampoline;
    g_proxy.emitCtx = nullptr;
    g_proxy.wireLogEnabled = true;

    if (erproto::isSma4Profile(profile))
    {
        g_proxy.stagedTx = g_proxy.sma4HandshakeId();
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
