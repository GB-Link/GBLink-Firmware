#pragma once

#include <cstdint>
#include <cstring>
#include <cstddef>

#include "../link_defines.h"

extern "C"
{
    #include "../layers/linkLayer.h"
}

namespace erproto
{

constexpr uint16_t CMD_NONE = 0x7FFF;

constexpr uint8_t ERDR_MAGIC[4] = { 0x45, 0x52, 0x44, 0x52 };

constexpr uint8_t HOST_LOAD_CARD    = 0x01;
constexpr uint8_t HOST_START_SCAN   = 0x02;
constexpr uint8_t HOST_CANCEL_SCAN  = 0x03;
constexpr uint8_t HOST_SET_WIRE_LOG = 0x04;

constexpr uint8_t EVT_PHASE    = 0x10;
constexpr uint8_t EVT_COMPLETE = 0x11;
constexpr uint8_t EVT_ERROR    = 0x12;
constexpr uint8_t EVT_WIRE     = 0x13;

constexpr uint8_t WIRE_FLAG_SCAN_ARMED = 0x01;
constexpr uint8_t WIRE_FLAG_IDLE_RX    = 0x02;
constexpr uint8_t WIRE_FLAG_HANDSHAKE  = 0x04;
constexpr uint8_t WIRE_FLAG_SUMMARY    = 0x08;
constexpr uint8_t WIRE_FLAG_SD_FLIPPED = 0x10;
constexpr uint8_t WIRE_FLAG_PIO_ARMED  = 0x20;
constexpr uint8_t WIRE_FLAG_NO_TRAFFIC = 0x40;
constexpr uint8_t WIRE_FLAG_SESSION    = 0x80;

constexpr size_t   maxCardBytes  = 8192;
constexpr size_t   maxFrameBytes = 16;
constexpr uint16_t sma4CardBytes = 1998;
constexpr uint16_t sma4CardWords = sma4CardBytes / 2;

enum class Profile : uint8_t
{
    sma4           = 1,
    pokemonRuby    = 2,
    sma4JPN        = 3,
    pokemonRubyJPN = 4,
    sma4EUR        = 5,
    sma4EURFra     = 6,
    sma4EURGer     = 7,
    sma4EUREsp     = 8,
    sma4EURIta     = 9,
};

inline bool isSma4Profile(Profile p)
{
    return p == Profile::sma4
        || p == Profile::sma4JPN
        || p == Profile::sma4EUR
        || p == Profile::sma4EURFra
        || p == Profile::sma4EURGer
        || p == Profile::sma4EUREsp
        || p == Profile::sma4EURIta;
}

enum class Phase : uint8_t
{
    idle        = 0,
    handshaking = 1,
    lakitu      = 2,
    sending     = 3,
    complete    = 4,
    error       = 5,
};

enum class ErrorCode : uint8_t
{
    none            = 0,
    cardTooLarge    = 1,
    cardNotLoaded   = 2,
    gameRejected    = 3,
    timeout         = 4,
    cableSilent     = 5,
    cancel          = 6,
    unexpected      = 7,
};

namespace sma4
{
constexpr uint16_t ID           = 0xFBFB;
constexpr uint16_t ID_EUR_ENG   = 0xEAEA;
constexpr uint16_t ID_EUR_FRA   = 0xE9E9;
constexpr uint16_t ID_EUR_GER   = 0xE8E8;
constexpr uint16_t ID_EUR_ESP   = 0xE7E7;
constexpr uint16_t ID_EUR_ITA   = 0xE6E6;
constexpr uint16_t GAME_ID_AX   = 0x5841;
constexpr uint16_t GAME_ID_4E   = 0x4534;
constexpr uint16_t GAME_ID_4J   = 0x4A34;
constexpr uint16_t REQ_LEVEL    = 0xEEEE;
constexpr uint16_t REQ_POWERUP  = 0xEDED;
constexpr uint16_t REQ_DEMO     = 0xECEC;
constexpr uint16_t LAKITU_GONE  = 0xF3F3;
constexpr uint16_t LAKITU_MOVE  = 0xF2F2;
constexpr uint16_t LAKITU_HERE  = 0xF1F1;
constexpr uint16_t WAIT_LEVEL   = 0xFAFA;
constexpr uint16_t WAIT_POWERUP = 0xF0F0;
constexpr uint16_t WAIT_DEMO    = 0xEFEF;
constexpr uint16_t SCANNED      = 0xF9F9;
constexpr uint16_t RDY_DATA     = 0xFEFE;
constexpr uint16_t SEND_DATA    = 0xFDFD;
constexpr uint16_t SENT         = 0xFCFC;
constexpr uint16_t RECV_GOOD    = 0xF5F5;
constexpr uint16_t RECV_BAD     = 0xF4F4;
constexpr uint16_t LAKITU_FRAMES = 10;
}

// Aliases of the shared gen3 link words so this header stays the single
// e-reader vocabulary while link_defines.h stays the single value source.
namespace poke
{
constexpr uint16_t kMasterHandshake = LINK_MASTER_HANDSHAKE;
constexpr uint16_t kSlaveHandshake  = LINK_SLAVE_HANDSHAKE;
constexpr uint16_t kSendLinkType    = LINKCMD_SEND_LINK_TYPE;
constexpr uint16_t kReadyAdvance    = 0xAAAA; // e-reader block advance; the LINKCMD_ name for 0xAAAA is blender-specific
constexpr uint16_t kSendKeys        = LINKCMD_BLENDER_SEND_KEYS;

constexpr uint32_t kLinkTypeRuby = LINKTYPE_MYSTERY_EVENT;
}

inline bool isIdleWord(uint16_t w)
{
    return w == 0 || w == CMD_NONE || w == 0xFFFF;
}

inline size_t erdrPayloadLen(uint8_t evt)
{
    switch (evt)
    {
        case EVT_PHASE:    return 3;
        case EVT_COMPLETE: return 1;
        case EVT_ERROR:    return 3;
        case EVT_WIRE:     return 9;
        default:           return 0;
    }
}

inline size_t erdrSerialize(uint8_t* out, uint8_t evt, const uint8_t* payload, size_t payloadLen)
{
    out[0] = ERDR_MAGIC[0];
    out[1] = ERDR_MAGIC[1];
    out[2] = ERDR_MAGIC[2];
    out[3] = ERDR_MAGIC[3];
    out[4] = evt;
    if (payloadLen > 0 && payload != nullptr)
        std::memcpy(&out[5], payload, payloadLen);
    return 5 + payloadLen;
}

struct PokemonConfig
{
    uint32_t linkType;
    uint8_t  gameVersion;
    uint8_t  language = 2;
};

inline PokemonConfig pokemonConfigFor(Profile p)
{
    const uint8_t language = (p == Profile::pokemonRubyJPN) ? 1 : 2;
    return { poke::kLinkTypeRuby, 0, language };
}

inline void buildLinkPlayerBlock(uint8_t* block, const PokemonConfig& cfg)
{
    static const char magic[] = "GameFreak inc.";
    static const char name[] = "GB-Link";
    std::memset(block, 0, 60);
    std::memcpy(block, magic, 14);
    std::memcpy(block + 44, magic, 14);

    const uint16_t versionWord = static_cast<uint16_t>(0x4000 + cfg.gameVersion);
    block[16] = static_cast<uint8_t>(versionWord & 0xFF);
    block[17] = static_cast<uint8_t>(versionWord >> 8);
    block[18] = 0;
    block[19] = 0;
    block[20] = 0x9E;
    block[21] = 0x52;
    block[22] = 0x00;
    block[23] = 0x00;
    std::memcpy(block + 24, name, 7);
    block[35] = 0;
    block[36] = static_cast<uint8_t>(cfg.linkType & 0xFF);
    block[37] = static_cast<uint8_t>((cfg.linkType >> 8) & 0xFF);
    block[38] = static_cast<uint8_t>((cfg.linkType >> 16) & 0xFF);
    block[39] = static_cast<uint8_t>((cfg.linkType >> 24) & 0xFF);
    block[40] = 1;
    block[41] = 0;
    block[42] = cfg.language;
    block[43] = 0;
}

enum class Sma4State : uint8_t
{
    idle,
    handshakeId,
    handshakeAx,
    handshake4e,
    waitRequest,
    lakituGone,
    lakituMove,
    lakituHere,
    scanReady,
    sendDataStart,
    sendPayload,
    sendChecksumLo,
    sendChecksumHi,
    sendSent,
    waitAck,
    done,
};

struct EReaderProxy
{
    using EmitFn = void(*)(void* ctx, const uint8_t* frame, size_t len);

    Profile profile = Profile::sma4;
    PokemonConfig pokeCfg = pokemonConfigFor(Profile::pokemonRuby);
    EmitFn emit = nullptr;
    void* emitCtx = nullptr;

    uint8_t card[maxCardBytes] = {};
    size_t  cardSize = 0;
    bool    cardLoaded = false;
    bool    scanArmed = false;
    bool    cancelled = false;
    bool    pokeWaitingPayload = false;
    bool    pokeSentLinkPlayer = false;
    bool    pokeSentPayload = false;

    volatile uint16_t stagedTx = CMD_NONE;
    Phase currentPhase = Phase::idle;

    Sma4State sma4 = Sma4State::idle;
    uint16_t cardRequest = 0;
    uint16_t lakituCounter = 0;
    uint16_t payloadIndex = 0;
    uint32_t checksum = 0;

    bool     wireLogEnabled = false;
    uint32_t wireTotal = 0;
    uint16_t wireIdleRounds = 0;

    void emitWire(uint8_t flags, uint16_t rx, uint16_t tx)
    {
        if (!wireLogEnabled || !emit) return;
        uint8_t payload[9];
        payload[0] = flags;
        payload[1] = static_cast<uint8_t>(rx & 0xFF);
        payload[2] = static_cast<uint8_t>(rx >> 8);
        payload[3] = static_cast<uint8_t>(tx & 0xFF);
        payload[4] = static_cast<uint8_t>(tx >> 8);
        payload[5] = static_cast<uint8_t>(wireTotal & 0xFF);
        payload[6] = static_cast<uint8_t>((wireTotal >> 8) & 0xFF);
        payload[7] = static_cast<uint8_t>((wireTotal >> 16) & 0xFF);
        payload[8] = static_cast<uint8_t>((wireTotal >> 24) & 0xFF);
        uint8_t frame[maxFrameBytes];
        const size_t len = erdrSerialize(frame, EVT_WIRE, payload, 9);
        emit(emitCtx, frame, len);
    }

    void emitWireSession()
    {
        emitWire(WIRE_FLAG_SESSION, 0, 0);
    }

    // scToggled: SC clock activity observed since the previous beacon. SC is
    // the same pin for both cable types, so "SC toggling but zero words
    // received" is proof of listening on the wrong SD pin — the host's
    // cable-flip fallback keys on it.
    void emitWireNoTraffic(bool scToggled)
    {
        const uint8_t pins =
            (link_readPartnerPins() & static_cast<uint8_t>(~0x01))
            | (scToggled ? 0x01 : 0x00);
        emitWire(WIRE_FLAG_NO_TRAFFIC, link_getDetectedCableType(),
                 static_cast<uint16_t>(pins));
    }

    void noteWireRound(uint16_t rx, uint16_t tx)
    {
        if (!wireLogEnabled) return;

        wireTotal++;

        uint8_t flags = scanArmed ? WIRE_FLAG_SCAN_ARMED : 0;
        if (isIdleWord(rx))
            flags |= WIRE_FLAG_IDLE_RX;
        if (!isSma4Profile(profile) && isMasterHandshakeWord(rx))
            flags |= WIRE_FLAG_HANDSHAKE;

        bool emitNow = (wireTotal == 1)
                    || !isIdleWord(rx)
                    || (flags & WIRE_FLAG_HANDSHAKE);

        if (isIdleWord(rx) && !(flags & WIRE_FLAG_HANDSHAKE))
        {
            if (++wireIdleRounds >= 200)
            {
                emitNow = true;
                flags |= WIRE_FLAG_SUMMARY;
                wireIdleRounds = 0;
            }
        }
        else
        {
            wireIdleRounds = 0;
        }

        if (emitNow)
            emitWire(flags, rx, tx);
    }

    void emitPhase(Phase phase, uint16_t detail = 0)
    {
        currentPhase = phase;
        if (!emit) return;
        uint8_t payload[3];
        payload[0] = static_cast<uint8_t>(phase);
        payload[1] = static_cast<uint8_t>(detail & 0xFF);
        payload[2] = static_cast<uint8_t>(detail >> 8);
        uint8_t frame[maxFrameBytes];
        const size_t len = erdrSerialize(frame, EVT_PHASE, payload, 3);
        emit(emitCtx, frame, len);
    }

    void emitComplete(uint8_t result = 1)
    {
        currentPhase = Phase::complete;
        if (!emit) return;
        uint8_t frame[maxFrameBytes];
        const size_t len = erdrSerialize(frame, EVT_COMPLETE, &result, 1);
        emit(emitCtx, frame, len);
    }

    void emitError(ErrorCode code, uint16_t detail = 0)
    {
        currentPhase = Phase::error;
        if (!emit) return;
        uint8_t payload[3];
        payload[0] = static_cast<uint8_t>(code);
        payload[1] = static_cast<uint8_t>(detail & 0xFF);
        payload[2] = static_cast<uint8_t>(detail >> 8);
        uint8_t frame[maxFrameBytes];
        const size_t len = erdrSerialize(frame, EVT_ERROR, payload, 3);
        emit(emitCtx, frame, len);
    }

    bool loadCardChunk(uint32_t offset, const uint8_t* data, size_t len)
    {
        // A fresh upload always restarts at offset 0; drop the previous card's
        // high-water size so a smaller card can't trail stale bytes.
        if (offset == 0)
            cardSize = 0;
        if (offset > maxCardBytes || len > maxCardBytes - offset) return false;
        std::memcpy(card + offset, data, len);
        if (offset + len > cardSize) cardSize = offset + len;
        cardLoaded = true;
        return true;
    }

    uint16_t sma4HandshakeId() const
    {
        switch (profile)
        {
            case Profile::sma4EUR:    return sma4::ID_EUR_ENG;
            case Profile::sma4EURFra: return sma4::ID_EUR_FRA;
            case Profile::sma4EURGer: return sma4::ID_EUR_GER;
            case Profile::sma4EUREsp: return sma4::ID_EUR_ESP;
            case Profile::sma4EURIta: return sma4::ID_EUR_ITA;
            default:                  return sma4::ID;
        }
    }

    uint16_t sma4GameId4() const
    {
        return (profile == Profile::sma4JPN) ? sma4::GAME_ID_4J : sma4::GAME_ID_4E;
    }

    void startScan()
    {
        cancelled = false;
        scanArmed = true;

        if (isSma4Profile(profile))
        {
            sma4 = Sma4State::handshakeId;
            stagedTx = sma4HandshakeId();
            emitPhase(Phase::handshaking);
            return;
        }

        if (!cardLoaded || cardSize == 0)
        {
            scanArmed = false;
            emitError(ErrorCode::cardNotLoaded);
            return;
        }
        pokeSentLinkPlayer = false;
        pokeSentPayload = false;
        emitPhase(Phase::handshaking);
    }

    void cancelScan()
    {
        cancelled = true;
        scanArmed = false;
        pokeWaitingPayload = false;
        if (isSma4Profile(profile))
        {
            sma4 = Sma4State::idle;
            stagedTx = sma4HandshakeId();
        }
        emitError(ErrorCode::cancel);
    }

    static uint16_t waitWordForRequest(uint16_t req)
    {
        switch (req)
        {
            case sma4::REQ_LEVEL:   return sma4::WAIT_LEVEL;
            case sma4::REQ_POWERUP: return sma4::WAIT_POWERUP;
            case sma4::REQ_DEMO:    return sma4::WAIT_DEMO;
            default:                return sma4::WAIT_LEVEL;
        }
    }

    static bool isMasterHandshakeWord(uint16_t w)
    {
        return (w & ~static_cast<uint16_t>(0x3)) == poke::kMasterHandshake;
    }

    // SMA4 only: the raw MULTI receive ISR feeds this; the Pokemon profiles run
    // through PacketLayer in eReaderProtocolSection.cpp instead.
    void onRound(uint16_t rx)
    {
        // stagedTx at entry is what pioIsr_tx latched for THIS round; capture it
        // before the FSM stages the next reply so the log pairs rx/tx correctly.
        const uint16_t txThisRound = stagedTx;

        if (cancelled) { stagedTx = CMD_NONE; return; }

        onRoundSma4(rx);
        noteWireRound(rx, txThisRound);
    }

    void onRoundSma4(uint16_t rx)
    {
        if (!scanArmed) { stagedTx = CMD_NONE; return; }

        switch (sma4)
        {
            case Sma4State::handshakeId:
                stagedTx = sma4HandshakeId();
                if (rx == sma4HandshakeId())
                {
                    sma4 = Sma4State::handshakeAx;
                    stagedTx = sma4::GAME_ID_AX;
                }
                break;

            case Sma4State::handshakeAx:
                stagedTx = sma4::GAME_ID_AX;
                if (rx == sma4::GAME_ID_AX)
                {
                    sma4 = Sma4State::handshake4e;
                    stagedTx = sma4GameId4();
                }
                break;

            case Sma4State::handshake4e:
                stagedTx = sma4GameId4();
                if (rx == sma4GameId4()) sma4 = Sma4State::waitRequest;
                break;

            case Sma4State::waitRequest:
                stagedTx = sma4GameId4();
                if (rx != sma4GameId4())
                {
                    cardRequest = rx;
                    sma4 = Sma4State::lakituGone;
                    stagedTx = sma4::LAKITU_GONE;
                    emitPhase(Phase::lakitu, rx);
                }
                break;

            case Sma4State::lakituGone:
                stagedTx = sma4::LAKITU_GONE;
                if (rx == sma4::LAKITU_MOVE)
                {
                    sma4 = Sma4State::lakituMove;
                    lakituCounter = 0;
                    stagedTx = sma4::LAKITU_MOVE;
                }
                break;

            case Sma4State::lakituMove:
                stagedTx = sma4::LAKITU_MOVE;
                if (++lakituCounter >= sma4::LAKITU_FRAMES)
                {
                    sma4 = Sma4State::lakituHere;
                    stagedTx = sma4::LAKITU_HERE;
                }
                break;

            case Sma4State::lakituHere:
                stagedTx = sma4::LAKITU_HERE;
                if (rx == waitWordForRequest(cardRequest))
                    sma4 = Sma4State::scanReady;
                break;

            case Sma4State::scanReady:
                if (rx == sma4::RDY_DATA)
                {
                    stagedTx = sma4::SEND_DATA;
                    if (cardLoaded && cardSize >= sma4CardBytes)
                    {
                        payloadIndex = 0;
                        checksum = 0;
                        sma4 = Sma4State::sendPayload;
                    }
                    else
                    {
                        sma4 = Sma4State::sendDataStart;
                    }
                    emitPhase(Phase::sending);
                }
                else
                {
                    stagedTx = sma4::SCANNED;
                }
                break;

            case Sma4State::sendDataStart:
                stagedTx = sma4::SEND_DATA;
                if (cardLoaded && cardSize >= sma4CardBytes)
                {
                    payloadIndex = 0;
                    checksum = 0;
                    sma4 = Sma4State::sendPayload;
                }
                break;

            case Sma4State::sendPayload:
            {
                const uint16_t block = static_cast<uint16_t>(card[payloadIndex * 2])
                                     | (static_cast<uint16_t>(card[payloadIndex * 2 + 1]) << 8);
                stagedTx = block;
                checksum = (checksum + block) & 0xFFFFFFFFu;
                if (++payloadIndex >= sma4CardWords)
                    sma4 = Sma4State::sendChecksumLo;
                break;
            }

            case Sma4State::sendChecksumLo:
                stagedTx = static_cast<uint16_t>(checksum & 0xFFFF);
                sma4 = Sma4State::sendChecksumHi;
                break;

            case Sma4State::sendChecksumHi:
                stagedTx = static_cast<uint16_t>((checksum >> 16) & 0xFFFF);
                sma4 = Sma4State::sendSent;
                break;

            case Sma4State::sendSent:
                stagedTx = sma4::SENT;
                sma4 = Sma4State::waitAck;
                break;

            case Sma4State::waitAck:
                stagedTx = sma4::SENT;
                if (rx == sma4::RECV_GOOD)
                {
                    sma4 = Sma4State::done;
                    scanArmed = false;
                    emitComplete(1);
                }
                else if (rx == sma4::RECV_BAD)
                {
                    sma4 = Sma4State::done;
                    scanArmed = false;
                    emitError(ErrorCode::gameRejected);
                }
                break;

            default:
                stagedTx = CMD_NONE;
                break;
        }
    }

    void applyHostFrame(uint8_t cmd, const uint8_t* payload, size_t len)
    {
        switch (cmd)
        {
            case HOST_LOAD_CARD:
                if (len < 6) return;
                {
                    const uint32_t offset = static_cast<uint32_t>(payload[0])
                                          | (static_cast<uint32_t>(payload[1]) << 8)
                                          | (static_cast<uint32_t>(payload[2]) << 16)
                                          | (static_cast<uint32_t>(payload[3]) << 24);
                    const uint16_t chunkLen = static_cast<uint16_t>(payload[4])
                                            | (static_cast<uint16_t>(payload[5]) << 8);
                    if (6u + chunkLen > len) return;
                    if (!loadCardChunk(offset, payload + 6, chunkLen))
                        emitError(ErrorCode::cardTooLarge);
                }
                break;
            case HOST_START_SCAN:
                startScan();
                break;
            case HOST_CANCEL_SCAN:
                cancelScan();
                break;
            case HOST_SET_WIRE_LOG:
                if (len >= 1)
                    wireLogEnabled = payload[0] != 0;
                break;
            default:
                break;
        }
    }
};

}
