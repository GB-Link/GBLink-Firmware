#pragma once

#include <cstdint>

namespace bcgproto
{

enum class GateType : uint8_t
{
    battleChipGate     = 0,
    progressChipGate   = 1,
    beastLinkGate      = 2,
    beastLinkGateIntl  = 3,
};

enum class Stage : int8_t
{
    sync    = -1,
    command = 0,
    unk0    = 1,
    unk1    = 2,
    data0   = 3,
    data1   = 4,
    id      = 5,
    unk2    = 6,
    unk3    = 7,
    end     = 8,
};

constexpr uint16_t UNIT_BATTLE_CHIP_GATE    = 0xFFC6;
constexpr uint16_t UNIT_PROGRESS_CHIP_GATE  = 0xFFC7;
constexpr uint16_t UNIT_BEAST_LINK_GATE     = 0xFFC4;
constexpr uint16_t UNIT_BEAST_LINK_GATE_INTL = 0xFF00;

constexpr uint16_t FRAME_START = 0x8FFF;
constexpr uint16_t PAD_WORD    = 0xFFFF;

constexpr uint16_t START_A380 = 0xA380;
constexpr uint16_t START_A390 = 0xA390;
constexpr uint16_t START_A3A0 = 0xA3A0;
constexpr uint16_t START_A3B0 = 0xA3B0;
constexpr uint16_t START_A3C0 = 0xA3C0;
constexpr uint16_t START_A3D0 = 0xA3D0;
constexpr uint16_t START_A6C0 = 0xA6C0;

constexpr uint16_t SEED0_INIT = 0x00FE;
constexpr uint16_t SEED1_INIT = 0xFFFE;
constexpr uint16_t SEED0_MASK = 0x00FF;
constexpr uint16_t SEED1_OR   = 0xFC00;
constexpr uint16_t SEED_STEP  = 3;

constexpr uint16_t unitCode(GateType type)
{
    switch (type)
    {
        case GateType::progressChipGate:  return UNIT_PROGRESS_CHIP_GATE;
        case GateType::beastLinkGate:     return UNIT_BEAST_LINK_GATE;
        case GateType::beastLinkGateIntl: return UNIT_BEAST_LINK_GATE_INTL;
        case GateType::battleChipGate:
        default:                          return UNIT_BATTLE_CHIP_GATE;
    }
}

constexpr GateType gateTypeFromVariant(uint8_t variant)
{
    switch (variant)
    {
        case 1:  return GateType::progressChipGate;
        case 2:  return GateType::beastLinkGate;
        case 3:  return GateType::beastLinkGateIntl;
        case 0:
        default: return GateType::battleChipGate;
    }
}

constexpr bool isStartToken(uint16_t cmd)
{
    switch (cmd)
    {
        case START_A380:
        case START_A390:
        case START_A3A0:
        case START_A3B0:
        case START_A3C0:
        case START_A3D0:
        case START_A6C0:
            return true;
        default:
            return false;
    }
}

constexpr uint8_t BCGS_MAGIC[4] = { 0x42, 0x43, 0x47, 0x53 };

struct BcgProxy
{
    GateType gateType = GateType::battleChipGate;
    uint16_t chipId = 0;
    Stage stage = Stage::sync;
    uint16_t seed0 = SEED0_INIT;
    uint16_t seed1 = SEED1_INIT;
    uint16_t stagedTx = UNIT_BATTLE_CHIP_GATE;
    bool chipFrameArmed = false;

    void reset(GateType type)
    {
        gateType = type;
        chipId = 0;
        stage = Stage::sync;
        seed0 = SEED0_INIT;
        seed1 = SEED1_INIT;
        stagedTx = unitCode(gateType);
        chipFrameArmed = false;
    }

    void applyHostPacket(GateType type, uint16_t id)
    {
        gateType = type;
        chipId = id;
        switch (stage)
        {
            case Stage::sync:
            case Stage::command:
                stagedTx = unitCode(gateType);
                break;
            case Stage::id:
                stagedTx = chipId;
                break;
            default:
                break;
        }
    }

    void enterStandby()
    {
        chipFrameArmed = false;
        stage = Stage::sync;
        stagedTx = unitCode(gateType);
    }

    void onRound(uint16_t cmd)
    {
        if (isStartToken(cmd))
        {
            if (chipFrameArmed && (stage == Stage::sync || stage == Stage::command))
            {
                chipFrameArmed = false;
                stage = Stage::unk0;
                stagedTx = PAD_WORD;
                return;
            }
            enterStandby();
            return;
        }

        if (stage == Stage::sync)
        {
            if (cmd == FRAME_START)
            {
                chipFrameArmed = true;
                stagedTx = unitCode(gateType);
                return;
            }
            if (chipFrameArmed)
            {
                chipFrameArmed = false;
                stage = Stage::unk0;
                stagedTx = PAD_WORD;
                return;
            }
            stagedTx = unitCode(gateType);
            return;
        }

        if (stage == Stage::end)
        {
            stage = Stage::command;
            chipFrameArmed = false;
            stagedTx = unitCode(gateType);
            return;
        }

        const int8_t nextStage = static_cast<int8_t>(stage) + 1;
        stage = static_cast<Stage>(nextStage);
        stagedTx = replyFor(stage);
    }

private:
    uint16_t replyFor(Stage s)
    {
        switch (s)
        {
            case Stage::sync:
            case Stage::command:
                return unitCode(gateType);

            case Stage::unk0:
            case Stage::unk1:
                return PAD_WORD;

            case Stage::data0:
            {
                const uint16_t reply = seed0;
                seed0 = static_cast<uint16_t>((seed0 + SEED_STEP) & SEED0_MASK);
                return reply;
            }

            case Stage::data1:
            {
                const uint16_t reply = seed1;
                seed1 = static_cast<uint16_t>((seed1 - SEED_STEP) | SEED1_OR);
                return reply;
            }

            case Stage::id:
                return chipId;

            case Stage::unk2:
            case Stage::unk3:
            case Stage::end:
                return 0;
        }
        return unitCode(gateType);
    }
};

} // namespace bcgproto
