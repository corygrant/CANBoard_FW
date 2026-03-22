#pragma once

#include <cstdint>
#include <bit>

// 0x0000 - 0x0FFF: System
// 0x1000 - 0x10FF: Outputs
// 0x1100 - 0x11FF: PWM
// 0x1200 - 0x12FF: Inputs
// 0x1300 - 0x13FF: CAN Inputs
// 0x1400 - 0x14FF: Virtual Inputs
// 0x1500 - 0x15FF: Conditions
// 0x1600 - 0x16FF: Counter
// 0x1700 - 0x17FF: Flasher
// 0x1800 - 0x18FF: Starter Disable
// 0x1900 - 0x19FF: Wipers
// 0x2000 - 0x20FF: CAN Outputs
// 0x3000 - 0x30FF: Keypads
// 0x3100 - 0x31FF: Keypad Buttons
// 0x3200 - 0x32FF: Keypad Dials

enum class ParamType : uint8_t
{
    UInt8 = 0,
    Int8 = 1,
    UInt16 = 2,
    Int16 = 3,
    UInt32 = 4,
    Int32 = 5,
    Float = 6,
    Bool = 7,
    Enum = 8
};

struct ParamInfo
{
    uint16_t nIndex;
    uint8_t nSubIndex;
    void* pVal;
    void* pTempVal; // Used for staging new value before applying
    ParamType eType;
    uint32_t nDefaultVal;
    uint32_t nMinVal;
    uint32_t nMaxVal;
};

// Helpers: encode signed values as sign-extended uint32_t (wire format)
constexpr uint32_t I8 (int8_t  v) { return static_cast<uint32_t>(v); }
constexpr uint32_t I16(int16_t v) { return static_cast<uint32_t>(v); }
constexpr uint32_t I32(int32_t v) { return static_cast<uint32_t>(v); }

// Helper: encode a float as its IEEE-754 bit pattern
constexpr uint32_t F(float v) { return std::bit_cast<uint32_t>(v); }

extern const ParamInfo stParams[];
extern const uint16_t NUM_PARAMS;

const ParamInfo* FindParam(uint16_t index, uint8_t subindex);
uint32_t ReadParam(const ParamInfo* param, bool temp = false);
bool WriteParam(const ParamInfo* param, uint32_t value, bool temp = false);
bool IsDefaultValue(const ParamInfo* param);