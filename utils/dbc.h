#pragma once

#include <cstdint>

enum class ByteOrder : uint8_t
{
    LittleEndian = 0, // Intel byte order
    BigEndian = 1     // Motorola byte order
};

class Dbc
{
public:
    // Integer decode/encode without scaling
    static int32_t DecodeInt(const uint8_t *pData, uint8_t nStartBit, uint8_t nBitLength, ByteOrder eByteOrder = ByteOrder::LittleEndian, bool bSigned = false);
    static void EncodeInt(uint8_t *pData, int32_t nRawValue, uint8_t nStartBit, uint8_t nBitLength, ByteOrder eByteOrder = ByteOrder::LittleEndian);

    // Integer decode/encode with DBC-style scaling (value = raw * scale + offset)
    // Returns integer result, but uses float scale/offset to match DBC format
    static int32_t DecodeInt(const uint8_t *pData, uint8_t nStartBit, uint8_t nBitLength, float fScale, float fOffset = 0.0f, ByteOrder eByteOrder = ByteOrder::LittleEndian, bool bSigned = false);
    static void EncodeInt(uint8_t *pData, int32_t nValue, uint8_t nStartBit, uint8_t nBitLength, float fScale, float fOffset = 0.0f, ByteOrder eByteOrder = ByteOrder::LittleEndian);

    // IEEE 754 32-bit float decode/encode (raw float bits, must be byte-aligned)
    static float DecodeFloat(const uint8_t *pData, uint8_t nStartBit);
    static void EncodeFloat(uint8_t *pData, float fValue, uint8_t nStartBit);

    // Float decode/encode with float scaling (value = raw * scale + offset)
    static float DecodeFloat(const uint8_t *pData, uint8_t nStartBit, uint8_t nBitLength, float fScale = 1.0f, float fOffset = 0.0f, ByteOrder eByteOrder = ByteOrder::LittleEndian, bool bSigned = false);
    static void EncodeFloat(uint8_t *pData, float fValue, uint8_t nStartBit, uint8_t nBitLength, float fScale = 1.0f, float fOffset = 0.0f, ByteOrder eByteOrder = ByteOrder::LittleEndian);

private:
    static int32_t DecodeLE(const uint8_t *pData, uint8_t nStartBit, uint8_t nBitLength, bool bSigned);
    static int32_t DecodeBE(const uint8_t *pData, uint8_t nStartBit, uint8_t nBitLength, bool bSigned);
    static void EncodeLE(uint8_t *pData, int32_t nRawValue, uint8_t nStartBit, uint8_t nBitLength);
    static void EncodeBE(uint8_t *pData, int32_t nRawValue, uint8_t nStartBit, uint8_t nBitLength);

    static float ApplyScaling(int32_t nRawValue, float fScale, float fOffset);
    static int32_t ReverseScaling(float fPhysicalValue, float fScale, float fOffset);
};
