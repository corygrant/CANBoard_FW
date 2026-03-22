#include "crc.h"

uint32_t CalculateCRC32Partial(const void* data, size_t length, uint32_t currentCrc) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint32_t crc = currentCrc;
    
    // Standard CRC-32 (Ethernet, ZIP, etc.) calculation
    for (size_t i = 0; i < length; i++) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 * (crc & 1));
        }
    }
    
    return crc; //No XOR here // Final XOR with 0xFFFFFFFF
}

uint32_t CalculateCRC32(const void* data, size_t length, uint32_t initialValue) {
    return ~CalculateCRC32Partial(data, length, initialValue);
}