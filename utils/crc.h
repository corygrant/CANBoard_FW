#pragma once

#include <stdint.h>
#include <stddef.h>

uint32_t CalculateCRC32Partial(const void* data, size_t length, uint32_t currentCrc);
uint32_t CalculateCRC32(const void* data, size_t length, uint32_t initialValue = 0xFFFFFFFF);
