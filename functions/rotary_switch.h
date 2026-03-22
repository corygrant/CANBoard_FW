#pragma once

#include "ch.h"
#include "hal.h"

#define INVERT_SW_1 0
#define INVERT_SW_2 0
#define INVERT_SW_3 0
#define INVERT_SW_4 0
#define INVERT_SW_5 0

enum RotarySwInput
{
    RotarySw1 = 0,
    RotarySw2,
    RotarySw3,
    RotarySw4,
    RotarySw5
};

void UpdateSwPos();
uint8_t GetRotarySwPos(RotarySwInput input);