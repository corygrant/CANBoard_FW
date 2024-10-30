#pragma once

#include "ch.h"
#include "hal.h"

enum DigIn{
    DigIn1 = 0,
    DigIn2,
    DigIn3,
    DigIn4,
    DigIn5,
    DigIn6,
    DigIn7,
    DigIn8,
    IdSel1,
    IdSel2
};

enum DigOut{
    DigOut1 = 0,
    DigOut2,
    DigOut3,
    DigOut4
};

bool GetDigIn(DigIn input);
void SetDigOut(DigOut output, bool val);
bool GetDigOut(DigOut output);