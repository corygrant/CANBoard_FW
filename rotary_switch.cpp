#include "rotary_switch.h"
#include "analog.h"

uint8_t nSwPos[5] = {0};

uint8_t CheckSwPos(float swVolts)
{
    //==================================================
    //MODIFY RANGES BASED ON SWITCH READINGS
    //==================================================
    if (swVolts > 3.91)
        return 7;
    if (swVolts > 3.20)
        return 6;
    if (swVolts > 2.60)
        return 5;
    if (swVolts > 2.13)
        return 4;
    if (swVolts > 1.66)
        return 3;
    if (swVolts > 1.18)
        return 2;
    if (swVolts > 0.71)
        return 1;

    return 0;
}

uint8_t CheckSwPos(float swVolts, bool invert)
{
    uint8_t pos = CheckSwPos(swVolts);

    if (invert)
    {
        // subtract from highest possible switch position, in case your switch has more
        return CheckSwPos(5) - pos;
    }
    else
    {
        return pos;
    }
}

void UpdateSwPos()
{
    nSwPos[0] = CheckSwPos(GetAdcVolts(AnIn1), INVERT_SW_1);
    nSwPos[1] = CheckSwPos(GetAdcVolts(AnIn2), INVERT_SW_2);
    nSwPos[2] = CheckSwPos(GetAdcVolts(AnIn3), INVERT_SW_3);
    nSwPos[3] = CheckSwPos(GetAdcVolts(AnIn4), INVERT_SW_4);
    nSwPos[4] = CheckSwPos(GetAdcVolts(AnIn5), INVERT_SW_5);
}

uint8_t GetRotarySwPos(RotarySwInput input)
{
    switch(input)
    {
        case RotarySw1:
            return nSwPos[0];
        case RotarySw2:
            return nSwPos[1];
        case RotarySw3:
            return nSwPos[2];
        case RotarySw4:
            return nSwPos[3];
        case RotarySw5:
            return nSwPos[4];
        default:
            return 0; // Invalid input
    }
}