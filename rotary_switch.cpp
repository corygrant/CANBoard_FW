#include "rotary_switch.h"
#include "analog.h"

uint8_t nSwPos[5] = {0};

uint8_t CheckSwPos(uint16_t swReading, bool invert)
{
    //==================================================
    //MODIFY RANGES BASED ON SWITCH READINGS
    //==================================================
    if(invert){
      if(swReading > 3300)
        return 0;
      if((swReading <= 3300) && (swReading > 2700))
        return 1;
      if((swReading <= 2700) && (swReading > 2200))
        return 2;
      if((swReading <= 2200) && (swReading > 1800))
        return 3;
      if((swReading <= 1800) && (swReading > 1400))
        return 4;
      if((swReading <= 1400) && (swReading > 1000))
        return 5;
      if((swReading <= 1000) && (swReading > 600))
        return 6;
      if(swReading <= 600)
        return 7;
    }
    else
    {
      if(swReading > 3300)
        return 7;
      if((swReading <= 3300) && (swReading > 2700))
        return 6;
      if((swReading <= 2700) && (swReading > 2200))
        return 5;
      if((swReading <= 2200) && (swReading > 1800))
        return 4;
      if((swReading <= 1800) && (swReading > 1400))
        return 3;
      if((swReading <= 1400) && (swReading > 1000))
        return 2;
      if((swReading <= 1000) && (swReading > 600))
        return 1;
      if(swReading <= 600)
        return 0;
    }

    return 0;
}

void UpdateSwPos()
{
    nSwPos[0] = CheckSwPos( GetAdcRaw(AnIn1), INVERT_SW_1);
    nSwPos[1] = CheckSwPos( GetAdcRaw(AnIn2), INVERT_SW_2);
    nSwPos[2] = CheckSwPos( GetAdcRaw(AnIn3), INVERT_SW_3);
    nSwPos[3] = CheckSwPos( GetAdcRaw(AnIn4), INVERT_SW_4);
    nSwPos[4] = CheckSwPos( GetAdcRaw(AnIn5), INVERT_SW_5);
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