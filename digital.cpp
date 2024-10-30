#include "digital.h"

bool bLastDigOut[4] = {false, false, false, false};

bool GetDigIn(DigIn input)
{
    switch(input)
    {
        case DigIn1: return palReadLine(LINE_DI1) == PAL_LOW;
        case DigIn2: return palReadLine(LINE_DI2) == PAL_LOW;
        case DigIn3: return palReadLine(LINE_DI3) == PAL_LOW;
        case DigIn4: return palReadLine(LINE_DI4) == PAL_LOW;
        case DigIn5: return palReadLine(LINE_DI5) == PAL_LOW;
        case DigIn6: return palReadLine(LINE_DI6) == PAL_LOW;
        case DigIn7: return palReadLine(LINE_DI7) == PAL_LOW;
        case DigIn8: return palReadLine(LINE_DI8) == PAL_LOW;
        case IdSel1: return palReadLine(LINE_CAN_ID_1) == PAL_HIGH;
        case IdSel2: return palReadLine(LINE_CAN_ID_2) == PAL_HIGH;
        default: return false; // Invalid input
    }
}

void SetDigOut(DigOut output, bool val)
{
    switch(output)
    {
        case DigOut1: 
            palWriteLine(LINE_DO1, val ? PAL_HIGH : PAL_LOW); 
            bLastDigOut[0] = val;
            break;
        case DigOut2: 
            palWriteLine(LINE_DO2, val ? PAL_HIGH : PAL_LOW); 
            bLastDigOut[1] = val;
            break;
        case DigOut3: 
            palWriteLine(LINE_DO3, val ? PAL_HIGH : PAL_LOW);
            bLastDigOut[2] = val;
            break;
        case DigOut4: 
            palWriteLine(LINE_DO4, val ? PAL_HIGH : PAL_LOW);
            bLastDigOut[3] = val;
            break;
        default: break; // Invalid output
    }
} 

bool GetDigOut(DigOut output)
{
    switch(output)
    {
        case DigOut1: return bLastDigOut[0];
        case DigOut2: return bLastDigOut[1];
        case DigOut3: return bLastDigOut[2];
        case DigOut4: return bLastDigOut[3];
        default: return false; // Invalid output
    }
}