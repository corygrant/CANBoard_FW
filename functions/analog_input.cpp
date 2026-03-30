#include "analog_input.h"

void Analog_Input::Update()
{
    if(!pConfig->bEnabled)
    {
        fVal = 0;
        return;
    }

    fVal = (float)GetAdcRaw(channel);
    fValMillivolts = GetAdcVolts(channel);
}