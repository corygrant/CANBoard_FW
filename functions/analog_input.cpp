#include "analog_input.h"

void Analog_Input::Update()
{
    if(!pConfig->bEnabled)
    {
        fVal = 0;
        return;
    }

    fVal = (float)GetAdcRaw(m_channel);
    fValMillivolts = GetAdcVolts(m_channel) * 1000.0;

    RotaryUpdate();
    SwitchUpdate();
}

void Analog_Input::RotaryUpdate()
{
    if(!pConfig->stRotary.bEnabled)
    {
        fRotaryPos = 0;
        return;
    }

    if (fValMillivolts < pConfig->stRotary.fOffset)
    {
        fRotaryPos = pConfig->stRotary.bInvert ? pConfig->stRotary.fMaxPos : 0;
    }
    else
    {
        float stepsAboveOffset = (fValMillivolts - pConfig->stRotary.fOffset) / pConfig->stRotary.fStep;
        int pos = static_cast<int>(stepsAboveOffset);
        if (pos > pConfig->stRotary.fMaxPos)
            pos = pConfig->stRotary.fMaxPos;

        fRotaryPos = pConfig->stRotary.bInvert ? (pConfig->stRotary.fMaxPos - pos) : pos;
    }
}

void Analog_Input::SwitchUpdate()
{
    if(!pConfig->stSwitch.bEnabled)
    {
        fSwitchVal = 0;
        return;
    }

    fSwitchVal = input.Check(pConfig->stSwitch.eMode, pConfig->stSwitch.bInvert, fValMillivolts > static_cast<float>(pConfig->stSwitch.nThreshold));
}