#include "analog_switch.h"

void Analog_Switch::Update()
{
    if(!pConfig->bEnabled)
    {
        fVal = 0;
        return;
    }

    fVal = GetAdcRaw(channel) ? 1.0f : 0.0f;
}

bool GetAnSwitch(AnalogChannel channel) {
    switch(channel)
    {
        case AnalogChannel::AnIn1: return GetAdcVolts(AnalogChannel::AnIn1) > ANALOG_SWITCH_THRESHOLD;
        case AnalogChannel::AnIn2: return GetAdcVolts(AnalogChannel::AnIn2) > ANALOG_SWITCH_THRESHOLD;
        case AnalogChannel::AnIn3: return GetAdcVolts(AnalogChannel::AnIn3) > ANALOG_SWITCH_THRESHOLD;
        case AnalogChannel::AnIn4: return GetAdcVolts(AnalogChannel::AnIn4) > ANALOG_SWITCH_THRESHOLD;
        case AnalogChannel::AnIn5: return GetAdcVolts(AnalogChannel::AnIn5) > ANALOG_SWITCH_THRESHOLD;
        case AnalogChannel::TempSensor: return false; // Temperature sensor is not a switch
        default: return false; // Invalid channel
    }
}