#include "analog_switch.h"

bool GetAnSwitch(AnalogChannel channel) {
    switch(channel)
    {
        case AnIn1: return GetAdcRaw(AnIn1) > ANALOG_SWITCH_THRESHOLD;
        case AnIn2: return GetAdcRaw(AnIn2) > ANALOG_SWITCH_THRESHOLD;
        case AnIn3: return GetAdcRaw(AnIn3) > ANALOG_SWITCH_THRESHOLD;
        case AnIn4: return GetAdcRaw(AnIn4) > ANALOG_SWITCH_THRESHOLD;
        case AnIn5: return GetAdcRaw(AnIn5) > ANALOG_SWITCH_THRESHOLD;
        case TempSensor: return false; // Temperature sensor is not a switch
        default: return false; // Invalid channel
    }
}