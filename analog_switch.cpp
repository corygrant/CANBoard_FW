#include "analog_switch.h"

bool GetAnSwitch(AnalogChannel channel) {
    switch(channel)
    {
        case AnIn1: return GetAdcVolts(AnIn1) > ANALOG_SWITCH_THRESHOLD;
        case AnIn2: return GetAdcVolts(AnIn2) > ANALOG_SWITCH_THRESHOLD;
        case AnIn3: return GetAdcVolts(AnIn3) > ANALOG_SWITCH_THRESHOLD;
        case AnIn4: return GetAdcVolts(AnIn4) > ANALOG_SWITCH_THRESHOLD;
        case AnIn5: return GetAdcVolts(AnIn5) > ANALOG_SWITCH_THRESHOLD;
        case TempSensor: return false; // Temperature sensor is not a switch
        default: return false; // Invalid channel
    }
}