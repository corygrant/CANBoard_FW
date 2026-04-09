#pragma once

#include "port.h"
#include "config.h"
#include "input.h"

class Analog_Input
{
public:
    Analog_Input() {
    };

    static const uint16_t nBaseIndex = 0x2200;

    void SetConfig(Config_AnalogInput *config)
    {
        pConfig = config;
    }

    void Update();

    float fVal;
    float fValMillivolts;
    float fRotaryPos;
    float fSwitchVal;

private:
    Config_AnalogInput* pConfig;

    AnalogChannel channel;

    Input input;

    void RotaryUpdate();
    void SwitchUpdate();
};