#pragma once

#include "port.h"
#include "config.h"

class Analog_Input
{
public:
    Analog_Input() {
    };

    static const uint16_t nBaseIndex = 0x1200;

    void SetConfig(Config_AnalogInput *config)
    {
        pConfig = config;
    }

    void Update();

    float fVal;
    float fValMillivolts;

private:
    Config_AnalogInput* pConfig;

    AnalogChannel channel;
};