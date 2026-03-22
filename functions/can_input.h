#pragma once

#include "port.h"
#include "config.h"
#include "hal.h"
#include "input.h"

class CanInput
{
public:
    CanInput() {
    };

    static const uint16_t nBaseIndex = 0x1300;

    bool CheckMsg(CANRxFrame frame);

    void SetConfig(Config_CanInput* config) { pConfig = config; }
    void CheckTimeout();

    float fOutput;
    float fVal;

private:
    Config_CanInput* pConfig;

    Input input;

    uint32_t nLastRxTime;
};