#pragma once

#include <cstdint>
#include "config.h"

extern float *pVarMap[PDM_VAR_MAP_SIZE];

class Flasher
{
public:
    Flasher() {

    };

    static const uint16_t nBaseIndex = 0x1700;

    void SetConfig(Config_Flasher* config)
    {
        pConfig = config;
        pInput = pVarMap[config->nInput];
    }

    void Update(uint32_t timeNow);

    float fVal;

private:
    Config_Flasher* pConfig;
    
    float *pInput;

    uint32_t nTimeOff;
    uint32_t nTimeOn;
};