#pragma once

#include <cstdint>
#include "config.h"

extern float *pVarMap[PDM_VAR_MAP_SIZE];

class Condition
{
public:
    Condition() {
    };

    static const uint16_t nBaseIndex = 0x1500;

    void SetConfig(Config_Condition* config)
    {
        pConfig = config;
        pInput = pVarMap[config->nInput];
    }

    void Update();

    float fVal;

private:
    Config_Condition* pConfig;
    
    float *pInput;
};