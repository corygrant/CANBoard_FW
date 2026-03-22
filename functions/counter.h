#pragma once

#include <cstdint>
#include "config.h"
#include "dingopdm_config.h"

extern float *pVarMap[PDM_VAR_MAP_SIZE];

class Counter
{
public:
    Counter() {

    };

    static const uint16_t nBaseIndex = 0x1600;

    void SetConfig(Config_Counter* config)
    {
        pConfig = config;
        pIncInput = pVarMap[config->nIncInput];
        pDecInput = pVarMap[config->nDecInput];
        pResetInput = pVarMap[config->nResetInput];
    }

    void Update();

    float fVal;

private:
    Config_Counter* pConfig;
    
    float *pIncInput;
    float *pDecInput;
    float *pResetInput;

    bool bLastInc;
    bool bLastDec;
    bool bLastReset;
    
    uint32_t nLastIncTime;
    uint32_t nLastDecTime;
};
