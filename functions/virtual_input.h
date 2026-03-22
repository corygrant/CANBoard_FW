#pragma once

#include "port.h"
#include "config.h"
#include "input.h"

extern float *pVarMap[PDM_VAR_MAP_SIZE];

class VirtualInput
{
public:
    VirtualInput()
    {
    };

    static const uint16_t nBaseIndex = 0x1400;

    void SetConfig(Config_VirtualInput *config)
    {
        pConfig = config;
        pVar0 = pVarMap[config->nVar0];
        pVar1 = pVarMap[config->nVar1];
        pVar2 = pVarMap[config->nVar2];
    }

    void Update();

    float fVal;

private:
    Config_VirtualInput *pConfig;

    Input input;

    float *pVar0;
    float *pVar1;
    float *pVar2;

    bool bResult0;
    bool bResult1;
    bool bResult2;
    bool bResultSec0;
    bool bResultSec1;
};