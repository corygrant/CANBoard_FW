#pragma once

#include "port.h"
#include "config.h"

extern float *pVarMap[VAR_MAP_SIZE];

class Digital_Output
{
public:
    Digital_Output(ioline_t line)
        : m_line(line)
    {};

    static const uint16_t nBaseIndex = 0x2100;

    void SetConfig(Config_DigOutput *config)
    {
        pConfig = config;
        pInput = pVarMap[config->nInput];
    }

    void Update();

    float fVal;

private:
    const ioline_t m_line; 

    Config_DigOutput *pConfig;

    float *pInput;
};