#pragma once

#include "port.h"
#include "enums.h"
#include "config.h"
#include "input.h"

extern float *pVarMap[VAR_MAP_SIZE];

class Digital_Output
{
public:
    Digital_Output(ioline_t line)
        : m_line(line)
    {};

    static const uint16_t nBaseIndex = 0x1200;

    void SetConfig(Config_Output *config)
    {
        pConfig = config;
        pInput = pVarMap[config->nInput];
    }

    void Update();

    float fVal;

private:
    const ioline_t m_line;

    Config_Output *pConfig;

    float *pInput;

    bool bLast;
};