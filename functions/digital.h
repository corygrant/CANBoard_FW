#pragma once

#include "port.h"
#include "enums.h"
#include "config.h"
#include "input.h"

class Digital
{
public:
    Digital(ioline_t line)
        : m_line(line)
    {};

    static const uint16_t nBaseIndex = 0x1200;

    void SetConfig(Config_Input *config)
    {
        pConfig = config;

        SetPull(config->ePull);
    }

    void Update();

    float fVal;

private:
    const ioline_t m_line;

    void SetPull(InputPull pull);

    Config_Input *pConfig;

    Input input;

    bool bInit;
    bool bLast;
    bool bCheck;
    uint32_t nLastTrigTime;
};