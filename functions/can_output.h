#pragma once

#include "hal.h"

class CanOutput
{
public:
    bool CheckTxTime()
    {
        if (SYS_TIME - nLastTxTime >= nInterval)
        {
            nLastTxTime = SYS_TIME;
            return true;
        }

        return false;
    }

    CANTxFrame stFrame;
    uint16_t nInterval; //ms
    uint32_t nLastTxTime;
};