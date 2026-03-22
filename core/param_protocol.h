#pragma once

#include "port.h"
#include "param_registry.h"

struct ParamMsg
{
    MsgCmd eCmd;
    uint16_t nIndex;
    uint8_t nSubIndex;
    uint32_t nValue;
};

MsgCmd ProcessParamMsg(CANRxFrame *rx, uint16_t *nIndex);
void SetAllDefaultParams(bool temp = false);