#pragma once

#include <cstdint>
#include "enums.h"
#include "config.h"

msg_t InitCan(Config_DeviceConfig *conf);
void StopCan();
void ClearCanFilters();
void SetCanFilterId(uint8_t nFilterNum, uint32_t nId, bool bExtended);
void SetCanFilterEnabled(bool bEnabled);
uint32_t GetLastCanRxTime();