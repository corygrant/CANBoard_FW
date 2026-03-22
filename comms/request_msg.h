#pragma once

#include "ch.h"
#include "hal.h"

// CAN message handling functions extracted from pdm.cpp

void CheckRequestMsgs(CANRxFrame *frame);