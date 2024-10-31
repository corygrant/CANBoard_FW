#pragma once
#include "ch.h"
#include "hal.h"
#include "analog.h"

#define ANALOG_SWITCH_THRESHOLD 2.5f

bool GetAnSwitch(AnalogChannel channel);