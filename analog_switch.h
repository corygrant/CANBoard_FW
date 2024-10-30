#pragma once
#include "ch.h"
#include "hal.h"
#include "analog.h"

#define ANALOG_SWITCH_THRESHOLD 2048

bool GetAnSwitch(AnalogChannel channel);