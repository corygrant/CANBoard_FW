#pragma once

#include "port.h"
#include "digital_input.h"
#include "digital_output.h"
#include "analog_input.h"

extern Digital_Input digIn[NUM_DIG_INPUTS];
extern Digital_Input idSel[2];
extern Digital_Output digOut[NUM_DIG_OUTPUTS];
extern Analog_Input analogIn[NUM_ANALOG_INPUTS];