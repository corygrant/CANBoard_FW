#pragma once

#include "enums.h"
#include "config.h"
#include "status.h"

class Digital_Input;
class Analog_Input;
class Digital_Output;
class CanInput;
class CanOutputs;
class VirtualInput;
class Flasher;
class Counter;
class Condition;

extern Digital_Input digIn[NUM_DIG_INPUTS];
extern Digital_Input idSel[2];
extern Analog_Input analogIn[NUM_ANALOG_INPUTS];
extern Digital_Output digOut[NUM_DIG_OUTPUTS];
extern CanInput canIn[NUM_CAN_INPUTS];
extern CanOutputs canOutputs;
extern VirtualInput virtIn[NUM_VIRT_INPUTS];
extern Flasher flasher[NUM_FLASHERS];
extern Counter counter[NUM_COUNTERS];
extern Condition condition[NUM_CONDITIONS];

extern CanboardConfig stConfig;
extern CanboardConfig stConfigTemp; // Used for staging new config before applying
extern float *pVarMap[VAR_MAP_SIZE];

void InitCanboard();
uint8_t GetCanOffset();