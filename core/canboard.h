#pragma once

#include "enums.h"
#include "config.h"
#include "status.h"

class Digital;
class CanInput;
class CanOutputs;
class VirtualInput;
class Flasher;
class Counter;
class Condition;

extern Digital in[NUM_INPUTS];
extern CanInput canIn[NUM_CAN_INPUTS];
extern CanOutputs canOutputs;
extern VirtualInput virtIn[NUM_VIRT_INPUTS];
extern Flasher flasher[NUM_FLASHERS];
extern Counter counter[NUM_COUNTERS];
extern Condition condition[NUM_CONDITIONS];

extern CanboardConfig stConfig;
extern CanboardConfig stConfigTemp; // Used for staging new config before applying
extern float *pVarMap[PDM_VAR_MAP_SIZE];

void InitCanboard();