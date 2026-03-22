#pragma once

#include "port.h"
#include "config.h"
#include "hal.h"
#include "can_output.h"

// Same value, used to show difference in intent between number of frames vs number of outputs
#define CAN_OUT_FRAMES NUM_CAN_OUTPUTS // Max possible frames is when all outputs have unique IDs

extern float *pVarMap[VAR_MAP_SIZE];

class CanOutputs
{
public:

    static const uint16_t nBaseIndex = 0x2000;

    static void SetConfig(Config_CanOutput config[]) {
        for(uint8_t i = 0; i < NUM_CAN_OUTPUTS; i++) {
            pConfigs[i] = &config[i];
            pInput[i] = pVarMap[config[i].nInput];
        }
    }

    static void InitAllFrames();
    static void Update();

private:
    static Config_CanOutput* pConfigs[NUM_CAN_OUTPUTS];

    static float *pInput[NUM_CAN_OUTPUTS];

    static CanOutput canOut[CAN_OUT_FRAMES];
    
    // Frame indexes are independent of CAN output indexes, multiple outputs may be packed into the same frame based on ID
    static int8_t nAssignedOut[NUM_CAN_OUTPUTS]; // Maps CAN output index to frame index, -1 if not assigned

    static void CheckTime(uint8_t index);
    static void ClearFrames();
    static uint8_t CalcDlc(uint8_t nStartBit, uint8_t nBitLength);
};


// CAN Outputs are packed into frames based on ID, multiple outputs may share a frame if they have the same ID and IDE. 
// Each output configures which bits of the frame it uses for its value. 
// When an output is updated, it updates the relevant bits in its assigned frame, and the frame is transmitted at the configured interval.