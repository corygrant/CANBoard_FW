#include "config_handler.h"
#include "msg.h"
#include "canboard_config.h"
#include "can.h"
#include "can_input.h"
#include "can_outputs.h"
#include "counter.h"
#include "condition.h"
#include "digital_input.h"
#include "digital_output.h"
#include "analog_input.h"
#include "flasher.h"
#include "virtual_input.h"

extern CanboardConfig stConfig;
extern Digital_Input digIn[NUM_DIG_INPUTS];
extern Analog_Input analogIn[NUM_ANALOG_INPUTS];
extern Digital_Output digOut[NUM_DIG_OUTPUTS];
extern CanInput canIn[NUM_CAN_INPUTS];
extern CanOutputs canOutputs;
extern VirtualInput virtIn[NUM_VIRT_INPUTS];
extern Flasher flasher[NUM_FLASHERS];
extern Counter counter[NUM_COUNTERS];
extern Condition condition[NUM_CONDITIONS];

void ApplyAllConfig()
{
    ApplyConfig(Digital_Input::nBaseIndex);
    ApplyConfig(Digital_Output::nBaseIndex);
    ApplyConfig(Analog_Input::nBaseIndex);
    ApplyConfig(CanInput::nBaseIndex);
    ApplyConfig(CanOutputs::nBaseIndex);
    ApplyConfig(VirtualInput::nBaseIndex);
    ApplyConfig(Flasher::nBaseIndex);
    ApplyConfig(Counter::nBaseIndex);
    ApplyConfig(Condition::nBaseIndex);
}

void ApplyConfig(uint16_t nIndex)
{
    uint16_t nBaseIndex = nIndex & 0xFF00;

    // Device config (0x0000)
    if (nBaseIndex == 0x0000)
    {
        // TODO: Change CAN speed and filters without requiring reset
        
        SetCanFilterEnabled(stConfig.stDevConfig.bCanFilterEnabled);
    }

    if (nBaseIndex == Digital_Input::nBaseIndex)
    {
        for (uint8_t i = 0; i < NUM_DIG_INPUTS; i++)
            digIn[i].SetConfig(&stConfig.stDigInput[i]);
    }

    if (nBaseIndex == Digital_Output::nBaseIndex)
    {
        for (uint8_t i = 0; i < NUM_DIG_OUTPUTS; i++)
            digOut[i].SetConfig(&stConfig.stDigOutput[i]);
    }

    if (nBaseIndex == Analog_Input::nBaseIndex)
    {
        for (uint8_t i = 0; i < NUM_ANALOG_INPUTS; i++)
            analogIn[i].SetConfig(&stConfig.stAnalogInput[i]);
    }

    if (nBaseIndex == CanInput::nBaseIndex)
    {
        ClearCanFilters(); // Clear all filters before setting new ones

        // Set filter for CAN settings request message, (Base ID - 1)
        // Use filter 0, it is always enabled to allow all messages by hal so it must be used
        SetCanFilterId(0, stConfig.stDevConfig.nBaseId - 1, false);

        for (uint8_t i = 0; i < NUM_CAN_INPUTS; i++)
        {
            canIn[i].SetConfig(&stConfig.stCanInput[i]);
            if(!stConfig.stCanInput[i].bEnabled)
                continue; // Skip if not enabled
            
            // Set filter for this input
            uint32_t nId = 0;
            nId = stConfig.stCanInput[i].nID;
            SetCanFilterId(i + 1, nId, stConfig.stCanInput[i].nIDE == 1);
        }

        //TODO: Set can filter without requiring reset, need a new message to indicate all IDs set before stopping CAN
    }

    if (nBaseIndex == CanOutputs::nBaseIndex)
    {
        canOutputs.SetConfig(stConfig.stCanOutput);

        CanOutputs::InitAllFrames();
    }

    if (nBaseIndex == VirtualInput::nBaseIndex)
    {
        for (uint8_t i = 0; i < NUM_VIRT_INPUTS; i++)
            virtIn[i].SetConfig(&stConfig.stVirtualInput[i]);
    }

    if (nBaseIndex == Flasher::nBaseIndex)
    {
        for (uint8_t i = 0; i < NUM_FLASHERS; i++)
            flasher[i].SetConfig(&stConfig.stFlasher[i]);
    }

    if (nBaseIndex == Counter::nBaseIndex)
    {
        for (uint8_t i = 0; i < NUM_COUNTERS; i++)
            counter[i].SetConfig(&stConfig.stCounter[i]);
    }

    if (nBaseIndex == Condition::nBaseIndex)
    {
        for (uint8_t i = 0; i < NUM_CONDITIONS; i++)
            condition[i].SetConfig(&stConfig.stCondition[i]);
    }
}