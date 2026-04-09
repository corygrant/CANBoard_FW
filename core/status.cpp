#include "status.h"
#include "canboard.h"
#include "config.h"
#include "canboard_config.h"
#include "digital_input.h"
#include "digital_output.h"
#include "analog_input.h"
#include "can_input.h"
#include "virtual_input.h"
#include "flasher.h"
#include "counter.h"
#include "condition.h"

bool GetInputVal(uint8_t nInput)
{
    if (nInput >= NUM_DIG_INPUTS)
        return false;

    return digIn[nInput].fVal;
}

bool GetOutputState(uint8_t nOutput)
{
    if (nOutput >= NUM_DIG_OUTPUTS)
        return false;

    return static_cast<bool>(digOut[nOutput].fVal);
}

uint16_t GetAnalogInputVal(uint8_t nInput)
{
    if (nInput >= NUM_ANALOG_INPUTS)
        return 0;

    return static_cast<uint16_t>(analogIn[nInput].fVal);
}

float GetAnalogInputMv(uint8_t nInput)
{
    if (nInput >= NUM_ANALOG_INPUTS)
        return 0;

    return analogIn[nInput].fValMillivolts;
}

uint8_t GetRotarySwitchPos(uint8_t nInput)
{
    if (nInput >= NUM_ANALOG_INPUTS)
        return 0;

    return static_cast<uint8_t>(analogIn[nInput].fRotaryPos);
}

bool GetAnalogSwitchVal(uint8_t nInput)
{
    if (nInput >= NUM_ANALOG_INPUTS)
        return false;

    return static_cast<bool>(analogIn[nInput].fSwitchVal);
}

bool GetAnyCanInEnable()
{
    for (uint8_t i = 0; i < NUM_CAN_INPUTS; i++)
    {
        if (stConfig.stCanInput[i].bEnabled)
            return true;
    }
    return false;
}

bool GetCanInEnable(uint8_t nInput)
{
    if (nInput >= NUM_CAN_INPUTS)
        return false;

    return stConfig.stCanInput[nInput].bEnabled;
}

bool GetCanInOutput(uint8_t nInput)
{
    if (nInput >= NUM_CAN_INPUTS)
        return false;

    return canIn[nInput].fOutput;
}

float GetCanInVal(uint8_t nInput)
{
    if (nInput >= NUM_CAN_INPUTS)
        return false;

    return canIn[nInput].fVal;
}

float GetCanInFactor(uint8_t nInput)
{
    if (nInput >= NUM_CAN_INPUTS)
        return 0;

    return stConfig.stCanInput[nInput].fFactor;
}

float GetCanInOffset(uint8_t nInput)
{
    if (nInput >= NUM_CAN_INPUTS)
        return 0;

    return stConfig.stCanInput[nInput].fOffset;
}

ByteOrder GetCanInByteOrder(uint8_t nInput)
{
    if (nInput >= NUM_CAN_INPUTS)
        return ByteOrder::LittleEndian;

    return stConfig.stCanInput[nInput].eByteOrder;
}

uint32_t GetCanInOutputs()
{
    uint32_t result = 0;
    
    for (uint8_t i = 0; i < NUM_CAN_INPUTS; i++) {
        result |= (((uint32_t)canIn[i].fVal & 0x01) << i);
    }
    
    return result;
}

bool GetAnyVirtInEnable()
{
    for (uint8_t i = 0; i < NUM_VIRT_INPUTS; i++)
    {
        if (stConfig.stVirtualInput[i].bEnabled)
            return true;
    }
    return false;
}

bool GetVirtInVal(uint8_t nInput)
{
    if (nInput >= NUM_VIRT_INPUTS)
        return false;

    return virtIn[nInput].fVal;
}

uint32_t GetVirtIns()
{
    uint32_t result = 0;
    
    for (uint8_t i = 0; i < NUM_VIRT_INPUTS; i++) {
        result |= (((uint32_t)virtIn[i].fVal & 0x01) << i);
    }
    
    return result;
}

bool GetAnyFlasherEnable()
{
    for (uint8_t i = 0; i < NUM_FLASHERS; i++)
    {
        if (stConfig.stFlasher[i].bEnabled)
            return true;
    }
    return false;
}

bool GetFlasherVal(uint8_t nFlasher)
{
    if (nFlasher >= NUM_FLASHERS)
        return false;

    return flasher[nFlasher].fVal;
}

bool GetAnyCounterEnable()
{
    for (uint8_t i = 0; i < NUM_COUNTERS; i++)
    {
        if (stConfig.stCounter[i].bEnabled)
            return true;
    }
    return false;
}

float GetCounterVal(uint8_t nCounter)
{
    if (nCounter >= NUM_COUNTERS)
        return 0;

    return counter[nCounter].fVal;
}

bool GetAnyConditionEnable()
{
    for (uint8_t i = 0; i < NUM_CONDITIONS; i++)
    {
        if (stConfig.stCondition[i].bEnabled)
            return true;
    }
    return false;
}

uint32_t GetConditions()
{
    uint32_t result = 0;
    
    for (uint8_t i = 0; i < NUM_CONDITIONS; i++) {
        result |= (((uint32_t)condition[i].fVal & 0x01) << i);
    }
    
    return result;
}