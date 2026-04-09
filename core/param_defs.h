#pragma once

#include "config.h"
#include "param_registry.h"

// Parameter definitions for compile-time registration
// Index = base + instance, SubIndex = parameter within instance

//=============================================================================
// Device Config Parameters - Base 0x0000
//=============================================================================
#define DEVICE_CONFIG_PARAMS() \
    {0x0000, 0, &stConfig.stDevConfig.nBaseId,            &stConfigTemp.stDevConfig.nBaseId,           ParamType::UInt16, 0x7D0, 0, 0x7FF}, \
    {0x0000, 1, &stConfig.stDevConfig.nParamRxId,         &stConfigTemp.stDevConfig.nParamRxId,        ParamType::UInt16, 0x080, 0, 0x7FF}, \
    {0x0000, 2, &stConfig.stDevConfig.nParamTxId,         &stConfigTemp.stDevConfig.nParamTxId,        ParamType::UInt16, 0x081, 0, 0x7FF}, \
    {0x0000, 3, &stConfig.stDevConfig.eCanSpeed,          &stConfigTemp.stDevConfig.eCanSpeed,         ParamType::Enum,   static_cast<uint32_t>(CanBitrate::Bitrate_500K), 0, 4}, \
    {0x0000, 4, &stConfig.stDevConfig.bCanFilterEnabled,  &stConfigTemp.stDevConfig.bCanFilterEnabled, ParamType::Bool,   0, 0, 1}

//=============================================================================
// Digital Input Parameters - Base 0x1200
//=============================================================================
#define DIGITAL_INPUT_PARAMS(i) \
    {0x1200 + (i), 0, &stConfig.stDigInput[i].bEnabled,      &stConfigTemp.stDigInput[i].bEnabled,     ParamType::Bool,   0, 0, 1}, \
    {0x1200 + (i), 1, &stConfig.stDigInput[i].eMode,         &stConfigTemp.stDigInput[i].eMode,        ParamType::Enum,   static_cast<uint32_t>(InputMode::Momentary), 0, 1}, \
    {0x1200 + (i), 2, &stConfig.stDigInput[i].bInvert,       &stConfigTemp.stDigInput[i].bInvert,      ParamType::Bool,   0, 0, 1}, \
    {0x1200 + (i), 3, &stConfig.stDigInput[i].nDebounceTime, &stConfigTemp.stDigInput[i].nDebounceTime,ParamType::UInt16, 20, 0, 1000}, \
    {0x1200 + (i), 4, &stConfig.stDigInput[i].ePull,         &stConfigTemp.stDigInput[i].ePull,        ParamType::Enum,   static_cast<uint32_t>(InputPull::None), 0, 2}

//=============================================================================
// CAN Input Parameters - Base 0x1300
//=============================================================================
#define CAN_INPUT_PARAMS(i) \
    {0x1300 + (i), 0,  &stConfig.stCanInput[i].bEnabled,        &stConfigTemp.stCanInput[i].bEnabled,        ParamType::Bool,   0, 0, 1}, \
    {0x1300 + (i), 1,  &stConfig.stCanInput[i].bTimeoutEnabled, &stConfigTemp.stCanInput[i].bTimeoutEnabled, ParamType::Bool,   0, 0, 1}, \
    {0x1300 + (i), 2,  &stConfig.stCanInput[i].nTimeout,        &stConfigTemp.stCanInput[i].nTimeout,        ParamType::UInt16, 1000, 0, 60000}, \
    {0x1300 + (i), 3,  &stConfig.stCanInput[i].nIDE,            &stConfigTemp.stCanInput[i].nIDE,            ParamType::UInt8,  0, 0, 1}, \
    {0x1300 + (i), 4,  &stConfig.stCanInput[i].nID,             &stConfigTemp.stCanInput[i].nID,             ParamType::UInt32, 0, 0, 536870911}, \
    {0x1300 + (i), 5,  &stConfig.stCanInput[i].nStartBit,       &stConfigTemp.stCanInput[i].nStartBit,       ParamType::UInt8,  0, 0, 63}, \
    {0x1300 + (i), 6,  &stConfig.stCanInput[i].nBitLength,      &stConfigTemp.stCanInput[i].nBitLength,      ParamType::UInt8,  8, 1, 32}, \
    {0x1300 + (i), 7,  &stConfig.stCanInput[i].fFactor,         &stConfigTemp.stCanInput[i].fFactor,         ParamType::Float,  F(1.0f), F(-1e9f), F(1e9f)}, \
    {0x1300 + (i), 8,  &stConfig.stCanInput[i].fOffset,         &stConfigTemp.stCanInput[i].fOffset,         ParamType::Float,  F(0.0f), F(-1e9f), F(1e9f)}, \
    {0x1300 + (i), 9,  &stConfig.stCanInput[i].eByteOrder,      &stConfigTemp.stCanInput[i].eByteOrder,      ParamType::Enum,   static_cast<uint32_t>(ByteOrder::LittleEndian), 0, 1}, \
    {0x1300 + (i), 10, &stConfig.stCanInput[i].bSigned,         &stConfigTemp.stCanInput[i].bSigned,         ParamType::Bool,   0, 0, 1}, \
    {0x1300 + (i), 11, &stConfig.stCanInput[i].eOperator,       &stConfigTemp.stCanInput[i].eOperator,       ParamType::Enum,   static_cast<uint32_t>(Operator::Equal), 0, 7}, \
    {0x1300 + (i), 12, &stConfig.stCanInput[i].fOperand,        &stConfigTemp.stCanInput[i].fOperand,        ParamType::Float,  F(0.0f), F(-1e9f), F(1e9f)}, \
    {0x1300 + (i), 13, &stConfig.stCanInput[i].eMode,           &stConfigTemp.stCanInput[i].eMode,           ParamType::Enum,   static_cast<uint32_t>(InputMode::Momentary), 0, 1}
    
//=============================================================================
// Virtual Input Parameters - Base 0x1400
//=============================================================================
#define VIRTUAL_INPUT_PARAMS(i) \
    {0x1400 + (i), 0,  &stConfig.stVirtualInput[i].bEnabled, &stConfigTemp.stVirtualInput[i].bEnabled, ParamType::Bool,   0, 0, 1}, \
    {0x1400 + (i), 1,  &stConfig.stVirtualInput[i].bNot0,    &stConfigTemp.stVirtualInput[i].bNot0,    ParamType::Bool,   0, 0, 1}, \
    {0x1400 + (i), 2,  &stConfig.stVirtualInput[i].nVar0,    &stConfigTemp.stVirtualInput[i].nVar0,    ParamType::UInt16, 0, 0, VAR_MAP_SIZE - 1}, \
    {0x1400 + (i), 3,  &stConfig.stVirtualInput[i].eCond0,   &stConfigTemp.stVirtualInput[i].eCond0,   ParamType::Enum,   static_cast<uint32_t>(BoolOperator::And), 0, 2}, \
    {0x1400 + (i), 4,  &stConfig.stVirtualInput[i].bNot1,    &stConfigTemp.stVirtualInput[i].bNot1,    ParamType::Bool,   0, 0, 1}, \
    {0x1400 + (i), 5,  &stConfig.stVirtualInput[i].nVar1,    &stConfigTemp.stVirtualInput[i].nVar1,    ParamType::UInt16, 0, 0, VAR_MAP_SIZE - 1}, \
    {0x1400 + (i), 6,  &stConfig.stVirtualInput[i].eCond1,   &stConfigTemp.stVirtualInput[i].eCond1,   ParamType::Enum,   static_cast<uint32_t>(BoolOperator::And), 0, 2}, \
    {0x1400 + (i), 7,  &stConfig.stVirtualInput[i].bNot2,    &stConfigTemp.stVirtualInput[i].bNot2,    ParamType::Bool,   0, 0, 1}, \
    {0x1400 + (i), 8,  &stConfig.stVirtualInput[i].nVar2,    &stConfigTemp.stVirtualInput[i].nVar2,    ParamType::UInt16, 0, 0, VAR_MAP_SIZE - 1}, \
    {0x1400 + (i), 9,  &stConfig.stVirtualInput[i].eMode,    &stConfigTemp.stVirtualInput[i].eMode,    ParamType::Enum,   static_cast<uint32_t>(InputMode::Momentary), 0, 1}

//=============================================================================
// Condition Parameters - Base 0x1500
//=============================================================================
#define CONDITION_PARAMS(i) \
    {0x1500 + (i), 0, &stConfig.stCondition[i].bEnabled,  &stConfigTemp.stCondition[i].bEnabled,  ParamType::Bool,   0, 0, 1}, \
    {0x1500 + (i), 1, &stConfig.stCondition[i].nInput,    &stConfigTemp.stCondition[i].nInput,    ParamType::UInt16, 0, 0, VAR_MAP_SIZE - 1}, \
    {0x1500 + (i), 2, &stConfig.stCondition[i].eOperator, &stConfigTemp.stCondition[i].eOperator, ParamType::Enum,   static_cast<uint32_t>(Operator::Equal), 0, 7}, \
    {0x1500 + (i), 3, &stConfig.stCondition[i].fArg,      &stConfigTemp.stCondition[i].fArg,      ParamType::Float,  F(0.0f), F(-1e9f), F(1e9f)}

//=============================================================================
// Counter Parameters - Base 0x1600
//=============================================================================
#define COUNTER_PARAMS(i) \
    {0x1600 + (i), 0,  &stConfig.stCounter[i].bEnabled,    &stConfigTemp.stCounter[i].bEnabled,    ParamType::Bool,   0, 0, 1}, \
    {0x1600 + (i), 1,  &stConfig.stCounter[i].nIncInput,   &stConfigTemp.stCounter[i].nIncInput,   ParamType::UInt16, 0, 0, VAR_MAP_SIZE - 1}, \
    {0x1600 + (i), 2,  &stConfig.stCounter[i].nDecInput,   &stConfigTemp.stCounter[i].nDecInput,   ParamType::UInt16, 0, 0, VAR_MAP_SIZE - 1}, \
    {0x1600 + (i), 3,  &stConfig.stCounter[i].nResetInput, &stConfigTemp.stCounter[i].nResetInput, ParamType::UInt16, 0, 0, VAR_MAP_SIZE - 1}, \
    {0x1600 + (i), 4,  &stConfig.stCounter[i].nMinCount,   &stConfigTemp.stCounter[i].nMinCount,   ParamType::UInt8,  0, 0, 255}, \
    {0x1600 + (i), 5,  &stConfig.stCounter[i].nMaxCount,   &stConfigTemp.stCounter[i].nMaxCount,   ParamType::UInt8,  10, 0, 255}, \
    {0x1600 + (i), 6,  &stConfig.stCounter[i].eIncEdge,    &stConfigTemp.stCounter[i].eIncEdge,    ParamType::Enum,   static_cast<uint32_t>(InputEdge::Rising), 0, 2}, \
    {0x1600 + (i), 7,  &stConfig.stCounter[i].eDecEdge,    &stConfigTemp.stCounter[i].eDecEdge,    ParamType::Enum,   static_cast<uint32_t>(InputEdge::Rising), 0, 2}, \
    {0x1600 + (i), 8,  &stConfig.stCounter[i].eResetEdge,  &stConfigTemp.stCounter[i].eResetEdge,  ParamType::Enum,   static_cast<uint32_t>(InputEdge::Rising), 0, 2}, \
    {0x1600 + (i), 9,  &stConfig.stCounter[i].bWrapAround, &stConfigTemp.stCounter[i].bWrapAround, ParamType::Bool,   0, 0, 1}, \
    {0x1600 + (i), 10, &stConfig.stCounter[i].bHoldToReset,&stConfigTemp.stCounter[i].bHoldToReset, ParamType::Bool,   0, 0, 1}, \
    {0x1600 + (i), 11, &stConfig.stCounter[i].nResetTime,  &stConfigTemp.stCounter[i].nResetTime,   ParamType::UInt16, 2000, 0, 10000}

//=============================================================================
// Flasher Parameters - Base 0x1700
//=============================================================================
#define FLASHER_PARAMS(i) \
    {0x1700 + (i), 0, &stConfig.stFlasher[i].bEnabled,      &stConfigTemp.stFlasher[i].bEnabled,      ParamType::Bool,   0, 0, 1}, \
    {0x1700 + (i), 1, &stConfig.stFlasher[i].nInput,        &stConfigTemp.stFlasher[i].nInput,        ParamType::UInt16, 0, 0, VAR_MAP_SIZE - 1}, \
    {0x1700 + (i), 2, &stConfig.stFlasher[i].nFlashOnTime,  &stConfigTemp.stFlasher[i].nFlashOnTime,  ParamType::UInt16, 500, 0, 5000}, \
    {0x1700 + (i), 3, &stConfig.stFlasher[i].nFlashOffTime, &stConfigTemp.stFlasher[i].nFlashOffTime, ParamType::UInt16, 500, 0, 5000}, \
    {0x1700 + (i), 4, &stConfig.stFlasher[i].bSingleCycle,  &stConfigTemp.stFlasher[i].bSingleCycle,  ParamType::Bool,   0, 0, 1}

//=============================================================================
// CAN Output Parameters - Base 0x2000
//=============================================================================
#define CAN_OUTPUT_PARAMS(i) \
    {0x2000 + (i), 0,  &stConfig.stCanOutput[i].bEnabled,   &stConfigTemp.stCanOutput[i].bEnabled,   ParamType::Bool,   0, 0, 1}, \
    {0x2000 + (i), 1,  &stConfig.stCanOutput[i].nInput,     &stConfigTemp.stCanOutput[i].nInput,     ParamType::UInt16, 0, 0, VAR_MAP_SIZE - 1}, \
    {0x2000 + (i), 2,  &stConfig.stCanOutput[i].nIDE,       &stConfigTemp.stCanOutput[i].nIDE,       ParamType::UInt8,  0, 0, 1}, \
    {0x2000 + (i), 3,  &stConfig.stCanOutput[i].nID,        &stConfigTemp.stCanOutput[i].nID,        ParamType::UInt32, 0, 0, 536870911}, \
    {0x2000 + (i), 4,  &stConfig.stCanOutput[i].nStartBit,  &stConfigTemp.stCanOutput[i].nStartBit,  ParamType::UInt8,  0, 0, 63}, \
    {0x2000 + (i), 5,  &stConfig.stCanOutput[i].nBitLength, &stConfigTemp.stCanOutput[i].nBitLength, ParamType::UInt8,  8, 1, 32}, \
    {0x2000 + (i), 6,  &stConfig.stCanOutput[i].fFactor,    &stConfigTemp.stCanOutput[i].fFactor,    ParamType::Float,  F(1.0f), F(-1e9f), F(1e9f)}, \
    {0x2000 + (i), 7,  &stConfig.stCanOutput[i].fOffset,    &stConfigTemp.stCanOutput[i].fOffset,    ParamType::Float,  F(0.0f), F(-1e9f), F(1e9f)}, \
    {0x2000 + (i), 8,  &stConfig.stCanOutput[i].eByteOrder, &stConfigTemp.stCanOutput[i].eByteOrder, ParamType::Enum,   static_cast<uint32_t>(ByteOrder::LittleEndian), 0, 1}, \
    {0x2000 + (i), 9,  &stConfig.stCanOutput[i].bSigned,    &stConfigTemp.stCanOutput[i].bSigned,    ParamType::Bool,   0, 0, 1}, \
    {0x2000 + (i), 10, &stConfig.stCanOutput[i].nInterval,  &stConfigTemp.stCanOutput[i].nInterval,  ParamType::UInt16, 1000, 0, 60000}

//=============================================================================
// Digital Output Parameters - Base 0x2100
//=============================================================================
#define DIGITAL_OUTPUT_PARAMS(i) \
    {0x2100 + (i), 0, &stConfig.stDigOutput[i].bEnabled,     &stConfigTemp.stDigOutput[i].bEnabled,    ParamType::Bool,   0, 0, 1}

//=============================================================================
// Analog Input Parameters - Base 0x2200
//=============================================================================
#define ANALOG_INPUT_PARAMS(i) \
    {0x2200 + (i), 0, &stConfig.stAnalogInput[i].bEnabled, &stConfigTemp.stAnalogInput[i].bEnabled, ParamType::Bool, 0, 0, 1}, \
    {0x2200 + (i), 1, &stConfig.stAnalogInput[i].stSwitch.bEnabled, &stConfigTemp.stAnalogInput[i].stSwitch.bEnabled, ParamType::Bool, 0, 0, 1}, \
    {0x2200 + (i), 2, &stConfig.stAnalogInput[i].stSwitch.eMode, &stConfigTemp.stAnalogInput[i].stSwitch.eMode, ParamType::Enum, static_cast<uint32_t>(InputMode::Momentary), 0, 1}, \
    {0x2200 + (i), 3, &stConfig.stAnalogInput[i].stSwitch.bInvert, &stConfigTemp.stAnalogInput[i].stSwitch.bInvert, ParamType::Bool, 0, 0, 1}, \
    {0x2200 + (i), 4, &stConfig.stAnalogInput[i].stSwitch.nThreshold, &stConfigTemp.stAnalogInput[i].stSwitch.nThreshold, ParamType::UInt16, 2000, 0, 5000}, \
    {0x2200 + (i), 5, &stConfig.stAnalogInput[i].stRotary.bEnabled, &stConfigTemp.stAnalogInput[i].stRotary.bEnabled, ParamType::Bool, 0, 0, 1}, \
    {0x2200 + (i), 6, &stConfig.stAnalogInput[i].stRotary.bInvert, &stConfigTemp.stAnalogInput[i].stRotary.bInvert, ParamType::Bool, 0, 0, 1}, \
    {0x2200 + (i), 7, &stConfig.stAnalogInput[i].stRotary.fOffset, &stConfigTemp.stAnalogInput[i].stRotary.fOffset, ParamType::Float, F(0.0f), F(-1e9f), F(1e9f)}, \
    {0x2200 + (i), 8, &stConfig.stAnalogInput[i].stRotary.fStep, &stConfigTemp.stAnalogInput[i].stRotary.fStep, ParamType::Float, F(100.0f), F(1e-6f), F(1e9f)}, \
    {0x2200 + (i), 9, &stConfig.stAnalogInput[i].stRotary.fMaxPos, &stConfigTemp.stAnalogInput[i].stRotary.fMaxPos, ParamType::Float, F(10.0f), F(0.0f), F(1e9f)}
