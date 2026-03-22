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
    {0x0000, 4, &stConfig.stDevConfig.bSleepEnabled,      &stConfigTemp.stDevConfig.bSleepEnabled,     ParamType::Bool,   0, 0, 1}, \
    {0x0000, 5, &stConfig.stDevConfig.bCanFilterEnabled,  &stConfigTemp.stDevConfig.bCanFilterEnabled, ParamType::Bool,   0, 0, 1}, \
    {0x0000, 6, &stConfig.stDevConfig.bConnectUsbToCan,   &stConfigTemp.stDevConfig.bConnectUsbToCan,  ParamType::Bool,   1, 0, 1}

//=============================================================================
// Output Parameters - Base 0x1000
//=============================================================================
#define OUTPUT_PARAMS(i) \
    {0x1000 + (i), 0,  &stConfig.stOutput[i].bEnabled,                   &stConfigTemp.stOutput[i].bEnabled,                   ParamType::Bool,   0, 0, 1}, \
    {0x1000 + (i), 1,  &stConfig.stOutput[i].nInput,                     &stConfigTemp.stOutput[i].nInput,                     ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1000 + (i), 2,  &stConfig.stOutput[i].fCurrentLimit,              &stConfigTemp.stOutput[i].fCurrentLimit,              ParamType::Float,  F(20.0f), F(0.0f), F(100.0f)}, \
    {0x1000 + (i), 3,  &stConfig.stOutput[i].fInrushLimit,               &stConfigTemp.stOutput[i].fInrushLimit,               ParamType::Float,  F(50.0f), F(0.0f), F(100.0f)}, \
    {0x1000 + (i), 4,  &stConfig.stOutput[i].nInrushTime,                &stConfigTemp.stOutput[i].nInrushTime,                ParamType::UInt16, 1000, 0, 10000}, \
    {0x1000 + (i), 5,  &stConfig.stOutput[i].eResetMode,                 &stConfigTemp.stOutput[i].eResetMode,                 ParamType::Enum,   static_cast<uint32_t>(ProfetResetMode::None), 0, 2}, \
    {0x1000 + (i), 6,  &stConfig.stOutput[i].nResetTime,                 &stConfigTemp.stOutput[i].nResetTime,                 ParamType::UInt16, 1000, 0, 60000}, \
    {0x1000 + (i), 7,  &stConfig.stOutput[i].nResetLimit,                &stConfigTemp.stOutput[i].nResetLimit,                ParamType::UInt8,  3, 0, 20}, \
    {0x1000 + (i), 8,  &stConfig.stOutput[i].stPwm.bEnabled,             &stConfigTemp.stOutput[i].stPwm.bEnabled,             ParamType::Bool,   0, 0, 1}, \
    {0x1000 + (i), 9,  &stConfig.stOutput[i].stPwm.bSoftStart,           &stConfigTemp.stOutput[i].stPwm.bSoftStart,           ParamType::Bool,   0, 0, 1}, \
    {0x1000 + (i), 10, &stConfig.stOutput[i].stPwm.bVariableDutyCycle,   &stConfigTemp.stOutput[i].stPwm.bVariableDutyCycle,   ParamType::Bool, 0, 0, 1}, \
    {0x1000 + (i), 11, &stConfig.stOutput[i].stPwm.nDutyCycleInput,      &stConfigTemp.stOutput[i].stPwm.nDutyCycleInput,      ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1000 + (i), 12, &stConfig.stOutput[i].stPwm.nFixedDutyCycle,      &stConfigTemp.stOutput[i].stPwm.nFixedDutyCycle,      ParamType::UInt8,  100, 0, 100}, \
    {0x1000 + (i), 13, &stConfig.stOutput[i].stPwm.nFreq,                &stConfigTemp.stOutput[i].stPwm.nFreq,                ParamType::UInt16, 100, 0, 400}, \
    {0x1000 + (i), 14, &stConfig.stOutput[i].stPwm.nSoftStartRampTime,   &stConfigTemp.stOutput[i].stPwm.nSoftStartRampTime,   ParamType::UInt16, 0, 0, 10000}, \
    {0x1000 + (i), 15, &stConfig.stOutput[i].stPwm.nDutyCycleInputDenom, &stConfigTemp.stOutput[i].stPwm.nDutyCycleInputDenom, ParamType::UInt16, 100, 1, 5000}, \
    {0x1000 + (i), 16, &stConfig.stOutput[i].nPrimaryOutput,             &stConfigTemp.stOutput[i].nPrimaryOutput,             ParamType::Int8,  I8(-1), I8(-1), PDM_NUM_OUTPUTS - 1}

//=============================================================================
// Digital Input Parameters - Base 0x1200
//=============================================================================
#define DIGITAL_INPUT_PARAMS(i) \
    {0x1200 + (i), 0, &stConfig.stInput[i].bEnabled,      &stConfigTemp.stInput[i].bEnabled,     ParamType::Bool,   0, 0, 1}, \
    {0x1200 + (i), 1, &stConfig.stInput[i].eMode,         &stConfigTemp.stInput[i].eMode,        ParamType::Enum,   static_cast<uint32_t>(InputMode::Momentary), 0, 1}, \
    {0x1200 + (i), 2, &stConfig.stInput[i].bInvert,       &stConfigTemp.stInput[i].bInvert,      ParamType::Bool,   0, 0, 1}, \
    {0x1200 + (i), 3, &stConfig.stInput[i].nDebounceTime, &stConfigTemp.stInput[i].nDebounceTime,ParamType::UInt16, 20, 0, 1000}, \
    {0x1200 + (i), 4, &stConfig.stInput[i].ePull,         &stConfigTemp.stInput[i].ePull,        ParamType::Enum,   static_cast<uint32_t>(InputPull::None), 0, 2}
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
    {0x1400 + (i), 2,  &stConfig.stVirtualInput[i].nVar0,    &stConfigTemp.stVirtualInput[i].nVar0,    ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1400 + (i), 3,  &stConfig.stVirtualInput[i].eCond0,   &stConfigTemp.stVirtualInput[i].eCond0,   ParamType::Enum,   static_cast<uint32_t>(BoolOperator::And), 0, 2}, \
    {0x1400 + (i), 4,  &stConfig.stVirtualInput[i].bNot1,    &stConfigTemp.stVirtualInput[i].bNot1,    ParamType::Bool,   0, 0, 1}, \
    {0x1400 + (i), 5,  &stConfig.stVirtualInput[i].nVar1,    &stConfigTemp.stVirtualInput[i].nVar1,    ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1400 + (i), 6,  &stConfig.stVirtualInput[i].eCond1,   &stConfigTemp.stVirtualInput[i].eCond1,   ParamType::Enum,   static_cast<uint32_t>(BoolOperator::And), 0, 2}, \
    {0x1400 + (i), 7,  &stConfig.stVirtualInput[i].bNot2,    &stConfigTemp.stVirtualInput[i].bNot2,    ParamType::Bool,   0, 0, 1}, \
    {0x1400 + (i), 8,  &stConfig.stVirtualInput[i].nVar2,    &stConfigTemp.stVirtualInput[i].nVar2,    ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1400 + (i), 9,  &stConfig.stVirtualInput[i].eMode,    &stConfigTemp.stVirtualInput[i].eMode,    ParamType::Enum,   static_cast<uint32_t>(InputMode::Momentary), 0, 1}

//=============================================================================
// Condition Parameters - Base 0x1500
//=============================================================================
#define CONDITION_PARAMS(i) \
    {0x1500 + (i), 0, &stConfig.stCondition[i].bEnabled,  &stConfigTemp.stCondition[i].bEnabled,  ParamType::Bool,   0, 0, 1}, \
    {0x1500 + (i), 1, &stConfig.stCondition[i].nInput,    &stConfigTemp.stCondition[i].nInput,    ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1500 + (i), 2, &stConfig.stCondition[i].eOperator, &stConfigTemp.stCondition[i].eOperator, ParamType::Enum,   static_cast<uint32_t>(Operator::Equal), 0, 7}, \
    {0x1500 + (i), 3, &stConfig.stCondition[i].fArg,      &stConfigTemp.stCondition[i].fArg,      ParamType::Float,  F(0.0f), F(-1e9f), F(1e9f)}

//=============================================================================
// Counter Parameters - Base 0x1600
//=============================================================================
#define COUNTER_PARAMS(i) \
    {0x1600 + (i), 0,  &stConfig.stCounter[i].bEnabled,    &stConfigTemp.stCounter[i].bEnabled,    ParamType::Bool,   0, 0, 1}, \
    {0x1600 + (i), 1,  &stConfig.stCounter[i].nIncInput,   &stConfigTemp.stCounter[i].nIncInput,   ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1600 + (i), 2,  &stConfig.stCounter[i].nDecInput,   &stConfigTemp.stCounter[i].nDecInput,   ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1600 + (i), 3,  &stConfig.stCounter[i].nResetInput, &stConfigTemp.stCounter[i].nResetInput, ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
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
    {0x1700 + (i), 1, &stConfig.stFlasher[i].nInput,        &stConfigTemp.stFlasher[i].nInput,        ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1700 + (i), 2, &stConfig.stFlasher[i].nFlashOnTime,  &stConfigTemp.stFlasher[i].nFlashOnTime,  ParamType::UInt16, 500, 0, 5000}, \
    {0x1700 + (i), 3, &stConfig.stFlasher[i].nFlashOffTime, &stConfigTemp.stFlasher[i].nFlashOffTime, ParamType::UInt16, 500, 0, 5000}, \
    {0x1700 + (i), 4, &stConfig.stFlasher[i].bSingleCycle,  &stConfigTemp.stFlasher[i].bSingleCycle,  ParamType::Bool,   0, 0, 1}

//=============================================================================
// Starter Parameters - Base 0x1800 (single instance)
//=============================================================================
#define STARTER_PARAMS() \
    {0x1800, 0, &stConfig.stStarter.bEnabled, &stConfigTemp.stStarter.bEnabled, ParamType::Bool,   0, 0, 1}, \
    {0x1800, 1, &stConfig.stStarter.nInput,   &stConfigTemp.stStarter.nInput,   ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}

// Starter disable output param - subindex starts at 2
#define STARTER_DISABLE_PARAM(i) \
    {0x1800, 2 + (i), &stConfig.stStarter.bDisableOut[i], &stConfigTemp.stStarter.bDisableOut[i], ParamType::Bool, 0, 0, 1}

//=============================================================================
// Wiper Parameters - Base 0x1900 (single instance)
//=============================================================================
#define WIPER_PARAMS() \
    {0x1900, 0,  &stConfig.stWiper.bEnabled,       &stConfigTemp.stWiper.bEnabled,       ParamType::Bool,   0, 0, 1}, \
    {0x1900, 1,  &stConfig.stWiper.eMode,          &stConfigTemp.stWiper.eMode,          ParamType::Enum,   static_cast<uint32_t>(WiperMode::DigIn), 0, 2}, \
    {0x1900, 2,  &stConfig.stWiper.nSlowInput,     &stConfigTemp.stWiper.nSlowInput,     ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1900, 3,  &stConfig.stWiper.nFastInput,     &stConfigTemp.stWiper.nFastInput,     ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1900, 4,  &stConfig.stWiper.nInterInput,    &stConfigTemp.stWiper.nInterInput,    ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1900, 5,  &stConfig.stWiper.nOnInput,       &stConfigTemp.stWiper.nOnInput,       ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1900, 6,  &stConfig.stWiper.nSpeedInput,    &stConfigTemp.stWiper.nSpeedInput,    ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1900, 7,  &stConfig.stWiper.nParkInput,     &stConfigTemp.stWiper.nParkInput,     ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1900, 8,  &stConfig.stWiper.bParkStopLevel, &stConfigTemp.stWiper.bParkStopLevel, ParamType::Bool,   0, 0, 1}, \
    {0x1900, 9,  &stConfig.stWiper.nSwipeInput,    &stConfigTemp.stWiper.nSwipeInput,    ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1900, 10, &stConfig.stWiper.nWashInput,     &stConfigTemp.stWiper.nWashInput,     ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x1900, 11, &stConfig.stWiper.nWashWipeCycles,&stConfigTemp.stWiper.nWashWipeCycles, ParamType::UInt8, 3, 0, 10}

// Wiper speed map array
#define WIPER_SPEED_MAP_PARAMS() \
    {0x1900, 12, &stConfig.stWiper.eSpeedMap[0], &stConfigTemp.stWiper.eSpeedMap[0], ParamType::Enum, static_cast<uint32_t>(WiperSpeed::Intermittent1), 0, 8}, \
    {0x1900, 13, &stConfig.stWiper.eSpeedMap[1], &stConfigTemp.stWiper.eSpeedMap[1], ParamType::Enum, static_cast<uint32_t>(WiperSpeed::Intermittent2), 0, 8}, \
    {0x1900, 14, &stConfig.stWiper.eSpeedMap[2], &stConfigTemp.stWiper.eSpeedMap[2], ParamType::Enum, static_cast<uint32_t>(WiperSpeed::Intermittent3), 0, 8}, \
    {0x1900, 15, &stConfig.stWiper.eSpeedMap[3], &stConfigTemp.stWiper.eSpeedMap[3], ParamType::Enum, static_cast<uint32_t>(WiperSpeed::Intermittent4), 0, 8}, \
    {0x1900, 16, &stConfig.stWiper.eSpeedMap[4], &stConfigTemp.stWiper.eSpeedMap[4], ParamType::Enum, static_cast<uint32_t>(WiperSpeed::Intermittent5), 0, 8}, \
    {0x1900, 17, &stConfig.stWiper.eSpeedMap[5], &stConfigTemp.stWiper.eSpeedMap[5], ParamType::Enum, static_cast<uint32_t>(WiperSpeed::Intermittent6), 0, 8}, \
    {0x1900, 18, &stConfig.stWiper.eSpeedMap[6], &stConfigTemp.stWiper.eSpeedMap[6], ParamType::Enum, static_cast<uint32_t>(WiperSpeed::Slow), 0, 8}, \
    {0x1900, 19, &stConfig.stWiper.eSpeedMap[7], &stConfigTemp.stWiper.eSpeedMap[7], ParamType::Enum, static_cast<uint32_t>(WiperSpeed::Fast), 0, 8}

// Wiper intermittent delay array
#define WIPER_INTERMIT_PARAMS() \
    {0x1900, 20, &stConfig.stWiper.nIntermitTime[0], &stConfigTemp.stWiper.nIntermitTime[0],ParamType::UInt16, 1000, 0, 30000}, \
    {0x1900, 21, &stConfig.stWiper.nIntermitTime[1], &stConfigTemp.stWiper.nIntermitTime[1],ParamType::UInt16, 2000, 0, 30000}, \
    {0x1900, 22, &stConfig.stWiper.nIntermitTime[2], &stConfigTemp.stWiper.nIntermitTime[2],ParamType::UInt16, 3000, 0, 30000}, \
    {0x1900, 23, &stConfig.stWiper.nIntermitTime[3], &stConfigTemp.stWiper.nIntermitTime[3],ParamType::UInt16, 4000, 0, 30000}, \
    {0x1900, 24, &stConfig.stWiper.nIntermitTime[4], &stConfigTemp.stWiper.nIntermitTime[4],ParamType::UInt16, 5000, 0, 30000}, \
    {0x1900, 25, &stConfig.stWiper.nIntermitTime[5], &stConfigTemp.stWiper.nIntermitTime[5],ParamType::UInt16, 6000, 0, 30000}

//=============================================================================
// CAN Output Parameters - Base 0x2000
//=============================================================================
#define CAN_OUTPUT_PARAMS(i) \
    {0x2000 + (i), 0,  &stConfig.stCanOutput[i].bEnabled,   &stConfigTemp.stCanOutput[i].bEnabled,   ParamType::Bool,   0, 0, 1}, \
    {0x2000 + (i), 1,  &stConfig.stCanOutput[i].nInput,     &stConfigTemp.stCanOutput[i].nInput,     ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
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
// Keypad Parameters - Base 0x3000
//=============================================================================
#define KEYPAD_BASE_PARAMS(i) \
    {0x3000 + (i), 0,  &stConfig.stKeypad[i].bEnabled,                &stConfigTemp.stKeypad[i].bEnabled,                 ParamType::Bool,   0, 0, 1}, \
    {0x3000 + (i), 1,  &stConfig.stKeypad[i].nNodeId,                 &stConfigTemp.stKeypad[i].nNodeId,                  ParamType::UInt8,  0, 0, 127}, \
    {0x3000 + (i), 2,  &stConfig.stKeypad[i].bTimeoutEnabled,         &stConfigTemp.stKeypad[i].bTimeoutEnabled,          ParamType::Bool,   0, 0, 1}, \
    {0x3000 + (i), 3,  &stConfig.stKeypad[i].nTimeout,                &stConfigTemp.stKeypad[i].nTimeout,                 ParamType::UInt16, 0, 0, 60000}, \
    {0x3000 + (i), 4,  &stConfig.stKeypad[i].eModel,                  &stConfigTemp.stKeypad[i].eModel,                   ParamType::Enum,   static_cast<uint8_t>(KeypadModel::Blink12Key), 0, 13}, \
    {0x3000 + (i), 5,  &stConfig.stKeypad[i].nBacklightBrightness,    &stConfigTemp.stKeypad[i].nBacklightBrightness,     ParamType::UInt8,  63, 0, 63}, \
    {0x3000 + (i), 6,  &stConfig.stKeypad[i].nDimBacklightBrightness, &stConfigTemp.stKeypad[i].nDimBacklightBrightness,  ParamType::UInt8,  32, 0, 63}, \
    {0x3000 + (i), 7,  &stConfig.stKeypad[i].nBacklightColor,         &stConfigTemp.stKeypad[i].nBacklightColor,          ParamType::UInt8,  0, 0, 9}, \
    {0x3000 + (i), 8,  &stConfig.stKeypad[i].nDimmingVar,             &stConfigTemp.stKeypad[i].nDimmingVar,              ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x3000 + (i), 9,  &stConfig.stKeypad[i].nButtonBrightness,       &stConfigTemp.stKeypad[i].nButtonBrightness,        ParamType::UInt8,  63, 0, 63}, \
    {0x3000 + (i), 10, &stConfig.stKeypad[i].nDimButtonBrightness,    &stConfigTemp.stKeypad[i].nDimButtonBrightness,     ParamType::UInt8,  32, 0, 63}

//=============================================================================
// Keypad Button Parameters - Base 0x3100 + (keypad * 32) + button
//=============================================================================
#define KEYPAD_BUTTON_PARAMS(k, b) \
    {0x3100 + (k) * 32 + (b), 0,  &stConfig.stKeypad[k].stButton[b].bEnabled,              &stConfigTemp.stKeypad[k].stButton[b].bEnabled,        ParamType::Bool,   0, 0, 1}, \
    {0x3100 + (k) * 32 + (b), 1,  &stConfig.stKeypad[k].stButton[b].eMode,                 &stConfigTemp.stKeypad[k].stButton[b].eMode,           ParamType::Enum,   static_cast<uint32_t>(InputMode::Momentary), 0, 1}, \
    {0x3100 + (k) * 32 + (b), 2,  &stConfig.stKeypad[k].stButton[b].nColors[0],            &stConfigTemp.stKeypad[k].stButton[b].nColors[0],      ParamType::UInt8,  0, 0, 7}, \
    {0x3100 + (k) * 32 + (b), 3,  &stConfig.stKeypad[k].stButton[b].nColors[1],            &stConfigTemp.stKeypad[k].stButton[b].nColors[1],      ParamType::UInt8,  0, 0, 7}, \
    {0x3100 + (k) * 32 + (b), 4,  &stConfig.stKeypad[k].stButton[b].nColors[2],            &stConfigTemp.stKeypad[k].stButton[b].nColors[2],      ParamType::UInt8,  0, 0, 7}, \
    {0x3100 + (k) * 32 + (b), 5,  &stConfig.stKeypad[k].stButton[b].nColors[3],            &stConfigTemp.stKeypad[k].stButton[b].nColors[3],      ParamType::UInt8,  0, 0, 7}, \
    {0x3100 + (k) * 32 + (b), 6,  &stConfig.stKeypad[k].stButton[b].nFaultColor,           &stConfigTemp.stKeypad[k].stButton[b].nFaultColor,     ParamType::UInt8,  0, 0, 7}, \
    {0x3100 + (k) * 32 + (b), 7,  &stConfig.stKeypad[k].stButton[b].nVars[0],              &stConfigTemp.stKeypad[k].stButton[b].nVars[0],        ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x3100 + (k) * 32 + (b), 8,  &stConfig.stKeypad[k].stButton[b].nVars[1],              &stConfigTemp.stKeypad[k].stButton[b].nVars[1],        ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x3100 + (k) * 32 + (b), 9,  &stConfig.stKeypad[k].stButton[b].nVars[2],              &stConfigTemp.stKeypad[k].stButton[b].nVars[2],        ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x3100 + (k) * 32 + (b), 10, &stConfig.stKeypad[k].stButton[b].nVars[3],              &stConfigTemp.stKeypad[k].stButton[b].nVars[3],        ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x3100 + (k) * 32 + (b), 11, &stConfig.stKeypad[k].stButton[b].nFaultVar,             &stConfigTemp.stKeypad[k].stButton[b].nFaultVar,       ParamType::UInt16, 0, 0, PDM_VAR_MAP_SIZE - 1}, \
    {0x3100 + (k) * 32 + (b), 12, &stConfig.stKeypad[k].stButton[b].bBlink[0],             &stConfigTemp.stKeypad[k].stButton[b].bBlink[0],       ParamType::Bool,   0, 0, 1}, \
    {0x3100 + (k) * 32 + (b), 13, &stConfig.stKeypad[k].stButton[b].bBlink[1],             &stConfigTemp.stKeypad[k].stButton[b].bBlink[1],       ParamType::Bool,   0, 0, 1}, \
    {0x3100 + (k) * 32 + (b), 14, &stConfig.stKeypad[k].stButton[b].bBlink[2],             &stConfigTemp.stKeypad[k].stButton[b].bBlink[2],       ParamType::Bool,   0, 0, 1}, \
    {0x3100 + (k) * 32 + (b), 15, &stConfig.stKeypad[k].stButton[b].bBlink[3],             &stConfigTemp.stKeypad[k].stButton[b].bBlink[3],       ParamType::Bool,   0, 0, 1}, \
    {0x3100 + (k) * 32 + (b), 16, &stConfig.stKeypad[k].stButton[b].bFaultBlink,           &stConfigTemp.stKeypad[k].stButton[b].bFaultBlink,     ParamType::Bool,   0, 0, 1}, \
    {0x3100 + (k) * 32 + (b), 17, &stConfig.stKeypad[k].stButton[b].nBlinkColors[0],       &stConfigTemp.stKeypad[k].stButton[b].nBlinkColors[0], ParamType::UInt8,  0, 0, 7}, \
    {0x3100 + (k) * 32 + (b), 18, &stConfig.stKeypad[k].stButton[b].nBlinkColors[1],       &stConfigTemp.stKeypad[k].stButton[b].nBlinkColors[1], ParamType::UInt8,  0, 0, 7}, \
    {0x3100 + (k) * 32 + (b), 19, &stConfig.stKeypad[k].stButton[b].nBlinkColors[2],       &stConfigTemp.stKeypad[k].stButton[b].nBlinkColors[2], ParamType::UInt8,  0, 0, 7}, \
    {0x3100 + (k) * 32 + (b), 20, &stConfig.stKeypad[k].stButton[b].nBlinkColors[3],       &stConfigTemp.stKeypad[k].stButton[b].nBlinkColors[3], ParamType::UInt8,  0, 0, 7}, \
    {0x3100 + (k) * 32 + (b), 21, &stConfig.stKeypad[k].stButton[b].nFaultBlinkColor,      &stConfigTemp.stKeypad[k].stButton[b].nFaultBlinkColor,ParamType::UInt8,  0, 0, 7}

//=============================================================================
// Keypad Dial Parameters - Base 0x3200 + (keypad * 4) + dial
//=============================================================================
#define KEYPAD_DIAL_PARAMS(k, d) \
    {0x3200 + (k) * 4 + (d), 0, &stConfig.stKeypad[k].stDial[d].bEnabled,     &stConfigTemp.stKeypad[k].stDial[d].bEnabled,     ParamType::Bool,   0, 0, 1},  \
    {0x3200 + (k) * 4 + (d), 1, &stConfig.stKeypad[k].stDial[d].nMinCount,    &stConfigTemp.stKeypad[k].stDial[d].nMinCount,    ParamType::UInt8, 0, 0, 16},  \
    {0x3200 + (k) * 4 + (d), 2, &stConfig.stKeypad[k].stDial[d].nMaxCount,    &stConfigTemp.stKeypad[k].stDial[d].nMaxCount,    ParamType::UInt8, 16, 0, 16}, \
    {0x3200 + (k) * 4 + (d), 3, &stConfig.stKeypad[k].stDial[d].nLedOffset,   &stConfigTemp.stKeypad[k].stDial[d].nLedOffset,   ParamType::UInt8, 0, 0, 16}
