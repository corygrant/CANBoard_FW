#pragma once

#include <cstdint>
#include "port.h"
#include "enums.h"
#include "hardware/mb85rc.h"
#include "dbc.h"

#define CONFIG_VERSION 0x0006 //Increment when config structure changes

struct Config_DeviceConfig{
  uint16_t nConfigVersion;
  uint16_t nBaseId;
  uint16_t nParamRxId; //Config->Pdm
  uint16_t nParamTxId; //Pdm->Config
  CanBitrate eCanSpeed;
  bool bSleepEnabled;
  bool bCanFilterEnabled;
  bool bConnectUsbToCan;
};

struct Config_Input{
  bool bEnabled;
  InputMode eMode;
  bool bInvert;
  uint16_t nDebounceTime; //ms
  InputPull ePull;
};

struct Config_VirtualInput{
  bool bEnabled;
  bool bNot0;
  uint16_t nVar0;
  BoolOperator eCond0;
  bool bNot1;
  uint16_t nVar1;
  BoolOperator eCond1;
  bool bNot2;
  uint16_t nVar2;
  InputMode eMode;
};

struct Config_PwmOutput{
  bool bEnabled;
  bool bSoftStart;
  bool bVariableDutyCycle;
  uint16_t nDutyCycleInput;
  uint8_t nFixedDutyCycle;
  uint16_t nFreq;
  uint16_t nSoftStartRampTime; //ms
  uint16_t nDutyCycleInputDenom;
};

struct Config_Output{
  bool bEnabled;
  uint16_t nInput;
  float fCurrentLimit;
  float fInrushLimit;
  uint16_t nInrushTime; //ms
  ProfetResetMode eResetMode;
  uint16_t nResetTime; //ms
  uint8_t nResetLimit;
  int8_t nPrimaryOutput;  //index of primary to follow, -1 = unpaired

  Config_PwmOutput stPwm;
};

struct Config_Wiper{
  bool bEnabled;
  WiperMode eMode;
  uint16_t nSlowInput;   //WiperMode_DigIn
  uint16_t nFastInput;   //WiperMode_DigIn
  uint16_t nInterInput;  //WiperMode_DigIn
  uint16_t nOnInput;     //WiperMode_MixIn
  uint16_t nSpeedInput;  //WiperMode_IntIn and WiperMode_MixIn
  uint16_t nParkInput;
  bool bParkStopLevel;
  uint16_t nSwipeInput;
  uint16_t nWashInput;
  uint8_t nWashWipeCycles;
  WiperSpeed eSpeedMap[PDM_NUM_WIPER_SPEED_MAP];
  uint16_t nIntermitTime[PDM_NUM_WIPER_INTER_DELAYS]; //ms
};

struct Config_Flasher{
  bool bEnabled;
  uint16_t nInput;
  uint16_t nFlashOnTime; //ms
  uint16_t nFlashOffTime; //ms
  bool bSingleCycle;
};

struct Config_Starter{
  bool bEnabled;
  uint16_t nInput;
  bool bDisableOut[PDM_NUM_OUTPUTS];
};

struct Config_CanInput{
  bool bEnabled;
  bool bTimeoutEnabled;
  uint16_t nTimeout; //ms
  uint8_t nIDE; //0=STD, 1=EXT
  uint32_t nID;
  uint8_t nStartBit;   
  uint8_t nBitLength;  
  float fFactor;       
  float fOffset;       
  ByteOrder eByteOrder;
  bool bSigned;        
  Operator eOperator;
  float fOperand;       
  InputMode eMode;
};

struct Config_CanOutput{
  bool bEnabled;
  uint16_t nInput;
  uint8_t nIDE; //0=STD, 1=EXT
  uint32_t nID;
  uint8_t nStartBit;   
  uint8_t nBitLength;  
  float fFactor;       
  float fOffset;       
  ByteOrder eByteOrder;
  bool bSigned;
  uint16_t nInterval; //ms
};

struct Config_Counter{
  bool bEnabled;
  uint16_t nIncInput;
  uint16_t nDecInput;
  uint16_t nResetInput;
  uint8_t nMinCount;
  uint8_t nMaxCount;
  InputEdge eIncEdge;
  InputEdge eDecEdge;
  InputEdge eResetEdge;
  bool bWrapAround;
  bool bHoldToReset;
  uint16_t nResetTime; //ms
};

struct Config_Condition{
  bool bEnabled;
  uint16_t nInput;
  Operator eOperator;
  float fArg;
};

struct Config_KeypadButton{
  bool bEnabled;
  InputMode eMode;
  uint8_t nColors[4];
  uint8_t nFaultColor;
  uint16_t nVars[4];
  uint16_t nFaultVar;
  bool bBlink[4];
  bool bFaultBlink;
  uint8_t nBlinkColors[4];
  uint8_t nFaultBlinkColor;
};

struct Config_KeypadDial{
  bool bEnabled;
  uint8_t nMinCount;
  uint8_t nMaxCount;
  uint8_t nLedOffset;
};

struct Config_Keypad{
  bool bEnabled;
  uint8_t nNodeId;
  bool bTimeoutEnabled;
  uint16_t nTimeout; //ms
  KeypadModel eModel;
  uint8_t nBacklightBrightness;
  uint8_t nDimBacklightBrightness;
  uint8_t nBacklightColor;
  uint16_t nDimmingVar;
  uint8_t nButtonBrightness;
  uint8_t nDimButtonBrightness;
  Config_KeypadButton stButton[KEYPAD_MAX_BUTTONS];
  Config_KeypadDial stDial[KEYPAD_MAX_DIALS];
};

struct PdmConfig{
  Config_DeviceConfig stDevConfig;
  Config_Input stInput[PDM_NUM_INPUTS];
  Config_VirtualInput stVirtualInput[PDM_NUM_VIRT_INPUTS];
  Config_Output stOutput[PDM_NUM_OUTPUTS];
  Config_Wiper stWiper;
  Config_Flasher stFlasher[PDM_NUM_FLASHERS];
  Config_Starter stStarter;
  Config_CanInput stCanInput[PDM_NUM_CAN_INPUTS];
  Config_CanOutput stCanOutput[PDM_NUM_CAN_OUTPUTS];
  Config_Counter stCounter[PDM_NUM_COUNTERS];
  Config_Condition stCondition[PDM_NUM_CONDITIONS];
  Config_Keypad stKeypad[PDM_NUM_KEYPADS];
};

extern PdmConfig stConfig;
extern PdmConfig stConfigTemp; // Used for staging new config before applying

void InitConfig();
bool WriteConfig();