#pragma once

#include <cstdint>
#include "port.h"
#include "enums.h"
#include "dbc.h"

#define CONFIG_VERSION 0x0001 //Increment when config structure changes

struct Config_DeviceConfig{
  uint16_t nConfigVersion;
  uint16_t nBaseId;
  uint16_t nParamRxId; //Config->Pdm
  uint16_t nParamTxId; //Pdm->Config
  CanBitrate eCanSpeed;
  bool bCanFilterEnabled;
};

struct Config_Input{
  bool bEnabled;
  InputMode eMode;
  bool bInvert;
  uint16_t nDebounceTime; //ms
  InputPull ePull;
};

struct Config_Output{
  bool bEnabled;
  uint16_t nInput;
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

struct Config_Flasher{
  bool bEnabled;
  uint16_t nInput;
  uint16_t nFlashOnTime; //ms
  uint16_t nFlashOffTime; //ms
  bool bSingleCycle;
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

struct CanboardConfig{
  Config_DeviceConfig stDevConfig;
  Config_Input stInput[NUM_INPUTS];
  Config_VirtualInput stVirtualInput[NUM_VIRT_INPUTS];
  Config_Flasher stFlasher[NUM_FLASHERS];
  Config_CanInput stCanInput[NUM_CAN_INPUTS];
  Config_CanOutput stCanOutput[NUM_CAN_OUTPUTS];
  Config_Counter stCounter[NUM_COUNTERS];
  Config_Condition stCondition[NUM_CONDITIONS];
};

extern CanboardConfig stConfig;
extern CanboardConfig stConfigTemp; // Used for staging new config before applying

void InitConfig();
bool WriteConfig();