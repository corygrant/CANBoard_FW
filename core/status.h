#pragma once

#include <cstdint>
#include "enums.h"
#include "dbc.h"

bool GetInputVal(uint8_t nInput);
bool GetOutputState(uint8_t nOutput);
uint16_t GetAnalogInputVal(uint8_t nInput);
float GetAnalogInputMv(uint8_t nInput);
uint8_t GetRotarySwitchPos(uint8_t nInput);
bool GetAnalogSwitchVal(uint8_t nInput);

bool GetAnyCanInEnable();
bool GetAnyVirtInEnable();
bool GetAnyFlasherEnable();
bool GetAnyCounterEnable();
bool GetAnyConditionEnable();

bool GetCanInEnable(uint8_t nInput);
bool GetCanInOutput(uint8_t nInput);
float GetCanInVal(uint8_t nInput);
float GetCanInFactor(uint8_t nInput);
float GetCanInOffset(uint8_t nInput);
ByteOrder GetCanInByteOrder(uint8_t nInput);
uint32_t GetCanInOutputs();

bool GetVirtInVal(uint8_t nInput);
uint32_t GetVirtIns();

bool GetFlasherVal(uint8_t nFlasher);

float GetCounterVal(uint8_t nCounter);

uint32_t GetConditions();