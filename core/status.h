#pragma once

#include <cstdint>
#include "enums.h"
#include "dbc.h"

uint8_t GetCanOffset();
uint8_t GetHearbeat();

bool GetInputVal(uint8_t nInput);

float GetOutputCurrent(uint8_t nOutput);
bool GetOutputState(uint8_t nOutput);

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