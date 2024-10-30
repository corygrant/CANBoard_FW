#pragma once

#include "ch.h"
#include "hal.h"

#define ADC1_NUM_CHANNELS   5
#define ADC1_BUF_DEPTH      1

#define ADC2_NUM_CHANNELS   1
#define ADC2_BUF_DEPTH      1

enum AnalogChannel
{
    AnIn1 = 0,
    AnIn2,
    AnIn3,
    AnIn4,
    AnIn5,
    TempSensor
};

void InitAdc();
adcsample_t GetAdcRaw(AnalogChannel channel);
uint16_t GetTemperature();