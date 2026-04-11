#pragma once

#include "hal.h"
#include "enums.h"

#define STM32_TEMP_3V3_30C  *((uint16_t*)0x1FFFF7B8)
#define STM32_TEMP_3V3_110C *((uint16_t*)0x1FFFF7C2)

#define NUM_DIG_OUTPUTS 4
#define NUM_DIG_INPUTS 8
#define NUM_ANALOG_INPUTS 5
#define NUM_VIRT_INPUTS 8
#define NUM_CAN_INPUTS 8
#define NUM_CAN_OUTPUTS 8
#define NUM_FLASHERS 4
#define NUM_COUNTERS 4
#define NUM_CONDITIONS 8

#define VAR_MAP_SYS_VARS 2

#define VAR_MAP_SIZE ( \
    VAR_MAP_SYS_VARS + \
    (NUM_DIG_INPUTS * 1) + \
    (NUM_ANALOG_INPUTS * 4) + \
    (NUM_CAN_INPUTS * 2) + \
    (NUM_VIRT_INPUTS * 1) + \
    (NUM_DIG_OUTPUTS * 1) + \
    (NUM_FLASHERS * 1) + \
    (NUM_CONDITIONS * 1) + \
    (NUM_COUNTERS * 1)\
)

#define NUM_TX_MSGS 3

#define ADC1_NUM_CHANNELS 5
#define ADC1_BUF_DEPTH 1

#define ADC2_NUM_CHANNELS   1
#define ADC2_BUF_DEPTH      1

#define SYS_TIME TIME_I2MS(chVTGetSystemTimeX())

static const float ALWAYS_FALSE = 0.0f;
static const float ALWAYS_TRUE = 1.0f;
 
enum class AnalogChannel
{
    AnIn1 = 0,
    AnIn2,
    AnIn3,
    AnIn4,
    AnIn5,
    TempSensor
};

const CANConfig &GetCanConfig(CanBitrate bitrate);

msg_t InitAdc();
uint16_t GetAdcRaw(AnalogChannel channel);
float GetAdcVolts(AnalogChannel channel);
uint16_t GetTemperature();