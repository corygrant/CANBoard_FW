#pragma once

#include "hal.h"

#define STM32_TEMP_3V3_30C  *((uint16_t*)0x1FFFF7B8)
#define STM32_TEMP_3V3_110C *((uint16_t*)0x1FFFF7C2)

#define NUM_OUTPUTS 4
#define NUM_INPUTS 2
#define NUM_VIRT_INPUTS 16
#define NUM_CAN_INPUTS 32
#define NUM_CAN_OUTPUTS 32
#define NUM_FLASHERS 4
#define NUM_COUNTERS 4
#define NUM_CONDITIONS 32

#define VAR_MAP_SYS_VARS 5

#define VAR_MAP_SIZE ( \
    VAR_MAP_SYS_VARS + \
    (NUM_INPUTS * 1) + \
    (NUM_CAN_INPUTS * 2) + \
    (NUM_VIRT_INPUTS * 1) + \
    (NUM_OUTPUTS * 4) + \
    (NUM_FLASHERS * 1) + \
    (NUM_CONDITIONS * 1) + \
    (NUM_COUNTERS * 1)\
)

#define NUM_TX_MSGS 3

#define ADC1_NUM_CHANNELS 8
#define ADC1_BUF_DEPTH 1

#define SYS_TIME TIME_I2MS(chVTGetSystemTimeX())

static const float ALWAYS_FALSE = 0.0f;
static const float ALWAYS_TRUE = 1.0f;
 
enum class AnalogChannel
{
    AI1 = 0,
    AI2,
    AI3,
    AI4,
    VRefInt
};

const CANConfig &GetCanConfig(CanBitrate bitrate);

msg_t InitAdc();
void DeInitAdc();
uint16_t GetAdcRaw(AnalogChannel channel);
float GetVDDA();