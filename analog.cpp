#include "analog.h"
#include "ch.hpp"
#include "hal.h"
#include "port.h"

adcsample_t adc1_samples[ADC1_NUM_CHANNELS] = {0};
adcsample_t adc2_samples[ADC2_NUM_CHANNELS] = {0};

static const ADCConversionGroup adc1_cfg = {
    .circular = false,
    .num_channels = ADC1_NUM_CHANNELS,
    .end_cb = NULL,
    .error_cb = NULL,
    .cfgr = ADC_CFGR_OVRMOD | ADC_CFGR_RES_12BITS, // CFGR: Configuration register
    .tr1 = ADC_TR_DISABLED,                                        // TR1: Watchdog threshold 1 register
    .tr2 = ADC_TR_DISABLED,                                        // TR2: Watchdog threshold 2 register
    .tr3 = ADC_TR_DISABLED,                                        // TR3: Watchdog threshold 3 register
    .awd2cr = 0,                                                   // AWD2CR: Analog watchdog 2 configuration register
    .awd3cr = 0,                                                   // AWD3CR: Analog watchdog 3 configuration register
    .smpr = {ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_601P5) |
            ADC_SMPR1_SMP_AN2(ADC_SMPR_SMP_601P5) |
            ADC_SMPR1_SMP_AN3(ADC_SMPR_SMP_601P5) |
            ADC_SMPR1_SMP_AN4(ADC_SMPR_SMP_601P5),
            ADC_SMPR2_SMP_AN16(ADC_SMPR_SMP_601P5)},              // SMPR: Sampling time register
    .sqr = {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1) |                      // SQR: Regular sequence register
            ADC_SQR1_SQ2_N(ADC_CHANNEL_IN2) |
            ADC_SQR1_SQ3_N(ADC_CHANNEL_IN3) |
            ADC_SQR1_SQ4_N(ADC_CHANNEL_IN4),
            ADC_SQR2_SQ5_N(ADC_CHANNEL_IN16),
            0,
            0}};

static const ADCConversionGroup adc2_cfg = {
    .circular = false,
    .num_channels = ADC2_NUM_CHANNELS,
    .end_cb = NULL,
    .error_cb = NULL,
    .cfgr = ADC_CFGR_OVRMOD | ADC_CFGR_RES_12BITS, // CFGR: Configuration register
    .tr1 = ADC_TR_DISABLED,                                        // TR1: Watchdog threshold 1 register
    .tr2 = ADC_TR_DISABLED,                                        // TR2: Watchdog threshold 2 register
    .tr3 = ADC_TR_DISABLED,                                        // TR3: Watchdog threshold 3 register
    .awd2cr = 0,                                                   // AWD2CR: Analog watchdog 2 configuration register
    .awd3cr = 0,                                                   // AWD3CR: Analog watchdog 3 configuration register
    .smpr = {ADC_SMPR1_SMP_AN1(ADC_SMPR_SMP_601P5),
             0}, // SMPR: Sampling time register
    .sqr = {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN1),
            0,
            0,
            0} // SQR: Regular sequence register
};

struct AdcThread : chibios_rt::BaseStaticThread<256>
{
    void main()
    {
        while (true)
        {
            adcConvert(&ADCD1, &adc1_cfg, adc1_samples, ADC1_BUF_DEPTH);
            adcConvert(&ADCD2, &adc2_cfg, adc2_samples, ADC2_BUF_DEPTH);

            chThdSleepMilliseconds(10);
        }
    }
};

static AdcThread adcThread;

void InitAdc()
{
    adcStart(&ADCD1, NULL);
    adcSTM32EnableTS(&ADCD1); // Enable temperature sensor

    adcStart(&ADCD2, NULL);

    adcThread.start(NORMALPRIO);
}

static adcsample_t GetAdcRaw(AnalogChannel channel)
{
    switch (channel)
    {
    case AnIn1:
        return adc1_samples[0];
    case AnIn2:
        return adc1_samples[1];
    case AnIn3:
        return adc1_samples[2];
    case AnIn4:
        return adc1_samples[3];
    case AnIn5:
        return adc2_samples[0];
    case TempSensor:
        return adc1_samples[4];
    default:
        return 0; // Invalid channel
    }
}

static float AdcToVolts(adcsample_t raw)
{
    // MCU vRef = 3.3v
    // 4095 counts full scale
    float mcuVolts = (3.3 / 4095) * raw;

    const float rUpper = 4700;
    const float rLower = 10000;

    return mcuVolts * ((rUpper + rLower) / rLower);
}

float GetAdcVolts(AnalogChannel channel)
{
    return AdcToVolts(GetAdcRaw(channel));
}

uint16_t GetTemperature()
{
    return (uint16_t)(30.0 + ((float)(GetAdcRaw(TempSensor) - STM32_TEMP_3V3_30C) / (STM32_TEMP_3V3_110C - STM32_TEMP_3V3_30C)) * (110.0 - 30.0));
}