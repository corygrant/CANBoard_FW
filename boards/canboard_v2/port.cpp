#include "port.h"
#include "canboard_config.h"


static const CANConfig canConfig1000 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=2, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(1)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

static const CANConfig canConfig500 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=4, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(3)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

static const CANConfig canConfig250 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=8, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(7)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

static const CANConfig canConfig125 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=16, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(15)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

const CANConfig& GetCanConfig(CanBitrate bitrate) {
    switch(bitrate) {
        case CanBitrate::Bitrate_1000K:
            return canConfig1000;
        case CanBitrate::Bitrate_500K:
            return canConfig500;
        case CanBitrate::Bitrate_250K:
            return canConfig250;
        case CanBitrate::Bitrate_125K:
            return canConfig125;
        default:
            return canConfig500;
    }
    return canConfig500;
}

adcsample_t adc1_samples[ADC1_NUM_CHANNELS] = {0};
adcsample_t adc2_samples[ADC2_NUM_CHANNELS] = {0};

static const ADCConversionGroup adc1_cfg = {
    .circular = true,
    .num_channels = ADC1_NUM_CHANNELS,
    .end_cb = NULL,
    .error_cb = NULL,
    .cfgr = ADC_CFGR_OVRMOD | ADC_CFGR_RES_12BITS | ADC_CFGR_CONT, // CFGR: Configuration register
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
    .circular = true,
    .num_channels = ADC2_NUM_CHANNELS,
    .end_cb = NULL,
    .error_cb = NULL,
    .cfgr = ADC_CFGR_OVRMOD | ADC_CFGR_RES_12BITS | ADC_CFGR_CONT, // CFGR: Configuration register
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

msg_t InitAdc()
{
    msg_t ret;
    ret = adcStart(&ADCD1, NULL);
    if(ret != HAL_RET_SUCCESS)
        return ret;

    adcSTM32EnableTS(&ADCD1); // Enable temperature sensor

    adcStartConversion(&ADCD1, &adc1_cfg, adc1_samples, ADC1_BUF_DEPTH);

    ret = adcStart(&ADCD2, NULL);
    if(ret != HAL_RET_SUCCESS)
        return ret;

    adcStartConversion(&ADCD2, &adc2_cfg, adc2_samples, ADC2_BUF_DEPTH);

    return HAL_RET_SUCCESS;
}

adcsample_t GetAdcRaw(AnalogChannel channel)
{
    switch (channel)
    {
    case AnalogChannel::AnIn1:
        return adc1_samples[0];
    case AnalogChannel::AnIn2:
        return adc1_samples[1];
    case AnalogChannel::AnIn3:
        return adc1_samples[2];
    case AnalogChannel::AnIn4:
        return adc1_samples[3];
    case AnalogChannel::AnIn5:
        return adc2_samples[0];
    case AnalogChannel::TempSensor:
        return adc1_samples[4];
    default:
        return 0; // Invalid channel
    }
}

float AdcToVolts(adcsample_t raw)
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
    return (uint16_t)(30.0 + ((float)(GetAdcRaw(AnalogChannel::TempSensor) - STM32_TEMP_3V3_30C) / (STM32_TEMP_3V3_110C - STM32_TEMP_3V3_30C)) * (110.0 - 30.0));
}