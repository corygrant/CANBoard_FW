#include "hw_devices.h"
#include "port_pwm.h"

#if PDM_TYPE == 0
Profet pf[PDM_NUM_OUTPUTS] = {
    Profet(1, ProfetModel::BTS7002_1EPP, LINE_PF1_IN, LINE_PF1_DEN, LINE_UNUSED, AnalogChannel::IS1, &PWMD3, &pwm3Cfg, PwmChannel::Ch1),
    Profet(2, ProfetModel::BTS7002_1EPP, LINE_PF2_IN, LINE_PF2_DEN, LINE_UNUSED, AnalogChannel::IS2, &PWMD4, &pwm4Cfg, PwmChannel::Ch1),
    Profet(3, ProfetModel::BTS7008_2EPA_CH1, LINE_PF3_IN, LINE_PF3_4_DEN, LINE_PF3_4_DSEL, AnalogChannel::IS3_4, &PWMD5, &pwm5Cfg, PwmChannel::Ch1),
    Profet(4, ProfetModel::BTS7008_2EPA_CH2, LINE_PF4_IN, LINE_PF3_4_DEN, LINE_PF3_4_DSEL, AnalogChannel::IS3_4, &PWMD9, &pwm9Cfg, PwmChannel::Ch1),
    Profet(5, ProfetModel::BTS7008_2EPA_CH1, LINE_PF5_IN, LINE_PF5_6_DEN, LINE_PF5_6_DSEL, AnalogChannel::IS5_6, &PWMD10, &pwm10Cfg, PwmChannel::Ch1),
    Profet(6, ProfetModel::BTS7008_2EPA_CH2, LINE_PF6_IN, LINE_PF5_6_DEN, LINE_PF5_6_DSEL, AnalogChannel::IS5_6, &PWMD11, &pwm11Cfg, PwmChannel::Ch1),
    Profet(7, ProfetModel::BTS7008_2EPA_CH1, LINE_PF7_IN, LINE_PF7_8_DEN, LINE_PF7_8_DSEL, AnalogChannel::IS7_8, &PWMD12, &pwm12Cfg, PwmChannel::Ch1),
    Profet(8, ProfetModel::BTS7008_2EPA_CH2, LINE_PF8_IN, LINE_PF7_8_DEN, LINE_PF7_8_DSEL, AnalogChannel::IS7_8, &PWMD13, &pwm13Cfg, PwmChannel::Ch1)};

Digital in[PDM_NUM_INPUTS] = {
    Digital(LINE_DI1),
    Digital(LINE_DI2)};    

Led statusLed = Led(LINE_LED_STATUS);
Led errorLed = Led(LINE_LED_ERROR);

MCP9808 tempSensor(I2CD1, MCP9808_I2CADDR_DEFAULT);

#endif