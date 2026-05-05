#include "hw_devices.h"

Digital_Input digIn[NUM_DIG_INPUTS] = {
    Digital_Input(LINE_DI1),
    Digital_Input(LINE_DI2),
    Digital_Input(LINE_DI3),
    Digital_Input(LINE_DI4),
    Digital_Input(LINE_DI5),
    Digital_Input(LINE_DI6),
    Digital_Input(LINE_DI7),
    Digital_Input(LINE_DI8)};   

Digital_Output digOut[NUM_DIG_OUTPUTS] = {
    Digital_Output(LINE_DO1),
    Digital_Output(LINE_DO2),
    Digital_Output(LINE_DO3),
    Digital_Output(LINE_DO4)};

Analog_Input analogIn[NUM_ANALOG_INPUTS] = {
    Analog_Input(AnalogChannel::AnIn1),
    Analog_Input(AnalogChannel::AnIn2),
    Analog_Input(AnalogChannel::AnIn3),
    Analog_Input(AnalogChannel::AnIn4),
    Analog_Input(AnalogChannel::AnIn5)};