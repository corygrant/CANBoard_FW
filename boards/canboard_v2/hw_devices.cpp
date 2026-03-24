#include "hw_devices.h"

/*
DigIn1 = 0,
DigIn2 = 1,
DigIn3 = 2,
DigIn4 = 3,
DigIn5 = 4,
DigIn6 = 5,
DigIn7 = 6,
DigIn8 = 7,
IdSel1 = 8,
IdSel2 = 9
*/
Digital_Input in[NUM_INPUTS] = {
    Digital_Input(LINE_DI1),
    Digital_Input(LINE_DI2),
    Digital_Input(LINE_DI3),
    Digital_Input(LINE_DI4),
    Digital_Input(LINE_DI5),
    Digital_Input(LINE_DI6),
    Digital_Input(LINE_DI7),
    Digital_Input(LINE_DI8),
    Digital_Input(LINE_CAN_ID_1),
    Digital_Input(LINE_CAN_ID_2)};    

/*
DigOut1 = 0,
DigOut2 = 1,
DigOut3 = 2,
DigOut4 = 3
*/
Digital_Output out[NUM_OUTPUTS] = {
    Digital_Output(LINE_DO1),
    Digital_Output(LINE_DO2),
    Digital_Output(LINE_DO3),
    Digital_Output(LINE_DO4)};