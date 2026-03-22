#include "msg.h"
#include "config.h"
#include "digital.h"
#include "status.h"


CANTxMsg TxMsg0()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 0 (Analog inputs 1-4 millivolts)
    //=======================================================
    stMsg.frame.IDE = CAN_IDE_STD;
    stMsg.frame.SID = CAN_BASE_ID + GetCanOffset() + 0;
    stMsg.frame.DLC = 8;
    stMsg.frame.data16[0] = (uint16_t)(GetAdcVolts(AnIn1) * 1000);
    stMsg.frame.data16[1] = (uint16_t)(GetAdcVolts(AnIn2) * 1000);
    stMsg.frame.data16[2] = (uint16_t)(GetAdcVolts(AnIn3) * 1000);
    stMsg.frame.data16[3] = (uint16_t)(GetAdcVolts(AnIn4) * 1000);

    stMsg.bSend = true; // Always send

    return stMsg;
}

CANTxMsg TxMsg1()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 1 (Analog input 5 millivolts and temperature)
    //=======================================================
    stMsg.frame.IDE = CAN_IDE_STD;
    stMsg.frame.SID = CAN_BASE_ID + GetCanOffset() + 1;
    stMsg.frame.DLC = 8;
    stMsg.frame.data16[0] = (uint16_t)(GetAdcVolts(AnIn5) * 1000);
    stMsg.frame.data16[1] = 0;
    stMsg.frame.data16[2] = 0;
    stMsg.frame.data16[3] = GetTemperature();

    stMsg.bSend = true; // Always send

    return stMsg;
}

CANTxMsg TxMsg2()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 2 (Rotary switches, dig inputs, analog input switches, low side output status, heartbeat)
    //=======================================================
    stMsg.frame.IDE = CAN_IDE_STD;
    stMsg.frame.SID = CAN_BASE_ID + GetCanOffset() + 2;
    stMsg.frame.DLC = 8;
    stMsg.frame.data8[0] = (GetRotarySwPos(RotarySw2) << 4) + GetRotarySwPos(RotarySw1);
    stMsg.frame.data8[1] = (GetRotarySwPos(RotarySw4) << 4) + GetRotarySwPos(RotarySw3);
    stMsg.frame.data8[2] = GetRotarySwPos(RotarySw5);
    stMsg.frame.data8[3] = 0; //Empty
    stMsg.frame.data8[4] = (GetDigIn(DigIn8) << 7) + (GetDigIn(DigIn7) << 6) + (GetDigIn(DigIn6) << 5) + (GetDigIn(DigIn5) << 4) + 
                        (GetDigIn(DigIn4) << 3) + (GetDigIn(DigIn3) << 2) + (GetDigIn(DigIn2) << 1) + GetDigIn(DigIn1);
    stMsg.frame.data8[5] = (GetAnSwitch(AnIn5) << 4) + (GetAnSwitch(AnIn4) << 3) + (GetAnSwitch(AnIn3) << 2) + (GetAnSwitch(AnIn2) << 1) + GetAnSwitch(AnIn1);
    stMsg.frame.data8[6] = (GetDigOut(DigOut4) << 3) + (GetDigOut(DigOut3) << 2) + (GetDigOut(DigOut2) << 1) + GetDigOut(DigOut1);;
    stMsg.frame.data8[7] = GetHearbeat();

    IncrementHeartbeat();

    stMsg.bSend = true; // Always send

    return stMsg;
}