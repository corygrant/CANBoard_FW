#include "msg.h"
#include "config.h"
#include "profet.h"
#include "digital.h"
#include "pdm.h"
#include "status.h"


CANTxMsg TxMsg0()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 0 (Digital inputs 1-2) and Device Status
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 0;
    stMsg.frame.DLC = 8; // Bytes to send
    stMsg.frame.data8[0] = (GetInputVal(1) << 1) + GetInputVal(0);
    stMsg.frame.data8[1] = static_cast<uint8_t>(GetPdmState()) + (PDM_TYPE << 4);
    stMsg.frame.data16[1] = (uint16_t)GetTotalCurrent();
    stMsg.frame.data16[2] = (uint16_t)(GetBattVolt() * 10.0);
    stMsg.frame.data16[3] = (uint16_t)(GetBoardTemp() * 10.0);

    stMsg.bSend = true; // Always send

    return stMsg;
}

CANTxMsg TxMsg1()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 1 (Out 1-4 Current)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 1;
    stMsg.frame.DLC = 8; // Bytes to send
    stMsg.frame.data16[0] = (uint16_t)GetOutputCurrent(0);
    stMsg.frame.data16[1] = (uint16_t)GetOutputCurrent(1);
    stMsg.frame.data16[2] = (uint16_t)GetOutputCurrent(2);
    stMsg.frame.data16[3] = (uint16_t)GetOutputCurrent(3);

    stMsg.bSend = true; // Always send

    return stMsg;
}

CANTxMsg TxMsg2()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 2 (Out 5-8 Current)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 2;
    stMsg.frame.DLC = 8; // Bytes to send
    stMsg.frame.data16[0] = (uint16_t)GetOutputCurrent(4);
    stMsg.frame.data16[1] = (uint16_t)GetOutputCurrent(5);
    stMsg.frame.data16[2] = (uint16_t)GetOutputCurrent(6);
    stMsg.frame.data16[3] = (uint16_t)GetOutputCurrent(7);

    stMsg.bSend = true; // Always send

    return stMsg;
}

CANTxMsg TxMsg3()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 3 (Out 1-8 Status)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 3;
    stMsg.frame.DLC = 8; // Bytes to send
    stMsg.frame.data8[0] = (static_cast<uint8_t>(GetOutputState(1)) << 4) + static_cast<uint8_t>(GetOutputState(0));
    stMsg.frame.data8[1] = (static_cast<uint8_t>(GetOutputState(3)) << 4) + static_cast<uint8_t>(GetOutputState(2));
    stMsg.frame.data8[2] = (static_cast<uint8_t>(GetOutputState(5)) << 4) + static_cast<uint8_t>(GetOutputState(4));
    stMsg.frame.data8[3] = (static_cast<uint8_t>(GetOutputState(7)) << 4) + static_cast<uint8_t>(GetOutputState(6));
    stMsg.frame.data8[4] = (GetWiperFastOut() << 1) + GetWiperSlowOut();
    stMsg.frame.data8[5] = (static_cast<uint8_t>(GetWiperState()) << 4) + static_cast<uint8_t>(GetWiperSpeed());
    stMsg.frame.data8[6] = (GetFlasherVal(3) << 3) + (GetFlasherVal(2) << 2) +
                           (GetFlasherVal(1) << 1) + GetFlasherVal(0);
    stMsg.frame.data8[7] = 0;

    stMsg.bSend = true; // Always send

    return stMsg;
}

CANTxMsg TxMsg4()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 4 (Out 1-8 Reset Count)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 4;
    stMsg.frame.DLC = 8; // Bytes to send
    stMsg.frame.data8[0] = GetOutputOcCount(0);
    stMsg.frame.data8[1] = GetOutputOcCount(1);
    stMsg.frame.data8[2] = GetOutputOcCount(2);
    stMsg.frame.data8[3] = GetOutputOcCount(3);
    stMsg.frame.data8[4] = GetOutputOcCount(4);
    stMsg.frame.data8[5] = GetOutputOcCount(5);
    stMsg.frame.data8[6] = GetOutputOcCount(6);
    stMsg.frame.data8[7] = GetOutputOcCount(7);

    stMsg.bSend = true; // Always send

    return stMsg;
}

CANTxMsg TxMsg5()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 5 (CAN Inputs and Virtual Inputs)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 5;
    stMsg.frame.DLC = 8; // Bytes to send
    stMsg.frame.data32[0] = GetCanInOutputs();
    stMsg.frame.data32[1] = GetVirtIns();

    stMsg.bSend = GetAnyCanInEnable() || GetAnyVirtInEnable();

    return stMsg;
}

CANTxMsg TxMsg6()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 6 (Counters and Conditions)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 6;
    stMsg.frame.DLC = 8; // Bytes to send
    stMsg.frame.data8[0] = (uint8_t)GetCounterVal(0);
    stMsg.frame.data8[1] = (uint8_t)GetCounterVal(1);
    stMsg.frame.data8[2] = (uint8_t)GetCounterVal(2);
    stMsg.frame.data8[3] = (uint8_t)GetCounterVal(3);
    stMsg.frame.data32[1] = GetConditions();

    stMsg.bSend = GetAnyCounterEnable() || GetAnyConditionEnable();

    return stMsg;
}

CANTxMsg TxMsg7()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 7 (CAN Input Values 1-2)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 7;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(0), 0, 32, GetCanInFactor(0), GetCanInOffset(0), GetCanInByteOrder(0));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(1), 33, 32, GetCanInFactor(1), GetCanInOffset(1), GetCanInByteOrder(1));

    stMsg.bSend = GetCanInEnable(0) || GetCanInEnable(1);

    return stMsg;
}

CANTxMsg TxMsg8()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 8 (CAN Input Values 3-4)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 8;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(2), 0, 32, GetCanInFactor(2), GetCanInOffset(2), GetCanInByteOrder(2));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(3), 33, 32, GetCanInFactor(3), GetCanInOffset(3), GetCanInByteOrder(3));

    stMsg.bSend = GetCanInEnable(2) || GetCanInEnable(3);

    return stMsg;
}

CANTxMsg TxMsg9()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 9 (CAN Input Values 5-6)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 9;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(4), 0, 32, GetCanInFactor(4), GetCanInOffset(4), GetCanInByteOrder(4));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(5), 33, 32, GetCanInFactor(5), GetCanInOffset(5), GetCanInByteOrder(5));

    stMsg.bSend = GetCanInEnable(4) || GetCanInEnable(5);

    return stMsg;
}

CANTxMsg TxMsg10()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 10 (CAN Input Values 7-8)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 10;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(6), 0, 32, GetCanInFactor(6), GetCanInOffset(6), GetCanInByteOrder(6));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(7), 33, 32, GetCanInFactor(7), GetCanInOffset(7), GetCanInByteOrder(7));

    stMsg.bSend = GetCanInEnable(6) || GetCanInEnable(7);

    return stMsg;
}

CANTxMsg TxMsg11()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 11 (CAN Input Values 9-10)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 11;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(8), 0, 32, GetCanInFactor(8), GetCanInOffset(8), GetCanInByteOrder(8));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(9), 33, 32, GetCanInFactor(9), GetCanInOffset(9), GetCanInByteOrder(9));

    stMsg.bSend = GetCanInEnable(8) || GetCanInEnable(9);

    return stMsg;
}

CANTxMsg TxMsg12()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 12 (CAN Input Values 11-12)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 12;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(10), 0, 32, GetCanInFactor(10), GetCanInOffset(10), GetCanInByteOrder(10));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(11), 33, 32, GetCanInFactor(11), GetCanInOffset(11), GetCanInByteOrder(11));

    stMsg.bSend = GetCanInEnable(10) || GetCanInEnable(11);

    return stMsg;
}

CANTxMsg TxMsg13()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 13 (CAN Input Values 13-14)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 13;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(12), 0, 32, GetCanInFactor(12), GetCanInOffset(12), GetCanInByteOrder(12));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(13), 33, 32, GetCanInFactor(13), GetCanInOffset(13), GetCanInByteOrder(13));

    stMsg.bSend = GetCanInEnable(12) || GetCanInEnable(13);

    return stMsg;
}

CANTxMsg TxMsg14()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 14 (CAN Input Values 15-16)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 14;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(14), 0, 32, GetCanInFactor(14), GetCanInOffset(14), GetCanInByteOrder(14));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(15), 33, 32, GetCanInFactor(15), GetCanInOffset(15), GetCanInByteOrder(15));

    stMsg.bSend = GetCanInEnable(14) || GetCanInEnable(15);

    return stMsg;
}

CANTxMsg TxMsg15()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 15 (CAN Input Values 17-18)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 15;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(16), 0, 32, GetCanInFactor(16), GetCanInOffset(16), GetCanInByteOrder(16));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(17), 33, 32, GetCanInFactor(17), GetCanInOffset(17), GetCanInByteOrder(17));

    stMsg.bSend = GetCanInEnable(16) || GetCanInEnable(17);

    return stMsg;
}

CANTxMsg TxMsg16()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 16 (CAN Input Values 19-20)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 16;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(18), 0, 32, GetCanInFactor(18), GetCanInOffset(18), GetCanInByteOrder(18));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(19), 33, 32, GetCanInFactor(19), GetCanInOffset(19), GetCanInByteOrder(19));

    stMsg.bSend = GetCanInEnable(18) || GetCanInEnable(19);

    return stMsg;
}

CANTxMsg TxMsg17()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 17 (CAN Input Values 21-22)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 17;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(20), 0, 32, GetCanInFactor(20), GetCanInOffset(20), GetCanInByteOrder(20));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(21), 33, 32, GetCanInFactor(21), GetCanInOffset(21), GetCanInByteOrder(21));

    stMsg.bSend = GetCanInEnable(20) || GetCanInEnable(21);

    return stMsg;
}

CANTxMsg TxMsg18()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 18 (CAN Input Values 23-24)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 18;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(22), 0, 32, GetCanInFactor(22), GetCanInOffset(22), GetCanInByteOrder(22));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(23), 33, 32, GetCanInFactor(23), GetCanInOffset(23), GetCanInByteOrder(23));

    stMsg.bSend = GetCanInEnable(22) || GetCanInEnable(23);

    return stMsg;
}

CANTxMsg TxMsg19()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 19 (CAN Input Values 25-26)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 19;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(24), 0, 32, GetCanInFactor(24), GetCanInOffset(24), GetCanInByteOrder(24));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(25), 33, 32, GetCanInFactor(25), GetCanInOffset(25), GetCanInByteOrder(25));

    stMsg.bSend = GetCanInEnable(24) || GetCanInEnable(25);

    return stMsg;
}

CANTxMsg TxMsg20()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 20 (CAN Input Values 27-28)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 20;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(26), 0, 32, GetCanInFactor(26), GetCanInOffset(26), GetCanInByteOrder(26));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(27), 33, 32, GetCanInFactor(27), GetCanInOffset(27), GetCanInByteOrder(27));

    stMsg.bSend = GetCanInEnable(26) || GetCanInEnable(27);

    return stMsg;
}

CANTxMsg TxMsg21()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 21 (CAN Input Values 29-30)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 21;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(28), 0, 32, GetCanInFactor(28), GetCanInOffset(28), GetCanInByteOrder(28));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(29), 33, 32, GetCanInFactor(29), GetCanInOffset(29), GetCanInByteOrder(29));

    stMsg.bSend = GetCanInEnable(28) || GetCanInEnable(29);

    return stMsg;
}

CANTxMsg TxMsg22()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 22 (CAN Input Values 31-32)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 22;
    stMsg.frame.DLC = 8; // Bytes to send
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(30), 0, 32, GetCanInFactor(30), GetCanInOffset(30), GetCanInByteOrder(30));
    Dbc::EncodeFloat(stMsg.frame.data8, GetCanInVal(31), 33, 32, GetCanInFactor(31), GetCanInOffset(31), GetCanInByteOrder(31));

    stMsg.bSend = GetCanInEnable(30) || GetCanInEnable(31);

    return stMsg;
}

CANTxMsg TxMsg23()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 23 (Output Duty Cycle)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 23;
    stMsg.frame.DLC = 8;
    stMsg.frame.data8[0] = GetOutputDC(0);
    stMsg.frame.data8[1] = GetOutputDC(1);
    stMsg.frame.data8[2] = GetOutputDC(2);
    stMsg.frame.data8[3] = GetOutputDC(3);
    stMsg.frame.data8[4] = GetOutputDC(4);
    stMsg.frame.data8[5] = GetOutputDC(5);
    stMsg.frame.data8[6] = GetOutputDC(6);
    stMsg.frame.data8[7] = GetOutputDC(7);

    stMsg.bSend = GetAnyPwmEnable();

    return stMsg;
}

CANTxMsg TxMsg24()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 24 (Keypad Button States)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 24;
    stMsg.frame.DLC = 8;
    stMsg.frame.data8[0] =  GetKeypadButtons(0) & 0xFF;
    stMsg.frame.data8[1] = (GetKeypadButtons(0) >> 8) & 0xFF;
    stMsg.frame.data8[2] = (GetKeypadButtons(0) >> 16) & 0xFF;
    stMsg.frame.data8[3] =  GetKeypadButtons(1) & 0xFF;
    stMsg.frame.data8[4] = (GetKeypadButtons(1) >> 8) & 0xFF;
    stMsg.frame.data8[5] = (GetKeypadButtons(1) >> 16) & 0xFF;
    stMsg.frame.data8[6] = 0;
    stMsg.frame.data8[7] = 0;

    stMsg.bSend = GetAnyKeypadEnable();

    return stMsg;
}

CANTxMsg TxMsg25()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 25 (Keypad 1 Dials)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 25;
    stMsg.frame.DLC = 8;
    stMsg.frame.data8[0] =  (uint16_t)GetKeypadDialVal(0,0) & 0xFF;
    stMsg.frame.data8[1] = ((uint16_t)GetKeypadDialVal(0,0) >> 8) & 0xFF;
    stMsg.frame.data8[2] = ((uint16_t)GetKeypadDialVal(0,1) & 0xFF);
    stMsg.frame.data8[3] = ((uint16_t)GetKeypadDialVal(0,1) >> 8) & 0xFF;
    stMsg.frame.data8[4] =  (uint16_t)GetKeypadDialVal(0,2) & 0xFF;
    stMsg.frame.data8[5] = ((uint16_t)GetKeypadDialVal(0,2) >> 8) & 0xFF;
    stMsg.frame.data8[6] =  (uint16_t)GetKeypadDialVal(0,3) & 0xFF;
    stMsg.frame.data8[7] = ((uint16_t)GetKeypadDialVal(0,3) >> 8) & 0xFF;

    stMsg.bSend = GetKeypadEnable(0);

    return stMsg;
}

CANTxMsg TxMsg26()
{
    CANTxMsg stMsg;
    //=======================================================
    // Build Msg 26 (Keypad 2 Dials)
    //=======================================================
    stMsg.frame.SID = stConfig.stDevConfig.nBaseId + 26;
    stMsg.frame.DLC = 8;
    stMsg.frame.data8[0] =  (uint16_t)GetKeypadDialVal(1,0) & 0xFF;
    stMsg.frame.data8[1] = ((uint16_t)GetKeypadDialVal(1,0) >> 8) & 0xFF;
    stMsg.frame.data8[2] = ((uint16_t)GetKeypadDialVal(1,1) & 0xFF);
    stMsg.frame.data8[3] = ((uint16_t)GetKeypadDialVal(1,1) >> 8) & 0xFF;
    stMsg.frame.data8[4] =  (uint16_t)GetKeypadDialVal(1,2) & 0xFF;
    stMsg.frame.data8[5] = ((uint16_t)GetKeypadDialVal(1,2) >> 8) & 0xFF;
    stMsg.frame.data8[6] =  (uint16_t)GetKeypadDialVal(1,3) & 0xFF;
    stMsg.frame.data8[7] = ((uint16_t)GetKeypadDialVal(1,3) >> 8) & 0xFF;

    stMsg.bSend = GetKeypadEnable(1);

    return stMsg;
}