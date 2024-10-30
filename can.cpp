#include "can.h"
#include "hal.h"
#include "port.h"
#include "analog.h"
#include "digital.h"
#include "rotary_switch.h"
#include "analog_switch.h"
#include "canboard_config.h"

uint8_t nCanBaseIdOffset = 0;
uint8_t nCanHeartbeat = 0;

static const CANFilter filters[1] = {
    {
        .filter = 0,
        .mode = 0,
        .scale = 1,
        .assignment = 0,
        .register1 = ((uint32_t)(CAN_BASE_ID + nCanBaseIdOffset + 3) << 21),
        .register2 = ((uint32_t)0x7FFU << 21) | (1U << 2),
    },
};

static THD_WORKING_AREA(waCanTxThread, 256);
void CanTxThread(void*)
{
    chRegSetThreadName("CAN Tx");

    CANTxFrame canTxMsg;

    while(1)
    {
        //=======================================================
        //Msg 0 (Analog inputs 1-4 millivolts)
        //=======================================================
        canTxMsg.IDE = CAN_IDE_STD;
        canTxMsg.SID = CAN_BASE_ID + nCanBaseIdOffset + 0;
        canTxMsg.DLC = 8;
        canTxMsg.data16[0] = (uint16_t)((float)GetAdcRaw(AnIn1) / 4096 * 4850);
        canTxMsg.data16[1] = (uint16_t)((float)GetAdcRaw(AnIn2) / 4096 * 4850);
        canTxMsg.data16[2] = (uint16_t)((float)GetAdcRaw(AnIn3) / 4096 * 4850);
        canTxMsg.data16[3] = (uint16_t)((float)GetAdcRaw(AnIn4) / 4096 * 4850);

        canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &canTxMsg, TIME_INFINITE);

        chThdSleepMilliseconds(CAN_TX_MSG_SPLIT);

        //=======================================================
        //Msg 1 (Analog input 5 millivolts and temperature)
        //=======================================================
        canTxMsg.IDE = CAN_IDE_STD;
        canTxMsg.SID = CAN_BASE_ID + nCanBaseIdOffset + 1;
        canTxMsg.DLC = 8;
        canTxMsg.data16[0] = (uint16_t)((float)GetAdcRaw(AnIn5) / 4096 * 4850);
        canTxMsg.data16[1] = 0;
        canTxMsg.data16[2] = 0;
        canTxMsg.data16[3] = GetTemperature();

        canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &canTxMsg, TIME_INFINITE);

        chThdSleepMilliseconds(CAN_TX_MSG_DELAY);

        //=======================================================
        //Msg 2 (Rotary switches, dig inputs, analog input switches, low side output status, heartbeat)
        //=======================================================
        canTxMsg.IDE = CAN_IDE_STD;
        canTxMsg.SID = CAN_BASE_ID + nCanBaseIdOffset + 2;
        canTxMsg.DLC = 8;
        canTxMsg.data8[0] = (GetRotarySwPos(RotarySw2) << 4) + GetRotarySwPos(RotarySw1);
        canTxMsg.data8[1] = (GetRotarySwPos(RotarySw4) << 4) + GetRotarySwPos(RotarySw3);
        canTxMsg.data8[2] = GetRotarySwPos(RotarySw5);
        canTxMsg.data8[3] = 0; //Empty
        canTxMsg.data8[4] = (GetDigIn(DigIn8) << 7) + (GetDigIn(DigIn7) << 6) + (GetDigIn(DigIn6) << 5) + (GetDigIn(DigIn5) << 4) + 
                            (GetDigIn(DigIn4) << 3) + (GetDigIn(DigIn3) << 2) + (GetDigIn(DigIn2) << 1) + GetDigIn(DigIn1);
        canTxMsg.data8[5] = (GetAnSwitch(AnIn5) << 4) + (GetAnSwitch(AnIn4) << 3) + (GetAnSwitch(AnIn3) << 2) + (GetAnSwitch(AnIn2) << 1) + GetAnSwitch(AnIn1);
        canTxMsg.data8[6] = (GetDigOut(DigOut4) << 3) + (GetDigOut(DigOut3) << 2) + (GetDigOut(DigOut2) << 1) + GetDigOut(DigOut1);;
        canTxMsg.data8[7] = nCanHeartbeat;

        canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &canTxMsg, TIME_INFINITE);

        nCanHeartbeat++;

        chThdSleepMilliseconds(CAN_TX_MSG_DELAY);
    }
}

static THD_WORKING_AREA(waCanRxThread, 128);
static THD_FUNCTION(CanRxThread, p)
{
    (void)p;

    CANRxFrame canRxMsg;

    chRegSetThreadName("CAN Rx");

    while (true)
    {
        msg_t msg = canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &canRxMsg, TIME_INFINITE);
        if (msg != MSG_OK)
            continue;
        if (canRxMsg.DLC >= 4)
        {
            SetDigOut(DigOut1, (bool)(canRxMsg.data8[0] & 0x01));
            SetDigOut(DigOut2, (bool)(canRxMsg.data8[1] & 0x01));
            SetDigOut(DigOut3, (bool)(canRxMsg.data8[2] & 0x01));
            SetDigOut(DigOut4, (bool)(canRxMsg.data8[3] & 0x01));
        }
        chThdSleepMilliseconds(10);
    }
}

void InitCan()
{
    //Set CAN base ID
    nCanBaseIdOffset = (GetDigIn(IdSel1) << 4) + (GetDigIn(IdSel2) << 5);

    canSTM32SetFilters(&CAND1, 0, 1, &filters[0]);
    canStart(&CAND1, &GetCanConfig());
    chThdCreateStatic(waCanTxThread, sizeof(waCanTxThread), NORMALPRIO + 1, CanTxThread, nullptr);
    chThdCreateStatic(waCanRxThread, sizeof(waCanRxThread), NORMALPRIO + 1, CanRxThread, nullptr);
}