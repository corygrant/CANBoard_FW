#include "canboard.h"
#include "ch.hpp"
#include "hal.h"
#include "port.h"
#include "canboard_config.h"
#include "config.h"
#include "param_protocol.h"
#include "config_handler.h"
#include "hw_devices.h"
#include "can.h"
#include "digital_input.h"
#include "digital_output.h"
#include "analog_input.h"
#include "can_input.h"
#include "can_outputs.h"
#include "virtual_input.h"
#include "flasher.h"
#include "counter.h"
#include "condition.h"
#include "mailbox.h"
#include "msg.h"
#include "request_msg.h"
#include "infomsg.h"
#include "error.h"

CanInput canIn[NUM_CAN_INPUTS];
CanOutputs canOutputs;
VirtualInput virtIn[NUM_VIRT_INPUTS];
Flasher flasher[NUM_FLASHERS];
Counter counter[NUM_COUNTERS];
Condition condition[NUM_CONDITIONS];

CanboardConfig stConfig;
CanboardConfig stConfigTemp; // Used for staging new config before applying
float *pVarMap[VAR_MAP_SIZE];

uint16_t nBaseIdOffset = 0;

void InitVarMap();
void CyclicUpdate();
void States();

struct CanboardThread : chibios_rt::BaseStaticThread<512>
{
    void main()
    {
        setName("CanboardThread");

        while (true)
        {
            CyclicUpdate();
            chThdSleepMilliseconds(2);
        }
    }
};
static CanboardThread canboardThread;

void InitCanboard()
{
    InitVarMap(); // Set val pointers

    InitConfig(); // Read config from memory

    ApplyAllConfig();

    if(!InitAdc() == HAL_RET_SUCCESS)
        Error::SetFatalError(FatalErrorType::ErrADC, MsgSrc::Init);
        
    if(!InitCan(&stConfig.stDevConfig) == HAL_RET_SUCCESS) // Starts CAN threads
        Error::SetFatalError(FatalErrorType::ErrCAN, MsgSrc::Init);

    InitInfoMsgs();

    canboardThread.start(NORMALPRIO);
}

void CyclicUpdate()
{
    CANRxFrame rxMsg;

    while (!RxFramesEmpty())
    {
        msg_t res = FetchRxFrame(&rxMsg);
        if (res == MSG_OK)
        {
            for (uint8_t i = 0; i < NUM_CAN_INPUTS; i++)
                canIn[i].CheckMsg(rxMsg);

            CheckRequestMsgs(&rxMsg);
            
            uint16_t nIndex = 0;
            MsgCmd cmd = ProcessParamMsg(&rxMsg, &nIndex);
            if (cmd == MsgCmd::WriteAllComplete)
            {
                ApplyAllConfig();
            }
            if (cmd == MsgCmd::Write)
            {
                ApplyConfig(nIndex & 0xFF00); // Mask instance, only base index is needed
            }
        }
    }

    for (uint8_t i = 0; i < NUM_DIG_INPUTS; i++)
        digIn[i].Update();

    for (uint8_t i = 0; i < NUM_DIG_OUTPUTS; i++)
        digOut[i].Update();

    for (uint8_t i = 0; i < NUM_ANALOG_INPUTS; i++)
        analogIn[i].Update();

    for (uint8_t i = 0; i < NUM_CAN_INPUTS; i++)
        canIn[i].CheckTimeout();

    canOutputs.Update();

    for (uint8_t i = 0; i < NUM_VIRT_INPUTS; i++)
        virtIn[i].Update();

    for (uint8_t i = 0; i < NUM_FLASHERS; i++)
        flasher[i].Update(SYS_TIME);

    for (uint8_t i = 0; i < NUM_COUNTERS; i++)
        counter[i].Update();

    for (uint8_t i = 0; i < NUM_CONDITIONS; i++)
        condition[i].Update();

    CheckInfoMsgs();

    //Set CAN base ID
    uint8_t idSel0 = static_cast<uint8_t>(idSel[0].fVal);
    uint8_t idSel1 = static_cast<uint8_t>(idSel[1].fVal);
    nBaseIdOffset = ((idSel0 & 0x01) << 4) + ((idSel1 & 0x01) << 5);
}

void InitVarMap()
{
    uint16_t index = 0;
    
    //System vars
    pVarMap[index++] = const_cast<float*>(&ALWAYS_FALSE);
    pVarMap[index++] = const_cast<float*>(&ALWAYS_TRUE);

    // Digital inputs
    for (uint8_t i = 0; i < NUM_DIG_INPUTS; i++)
        pVarMap[index++] = &digIn[i].fVal;

    // Digital outputs
    for (uint8_t i = 0; i < NUM_DIG_OUTPUTS; i++)
        pVarMap[index++] = &digOut[i].fVal;

    // Analog inputs
    for (uint8_t i = 0; i < NUM_ANALOG_INPUTS; i++)
    {
        pVarMap[index++] = &analogIn[i].fVal;
        pVarMap[index++] = &analogIn[i].fValMillivolts;
        pVarMap[index++] = &analogIn[i].fRotaryPos;
        pVarMap[index++] = &analogIn[i].fSwitchVal;
    }

    // CAN Inputs
    for (uint8_t i = 0; i < NUM_CAN_INPUTS; i++)
    {
        pVarMap[index++] = &canIn[i].fOutput;
        pVarMap[index++] = &canIn[i].fVal;
    }

    // Virtual Inputs
    for (uint8_t i = 0; i < NUM_VIRT_INPUTS; i++)
    {
        pVarMap[index++] = &virtIn[i].fVal;
    }

    // Flashers
    for (uint8_t i = 0; i < NUM_FLASHERS; i++)
    {
        pVarMap[index++] = &flasher[i].fVal;
    }

    // Conditions
    for (uint8_t i = 0; i < NUM_CONDITIONS; i++)
    {
        pVarMap[index++] = &condition[i].fVal;
    }

    // Counters
    for (uint8_t i = 0; i < NUM_COUNTERS; i++)
    {
        pVarMap[index++] = &counter[i].fVal;
    }

    //VarMap size must match the expected size
    if (index != VAR_MAP_SIZE)
        Error::SetFatalError(FatalErrorType::ErrVarMap, MsgSrc::Init);

}

uint8_t GetCanOffset()
{
    return nBaseIdOffset;
}