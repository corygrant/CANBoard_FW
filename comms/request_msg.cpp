#include "request_msg.h"
#include "pdm.h"
#include "can.h"
#include "config.h"
#include "config_handler.h"
#include "mcu_utils.h"
#include "dingopdm_config.h"
#include "mailbox.h"
#include "enums.h"

// External variables from pdm.cpp that we need access to
extern PdmConfig stConfig;
extern bool bSleepRequest;

void CheckRequestMsgs(CANRxFrame *frame)
{
    //Check for settings request message, (Base ID - 1)
    if(frame->SID != 0x080)// (stConfig.stDevConfig.nBaseId - 1))
        return;

    // Check for sleep request
    if ((frame->DLC == 8) && 
        (frame->data8[0] == static_cast<uint8_t>(MsgCmd::Sleep)) &&
        (frame->data8[1] == 'Q') && (frame->data8[2] == 'U') && 
        (frame->data8[3] == 'I') && (frame->data8[4] == 'T'))
    {
        CANTxFrame txMsg;
        txMsg.SID = 0x081;// stConfig.stDevConfig.nBaseId + TX_SETTINGS_ID_OFFSET;
        txMsg.IDE = CAN_IDE_STD;
        txMsg.DLC = 2;
        txMsg.data8[0] = static_cast<uint8_t>(MsgCmd::Sleep);
        txMsg.data8[1] = 'Q';
        txMsg.data8[2] = 'U';
        txMsg.data8[3] = 'I';
        txMsg.data8[4] = 'T';
        txMsg.data8[5] = 1; // Acknowledge sleep request
        txMsg.data8[6] = 0;
        txMsg.data8[7] = 0;

        PostTxFrame(&txMsg);

        bSleepRequest = true;
    }

    // Check for burn request
    if ((frame->DLC == 8) && 
        (frame->data8[0] == static_cast<uint8_t>(MsgCmd::BurnSettings)) &&
        (frame->data8[1] == 1) &&
        (frame->data8[2] == 3) && 
        (frame->data8[3] == 8))
    {
        CANTxFrame txMsg;
        txMsg.SID = 0x081;//stConfig.stDevConfig.nBaseId + TX_SETTINGS_ID_OFFSET;
        txMsg.IDE = CAN_IDE_STD;
        txMsg.DLC = 8;
        txMsg.data8[0] = static_cast<uint8_t>(MsgCmd::BurnSettings);
        txMsg.data8[1] = 1;
        txMsg.data8[2] = 3;
        txMsg.data8[3] = 8;
        txMsg.data8[4] = WriteConfig();
        txMsg.data8[5] = 0;
        txMsg.data8[6] = 0;
        txMsg.data8[7] = 0;
        PostTxFrame(&txMsg);
    }

    // Check for bootloader request
    if ((frame->DLC == 8) &&
        (frame->data8[0] == static_cast<uint8_t>(MsgCmd::Bootloader)) && 
        (frame->data8[1] == 'B') && (frame->data8[2] == 'O') && 
        (frame->data8[3] == 'O') && (frame->data8[4] == 'T') && (frame->data8[5] == 'L'))
    {
        RequestBootloader();
    }

    // Check for version request
    if ((frame->DLC == 8) &&
        (frame->data8[0] == static_cast<uint8_t>(MsgCmd::Version)))
    {
        CANTxFrame txMsg;
        txMsg.SID = 0x081;//stConfig.stDevConfig.nBaseId + TX_SETTINGS_ID_OFFSET;
        txMsg.IDE = CAN_IDE_STD;
        txMsg.DLC = 8;
        txMsg.data8[0] = static_cast<uint8_t>(MsgCmd::Version);
        txMsg.data8[1] = 0;
        txMsg.data8[2] = 0;
        txMsg.data8[3] = 0;
        txMsg.data8[4] = MAJOR_VERSION;
        txMsg.data8[5] = MINOR_VERSION;
        txMsg.data8[6] = BUILD >> 8;
        txMsg.data8[7] = BUILD & 0xFF;

        PostTxFrame(&txMsg);
    }
}