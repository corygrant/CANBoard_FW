#pragma once

#include <cstdint>
#include "port.h"
#include "enums.h"
#include "mailbox.h"
#include "canboard_config.h"    

struct CANTxMsg
{
    CANTxFrame frame;
    bool bSend;
};

CANTxMsg TxMsg0();
CANTxMsg TxMsg1();
CANTxMsg TxMsg2();

[[maybe_unused]] static CANTxMsg (*TxMsgs[NUM_TX_MSGS])() = {
    TxMsg0,
    TxMsg1,
    TxMsg2
};

