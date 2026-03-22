#pragma once

#include <cstdint>
#include "port.h"
#include "enums.h"
#include "mailbox.h"
#include "dingopdm_config.h"    

struct CANTxMsg
{
    CANTxFrame frame;
    bool bSend;
};

CANTxMsg TxMsg0();
CANTxMsg TxMsg1();
CANTxMsg TxMsg2();
CANTxMsg TxMsg3();
CANTxMsg TxMsg4();
CANTxMsg TxMsg5();
CANTxMsg TxMsg6();
CANTxMsg TxMsg7();
CANTxMsg TxMsg8();
CANTxMsg TxMsg9();
CANTxMsg TxMsg10();
CANTxMsg TxMsg11();
CANTxMsg TxMsg12();
CANTxMsg TxMsg13();
CANTxMsg TxMsg14();
CANTxMsg TxMsg15();
CANTxMsg TxMsg16();
CANTxMsg TxMsg17();
CANTxMsg TxMsg18();
CANTxMsg TxMsg19();
CANTxMsg TxMsg20();
CANTxMsg TxMsg21();
CANTxMsg TxMsg22();
CANTxMsg TxMsg23();
CANTxMsg TxMsg24();
CANTxMsg TxMsg25();
CANTxMsg TxMsg26();

[[maybe_unused]] static CANTxMsg (*TxMsgs[PDM_NUM_TX_MSGS])() = {
    TxMsg0,
    TxMsg1,
    TxMsg2,
    TxMsg3,
    TxMsg4,
    TxMsg5,
    TxMsg6,
    TxMsg7,
    TxMsg8,
    TxMsg9,
    TxMsg10,
    TxMsg11,
    TxMsg12,
    TxMsg13,
    TxMsg14,
    TxMsg15,
    TxMsg16,
    TxMsg17,
    TxMsg18,
    TxMsg19,
    TxMsg20,
    TxMsg21,
    TxMsg22,
    TxMsg23,
    TxMsg24,
    TxMsg25,
    TxMsg26};

