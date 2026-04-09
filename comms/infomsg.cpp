#include "infomsg.h"
#include "config.h"
#include "status.h"

void SendInfoMsg(MsgType type, MsgSrc src, uint16_t nId, uint16_t nData0, uint16_t nData1, uint16_t nData2)
{
    CANTxFrame tx;
    tx.DLC = 8;

    tx.data8[0] = static_cast<uint8_t>(type);
    tx.data8[1] = static_cast<uint8_t>(src);
    tx.data16[1] = nData0;
    tx.data16[2] = nData1;
    tx.data16[3] = nData2;

    tx.SID = nId;
    tx.IDE = CAN_IDE_STD;
    PostTxFrame(&tx);
}

void InfoMsg::Check(bool bTrigger, uint16_t nId, uint16_t nData0, uint16_t nData1, uint16_t nData2)
{
    if (!bTrigger)
    {
        bSent = false;
        return;
    }

    if (bSent)
        return;

    SendInfoMsg(m_type, m_src, nId, nData0, nData1, nData2);
    bSent = true;
}