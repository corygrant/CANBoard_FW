#include "infomsg.h"
#include "pdm.h"
#include "config.h"
#include "dingopdm_config.h"
#include "status.h"

// Static message objects that were in pdm.cpp
static InfoMsg StateRunMsg(MsgType::Info, MsgSrc::State_Run);
static InfoMsg StateSleepMsg(MsgType::Info, MsgSrc::State_Sleep);
static InfoMsg StateOvertempMsg(MsgType::Error, MsgSrc::State_Overtemp);
static InfoMsg StateErrorMsg(MsgType::Error, MsgSrc::State_Error);

static InfoMsg BattOvervoltageMsg(MsgType::Warning, MsgSrc::Voltage);
static InfoMsg BattUndervoltageMsg(MsgType::Warning, MsgSrc::Voltage);

static InfoMsg OutputOvercurrentMsg[PDM_NUM_OUTPUTS];
static InfoMsg OutputFaultMsg[PDM_NUM_OUTPUTS];

// External variables from pdm.cpp that we need access to
extern PdmConfig stConfig;
extern PdmState eState;
extern float fBattVolt;
extern Profet pf[PDM_NUM_OUTPUTS];

void CheckInfoMsgs()
{
    StateRunMsg.Check(eState == PdmState::Run, stConfig.stDevConfig.nBaseId, 0, 0, 0);
    StateSleepMsg.Check(eState == PdmState::Sleep, stConfig.stDevConfig.nBaseId, 0, 0, 0);
    StateOvertempMsg.Check(eState == PdmState::OverTemp, stConfig.stDevConfig.nBaseId, GetBoardTemp() * 10, 0, 0);
    StateErrorMsg.Check(eState == PdmState::Error, stConfig.stDevConfig.nBaseId, GetBoardTemp() * 10, GetTotalCurrent() * 10, 0);

    BattOvervoltageMsg.Check(fBattVolt > BATT_HIGH_VOLT, stConfig.stDevConfig.nBaseId, fBattVolt * 10, 0, 0);
    BattUndervoltageMsg.Check(fBattVolt < BATT_LOW_VOLT, stConfig.stDevConfig.nBaseId, fBattVolt * 10, 0, 0);

    for (uint8_t i = 0; i < PDM_NUM_OUTPUTS; i++)
    {
        OutputOvercurrentMsg[i].Check(GetOutputState(i) == ProfetState::Overcurrent, stConfig.stDevConfig.nBaseId, i, GetOutputCurrent(i), 0);
        OutputFaultMsg[i].Check(GetOutputState(i) == ProfetState::Fault, stConfig.stDevConfig.nBaseId, i, GetOutputCurrent(i), 0);
    }
}

void SendInfoMsg(MsgType type, MsgSrc src, uint16_t nId, uint16_t nData0, uint16_t nData1, uint16_t nData2)
{
    CANTxFrame tx;
    tx.DLC = 8;

    tx.data8[0] = static_cast<uint8_t>(type);
    tx.data8[1] = static_cast<uint8_t>(src);
    tx.data16[1] = nData0;
    tx.data16[2] = nData1;
    tx.data16[3] = nData2;

    tx.SID = nId + TX_MSG_ID_OFFSET;
    tx.IDE = CAN_IDE_STD;
    PostTxFrame(&tx);
}

void InitInfoMsgs()
{
    for (uint8_t i = 0; i < PDM_NUM_OUTPUTS; i++)
    {
        OutputOvercurrentMsg[i] = InfoMsg(MsgType::Warning, MsgSrc::Overcurrent);
        OutputFaultMsg[i] = InfoMsg(MsgType::Error, MsgSrc::Overcurrent);
    }
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