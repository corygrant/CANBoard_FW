#include "param_protocol.h"
#include "dingopdm_config.h"
#include "config.h"
#include "mailbox.h"
#include "config_handler.h"
#include "crc.h"
#include <cstring>

extern PdmConfig stConfig;

uint16_t nNumWriteParams = 0;
uint16_t nNumReadParams = 0;

uint32_t nReadCrc = 0xFFFFFFFF;
uint32_t nWriteCrc = 0xFFFFFFFF;
uint32_t nCheckCrc = 0xFFFFFFFF;

void DecodeParamCmd(CANRxFrame *rx, ParamMsg *out)
{
    out->eCmd = static_cast<MsgCmd>(rx->data8[0]);
    out->nIndex = rx->data8[1] | (rx->data8[2] << 8);
    out->nSubIndex = rx->data8[3];
    out->nValue =  rx->data8[4] |
                   rx->data8[5] << 8 |
                   rx->data8[6] << 16 |
                  (rx->data8[7]) << 24;
}

void EncodeParamRsp(CANTxFrame *tx, uint8_t cmd, uint16_t index, uint8_t subindex, uint32_t value)
{
    tx->IDE = 0;
    tx->RTR = 0;
    tx->DLC = 8;

    tx->SID =  stConfig.stDevConfig.nParamTxId;

    tx->data8[0] = cmd;
    tx->data8[1] = index & 0xFF;
    tx->data8[2] = (index >> 8) & 0xFF;
    tx->data8[3] = subindex;
    tx->data8[4] = value & 0xFF;
    tx->data8[5] = (value >> 8) & 0xFF;
    tx->data8[6] = (value >> 16) & 0xFF;
    tx->data8[7] = (value >> 24) & 0xFF;
}

void SendAllParams(bool modifiedOnly) {
    CANTxFrame tx;
    uint8_t nBatchCount = 0;

    for (int i = 0; i < NUM_PARAMS; i++) {
        if (modifiedOnly && IsDefaultValue(&stParams[i])) {
            continue; // Skip default params if only sending modified
        }

        if(nBatchCount > 50) { // Send in batches of 50 to avoid overflowing CAN buffers
            chThdSleepMilliseconds(10); // Short delay between batches
            nBatchCount = 0;
        }
        nBatchCount++;

        uint32_t value = ReadParam(&stParams[i]);
        EncodeParamRsp(&tx, static_cast<uint8_t>(MsgCmd::ReadAllRsp), 
                        stParams[i].nIndex, stParams[i].nSubIndex, value);
        msg_t ret;
        do {
            ret = PostTxFrame(&tx);
            if (ret != MSG_OK)
                chThdSleepMicroseconds(200);
        } while (ret != MSG_OK);

        nReadCrc = CalculateCRC32Partial(&tx.data8[4], 4, nReadCrc);

        nNumReadParams++;
    }

    chThdSleepMilliseconds(1);
    nReadCrc = ~nReadCrc; // Finalize CRC after all params sent
    EncodeParamRsp(&tx, static_cast<uint8_t>(MsgCmd::ReadAllComplete), nNumReadParams, 0, nReadCrc); // End of params marker, return number of params sent and CRC
    PostTxFrame(&tx);
}

void CheckCrc() {
    CANTxFrame tx;

    nCheckCrc = 0xFFFFFFFF; // Reset CRC for new batch
    for (int i = 0; i < NUM_PARAMS; i++) {
        uint32_t value = ReadParam(&stParams[i]);
        EncodeParamRsp(&tx, static_cast<uint8_t>(MsgCmd::ReadAllRsp), 
                        stParams[i].nIndex, stParams[i].nSubIndex, value);
        nCheckCrc = CalculateCRC32Partial(&tx.data8[4], 4, nCheckCrc);
    }
    nCheckCrc = ~nCheckCrc; // Finalize CRC after all params sent
    
    EncodeParamRsp(&tx, static_cast<uint8_t>(MsgCmd::CheckCrcRsp), 0, 0, nCheckCrc); // End of params marker, return number of params sent and CRC
    PostTxFrame(&tx);

}

void SetAllDefaultParams(bool temp) {
    for (int i = 0; i < NUM_PARAMS; i++) {
        WriteParam(&stParams[i], stParams[i].nDefaultVal, temp);
    }
}

void ApplyTempParams() {
    for (int i = 0; i < NUM_PARAMS; i++) {
        uint32_t tempVal = ReadParam(&stParams[i], true);
        WriteParam(&stParams[i], tempVal, false);
    }
}

MsgCmd ProcessParamMsg(CANRxFrame *rx, uint16_t *nIndex) {
    CANTxFrame tx;
    ParamMsg msg;

    if (rx->SID != stConfig.stDevConfig.nParamRxId)
        return MsgCmd::Invalid;

    if (rx->DLC != 8)
        return MsgCmd::Invalid;

    DecodeParamCmd(rx, &msg);

    *nIndex = msg.nIndex;

    switch(msg.eCmd) {
        case MsgCmd::Read: {
            const ParamInfo* param = FindParam(msg.nIndex, msg.nSubIndex);
            if (param) {
                uint32_t value = ReadParam(param);
                EncodeParamRsp(&tx, static_cast<uint8_t>(MsgCmd::Read), msg.nIndex, msg.nSubIndex, value);
                PostTxFrame(&tx);
                break;
            }
            EncodeParamRsp(&tx, static_cast<uint8_t>(MsgCmd::ReadParamNotFound), msg.nIndex, msg.nSubIndex, 0);
            PostTxFrame(&tx);
            break;
        }

        case MsgCmd::Write: {
            const ParamInfo* param = FindParam(msg.nIndex, msg.nSubIndex);
            if (param && WriteParam(param, msg.nValue)) {
                uint32_t value = ReadParam(param);
                EncodeParamRsp(&tx, static_cast<uint8_t>(MsgCmd::Write), msg.nIndex, msg.nSubIndex, value);
                PostTxFrame(&tx);
            }
            break;
        }

        case MsgCmd::ReadAll:
        case MsgCmd::ReadAllModified:
            nNumReadParams = 0;
            nReadCrc = 0xFFFFFFFF; // Reset CRC for new batch
            EncodeParamRsp(&tx, static_cast<uint8_t>(msg.eCmd), 0, 0, 0); // Start of params marker
            PostTxFrame(&tx);
            chThdSleepMilliseconds(1);
            SendAllParams(msg.eCmd == MsgCmd::ReadAllModified);
            break;

        case MsgCmd::WriteAll:
        case MsgCmd::WriteAllModified:
            nNumWriteParams = 0;
            nWriteCrc = 0xFFFFFFFF; // Reset CRC for new batch
            SetAllDefaultParams(true); // Clear temp values
            EncodeParamRsp(&tx, static_cast<uint8_t>(msg.eCmd), 0, 0, 0); // Start of params marker
            PostTxFrame(&tx);
            break;

        case MsgCmd::WriteAllVal: {
            const ParamInfo* param = FindParam(msg.nIndex, msg.nSubIndex);
            //Param not found or invalid value, respond with error
            if (!param){
                EncodeParamRsp(&tx, static_cast<uint8_t>(MsgCmd::WriteAllParamNotFound), msg.nIndex, msg.nSubIndex, 0);
                PostTxFrame(&tx);
                break;
            }
            //Param out of range, respond with error
            if (!WriteParam(param, msg.nValue, true)) {
                EncodeParamRsp(&tx, static_cast<uint8_t>(MsgCmd::WriteAllOutOfRange), msg.nIndex, msg.nSubIndex, msg.nValue);
                PostTxFrame(&tx);
                break;
            }
            //Param found, in range, staged successfully, respond with value for confirmation
            nWriteCrc = CalculateCRC32Partial(&rx->data8[4], 4, nWriteCrc);
            nNumWriteParams++;
            break;
        }

        case MsgCmd::WriteAllComplete: {
            // Ensure all params were written
            uint16_t nExpectedParams = rx->data8[1] | (rx->data8[2] << 8);
            if (nNumWriteParams == nExpectedParams) {
                ApplyTempParams();
            }
            nWriteCrc = ~nWriteCrc; // Finalize CRC
            EncodeParamRsp(&tx, static_cast<uint8_t>(MsgCmd::WriteAllComplete), nNumWriteParams, 0, nWriteCrc); // End of params marker, return number of params written and CRC
            PostTxFrame(&tx);
            break;
        }

        case MsgCmd::CheckCrc:
            CheckCrc();
            break;

        default:
            break;
    }

    return msg.eCmd;
}