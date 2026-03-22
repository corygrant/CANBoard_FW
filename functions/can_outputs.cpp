#include "can_outputs.h"
#include "mailbox.h"
#include "dbc.h"

Config_CanOutput* CanOutputs::pConfigs[PDM_NUM_CAN_OUTPUTS];
float* CanOutputs::pInput[PDM_NUM_CAN_OUTPUTS];
CanOutput CanOutputs::canOut[CAN_OUT_FRAMES];
int8_t CanOutputs::nAssignedOut[PDM_NUM_CAN_OUTPUTS];

void CanOutputs::ClearFrames()
{
    for (int i = 0; i < CAN_OUT_FRAMES; ++i)
    {
        canOut[i].stFrame.data64[0] = 0;
        canOut[i].stFrame.DLC = 0;
        canOut[i].stFrame.IDE = 0;
        canOut[i].stFrame.EID = 0; //Clears SID as well, union
    }

    for (int i = 0; i < PDM_NUM_CAN_OUTPUTS; ++i)
    {
        nAssignedOut[i] = -1;
    }
}

void CanOutputs::InitAllFrames()
{
    ClearFrames();

    for (int i = 0; i < PDM_NUM_CAN_OUTPUTS; ++i)
    {
        if (!pConfigs[i]->bEnabled) continue;

        uint8_t  nNewIDE      = pConfigs[i]->nIDE;
        uint16_t nNewID       = pConfigs[i]->nID;
        uint8_t  nNewStartBit = pConfigs[i]->nStartBit;
        uint8_t  nNewBitLen   = pConfigs[i]->nBitLength;
        uint16_t nNewInterval = pConfigs[i]->nInterval;

        if(nNewBitLen == 0 || nNewBitLen > 64) continue; // Invalid config, skip
        if(nNewInterval == 0) nNewInterval = 100; // Default to 100ms

        // Look for an existing frame with a matching ID
        for (int j = 0; j < CAN_OUT_FRAMES; ++j)
        {
            if (canOut[j].stFrame.DLC == 0) continue; // Unused frame slot

            bool bMatch = (nNewIDE == 0) ? (canOut[j].stFrame.IDE == 0 && canOut[j].stFrame.SID == nNewID)
                                      : (canOut[j].stFrame.IDE == 1 && canOut[j].stFrame.EID == nNewID);
            if (bMatch)
            {
                nAssignedOut[i] = j;

                //Expand DLC to fit longest output assigned to the frame
                uint8_t newDlc = CalcDlc(nNewStartBit, nNewBitLen);
                if (canOut[j].stFrame.DLC < newDlc)
                    canOut[j].stFrame.DLC = newDlc;

                //Use shortest interval of any output assigned to the frame
                if (canOut[j].nInterval > nNewInterval)
                    canOut[j].nInterval = nNewInterval;

                break;
            }
        }

        if (nAssignedOut[i] != -1) continue;

        // No existing frame found, assign to first available slot
        for (int j = 0; j < CAN_OUT_FRAMES; ++j)
        {
            if (canOut[j].stFrame.DLC == 0)
            {
                nAssignedOut[i] = j;
                canOut[j].stFrame.IDE = nNewIDE;

                //Only set one
                //SID and EID are a union
                if(nNewIDE == 0)
                    canOut[j].stFrame.SID = nNewID;
                else
                    canOut[j].stFrame.EID = nNewID;

                canOut[j].stFrame.DLC = CalcDlc(nNewStartBit, nNewBitLen);
                canOut[j].nInterval = nNewInterval;
                break;
            }
        }
    }
}

void CanOutputs::Update()
{
    for (int i = 0; i < CAN_OUT_FRAMES; ++i)
    {
        if (canOut[i].stFrame.DLC == 0) continue; // Skip unused frames

        if(canOut[i].CheckTxTime())
        {
            // Update data from all assigned outputs before sending
            for (int j = 0; j < PDM_NUM_CAN_OUTPUTS; ++j)
            {
                if (nAssignedOut[j] == i)
                {
                    Dbc::EncodeFloat( canOut[i].stFrame.data8, static_cast<float>(*pInput[j]), pConfigs[j]->nStartBit, pConfigs[j]->nBitLength,
                                    pConfigs[j]->fFactor, pConfigs[j]->fOffset, pConfigs[j]->eByteOrder);
                }
            }

            PostTxFrame(&canOut[i].stFrame);
        }
    }
}

uint8_t CanOutputs::CalcDlc(uint8_t nStartBit, uint8_t nBitLength)
{
    uint8_t nLastBit  = nStartBit + nBitLength - 1;
    uint8_t nEndByte  = nLastBit / 8;

    return (nEndByte + 1);
}