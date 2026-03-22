#pragma once

#include "enums.h"
#include "msg.h"

// Info message management functions extracted from pdm.cpp

void CheckInfoMsgs();
void InitInfoMsgs();

void SendInfoMsg(MsgType type, MsgSrc src, uint16_t nId, uint16_t nData0, uint16_t nData1, uint16_t nData2);

class InfoMsg
{
public:
    InfoMsg(MsgType type = MsgType::Info, MsgSrc src = MsgSrc::Init)
        : m_type(type), m_src(src){
          };

    InfoMsg& operator=(const InfoMsg& other){
        if (this != &other)
        {
            m_type = other.m_type;
            m_src = other.m_src;
        }
        return *this;
    }

    /*
     * Send a message if the trigger is true and the message hasn't been sent yet
     */

    void Check(bool bTrigger, uint16_t nId, uint16_t nData0, uint16_t nData1, uint16_t nData2);

private:
    MsgType m_type;
    MsgSrc m_src;

    bool bSent;
    bool bLastTrig;
};