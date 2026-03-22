#include "error.h"
#include "config.h"
#include "infomsg.h"

void Error::SetFatalError(FatalErrorType err, MsgSrc src)
{
    static InfoMsg FatalErrorMsg(MsgType::Error, src);
    FatalErrorMsg.Check(true, stConfig.stDevConfig.nBaseId, static_cast<uint8_t>(err), 0, 0);
}