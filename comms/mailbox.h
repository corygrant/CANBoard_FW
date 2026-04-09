#pragma once

#include <cstdint>
#include "hal.h"

#define MAILBOX_SIZE 16

msg_t PostTxFrame(CANTxFrame *frame);
msg_t FetchTxFrame(CANTxFrame *frame);
msg_t PostRxFrame(CANRxFrame *frame);
msg_t FetchRxFrame(CANRxFrame *frame);
bool RxFramesEmpty();