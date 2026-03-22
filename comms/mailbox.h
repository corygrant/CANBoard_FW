#pragma once

#include <cstdint>
#include "hal.h"

#define MAILBOX_SIZE 128

msg_t PostTxFrame(CANTxFrame *frame);
msg_t PostTxUsbFrame(CANTxFrame *frame);
msg_t FetchTxFrame(CANTxFrame *frame);
msg_t FetchTxUsbFrame(CANTxFrame *frame);
msg_t PostRxFrame(CANRxFrame *frame);
msg_t FetchRxFrame(CANRxFrame *frame);
bool RxFramesEmpty();