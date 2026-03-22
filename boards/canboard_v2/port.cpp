#include "port.h"
#include "canboard_config.h"


static const CANConfig canConfig1000 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=2, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(1)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

static const CANConfig canConfig500 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=4, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(3)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

static const CANConfig canConfig250 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=8, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(7)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

static const CANConfig canConfig125 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=16, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(15)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

const CANConfig& GetCanConfig(CanBitrate bitrate) {
    switch(bitrate) {
        case CanBitrate::Bitrate_1000K:
            return canConfig1000;
        case CanBitrate::Bitrate_500K:
            return canConfig500;
        case CanBitrate::Bitrate_250K:
            return canConfig250;
        case CanBitrate::Bitrate_125K:
            return canConfig125;
        default:
            return canConfig500;
    }
    return canConfig500;
}
