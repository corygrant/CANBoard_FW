#include "port.h"
#include "canboard_config.h"



static const CANConfig canConfig500 =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    /*
     For 36MHz http://www.bittiming.can-wiki.info/ gives us Pre-scaler=4, Seq 1=15 and Seq 2=2. Subtract '1' for register values
    */
    CAN_BTR_SJW(0) | CAN_BTR_BRP(3)  | CAN_BTR_TS1(14) | CAN_BTR_TS2(1),
};

const CANConfig& GetCanConfig() {
    return canConfig500;
}
