#include "ch.h"
#include "hal.h"

#include "canboard_config.h"
#include "can.h"
#include "analog.h"
#include "rotary_switch.h"

/*
 * Application entry point.
 */
int main(void)
{

  halInit();
  chSysInit();

  InitCan();
  InitAdc();

  while (true)
  {
    UpdateSwPos();
    chThdSleepMilliseconds(500);
  }
}
