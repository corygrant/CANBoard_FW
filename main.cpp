#include "ch.h"
#include "hal.h"

#include "canboard.h"

/*
 * Application entry point.
 */
int main(void)
{

  halInit();
  chSysInit();

  chThdSleepMilliseconds(500);

  InitCanboard();

  while (true)
  {
    chThdSleepMilliseconds(500);
  }
}
