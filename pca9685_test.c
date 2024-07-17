#include <stdio.h>
#include "ledcontrol.h"
#include "sdkconfig.h"

void app_main(void)
{
  controlLed(1, 800);
  controlLed(3, 100);
}
