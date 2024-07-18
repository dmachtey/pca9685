#include "ledcontrol.h"
#include "sdkconfig.h"
#include <stdio.h>

void app_main(void) {
  while (1) {
    for (uint16_t i = 0; i < 1000; ++i) {
      controlLed(1, i);
      controlLed(3, 1000 - i);
    }
  }
}
