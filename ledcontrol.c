// =========================================================================
// Copyright 2024 Damian Pablo Machtey. All rights reserved.
// =========================================================================

#include "ledcontrol.h"
#include "pca9685.h"
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <stdint.h>

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */

uint8_t master_initialized = 0;
PCA9685_t i2c_master;

/**
 * Initialize the i2c master
 *
 */
void i2c_master_init(void);

/**
 * Turn off all leds
 *
 */
void allLedOff(void);

int controlLed(uint8_t N, uint16_t perc) {
  if (!master_initialized) {
    master_initialized = 1;
    i2c_master_init();
    i2c_master.addr = I2C_ADDRESS;
    i2c_master.i2c_num = I2C_MASTER_NUM;
    PCA9685reset(i2c_master);
    PCA9685setPWMFrequency(i2c_master, (uint16_t)PCA9685_FREQ);
    PCA9685setOutputType(i2c_master, DIRECT);
    allLedOff();
  }

  if (N > 0 && N < 17)
    return PCA9685setPWM(i2c_master, N - 1, 0, 4095 * perc / 1000);
  return 1;
}

void i2c_master_init(void) {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

  int i2c_master_port = I2C_MASTER_NUM;
  ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode,
                                     I2C_MASTER_RX_BUF_DISABLE,
                                     I2C_MASTER_TX_BUF_DISABLE, 0));
}

void allLedOff(void) {
  for (uint8_t j = 1; j < 17; ++j)
    controlLed(j, 0);
}
