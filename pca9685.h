#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H

#include "esp_err.h"
#include <stdint.h>

typedef struct PCA9685_T {
  uint8_t addr;
  uint8_t i2c_num;
} PCA9685_t;

typedef enum OUTPUT_T { NPN, PNP, DIRECT } Output_t;

/**
 * Set the device address and i2c port number
 *
 * @param dev PCA9685_t struct that hold the device information
 * @param addr slave device address
 * @param i2c_num i2c port number
 */
void PCA9685setAddrPort(PCA9685_t *dev, uint8_t addr, uint8_t i2c_num);

/**
 * Reset the slave device to be used
 *
 * @param dev dev PCA9685_t struct that hold the device information
 *
 * @return esp_err_t error codes
 */
esp_err_t PCA9685reset(PCA9685_t dev);

/**
 * Set the PWM working frequency
 *
 * @param dev PCA9685_t struct that hold the device information
 * @param freq working frequency
 *
 * @return esp_err_t error codes
 */
esp_err_t PCA9685setPWMFrequency(PCA9685_t dev, uint16_t freq);

/**
 *
 *
 * @param dev PCA9685_t struct that hold the device information
 * @param output
 *
 * @return esp_err_t error codes
 */
esp_err_t PCA9685setOutputType(PCA9685_t dev, Output_t output);

/**
 *
 *
 * @param dev PCA9685_t struct that hold the device information
 * @param channel commanded channel [0-15]
 * @param on delay time [0-4095]
 * @param off delay time [0-4095]
 *
 * @return esp_err_t error codes
 */
esp_err_t PCA9685setPWM(PCA9685_t dev, uint8_t channel, uint16_t on,
                        uint16_t off);

#endif /* PCA9685_DRIVER_H */
