#ifndef LEDCONTROL_H
#define LEDCONTROL_H

#include <stdint.h>


#define I2C_MASTER_SCL_IO 19 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18 /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_ADDRESS 0x40 /*!< address for PCA9685 */
#define PCA9685_FREQ 1000 /*!< working frequency of PCA9685 */



/**
 *
 *
 * @param N Led Number [1-16]
 * @param perc PWM on percentage in [0-1000] range
 *
 * @return 0 if no error, >0 otherwise
 */
int controlLed(uint8_t N, uint16_t perc);


#endif // LEDCONTROL_H
