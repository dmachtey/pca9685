#include "pca9685.h"
#include <driver/i2c.h>
#include <math.h>
#include <stdint.h>

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

#define MODE1 0x00       /*!< Mode register 1 */
#define MODE2 0x01       /*!< Mode register 2 */
#define SUBADR1 0x02     /*!< I2C-bus subaddress 1 */
#define SUBADR2 0x03     /*!< I2C-bus subaddress 2 */
#define SUBADR3 0x04     /*!< I2C-bus subaddress 3 */
#define ALLCALLADR 0x05  /*!< LED All Call I2C-bus address */
#define LED0 0x6         /*!< LED0 start register */
#define LED0_ON_L 0x6    /*!< LED0 output and brightness control byte 0 */
#define LED0_ON_H 0x7    /*!< LED0 output and brightness control byte 1 */
#define LED0_OFF_L 0x8   /*!< LED0 output and brightness control byte 2 */
#define LED0_OFF_H 0x9   /*!< LED0 output and brightness control byte 3 */
#define LED_MULTIPLYER 4 /*!< For the other 15 channels */
#define ALLLED_ON_L                                                            \
  0xFA /*!< load all the LEDn_ON registers, byte 0 (turn 0-7 channels on) */
#define ALLLED_ON_H                                                            \
  0xFB /*!< load all the LEDn_ON registers, byte 1 (turn 8-15 channels on) */
#define ALLLED_OFF_L                                                           \
  0xFC /*!< load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off) */
#define ALLLED_OFF_H                                                           \
  0xFD /*!< load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)   \
        */
#define PRE_SCALE 0xFE        /*!< prescaler for output frequency */
#define CLOCK_FREQ 25000000.0 /*!< 25MHz default osc clock */
#define portTICK_RATE_MS portTICK_PERIOD_MS

/**
 * Read 2 bytes from i2c register
 *
 * @param dev PCA9685_t struct that hold the device information
 * @param regaddr register to read from
 * @param valueA
 * @param valueB
 *
 * @return esp_err_t error codes
 */
esp_err_t generic_read_two_i2c_register(PCA9685_t dev, uint8_t regaddr,
                                        uint8_t *valueA, uint8_t *valueB);

/**
 * Write 1 byte to i2c register
 *
 * @param dev PCA9685_t struct that hold the device information
 * @param regaddr register to write to
 * @param value
 *
 * @return esp_err_t error codes
 */
esp_err_t generic_write_i2c_register(PCA9685_t dev, uint8_t regaddr,
                                     uint8_t value);

/**
 * Write 2 words to i2c register
 *
 * @param dev PCA9685_t struct that hold the device information
 * @param regaddr register address to read
 * @param valueA
 * @param valueB
 *
 * @return esp_err_t error codes
 */
esp_err_t generic_write_i2c_register_two_words(PCA9685_t dev, uint8_t regaddr,
                                               uint16_t valueA,
                                               uint16_t valueB);

void PCA9685setAddrPort(PCA9685_t *dev, uint8_t addr, uint8_t i2c_num) {
  dev->addr = addr;
  dev->i2c_num = i2c_num;
}
esp_err_t PCA9685reset(PCA9685_t dev) {
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev.addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, MODE1, ACK_CHECK_EN); // 0x0 = "Mode register 1"
  i2c_master_write_byte(cmd, 0x80, ACK_CHECK_EN);  // 0x80 = "Reset"
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(dev.i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(50 / portTICK_RATE_MS);

  return ret;
}
esp_err_t PCA9685setPWMFrequency(PCA9685_t dev, uint16_t freq) {
  esp_err_t ret;

  // Send to sleep
  ret = generic_write_i2c_register(dev, MODE1, 0x10);
  if (ret != ESP_OK) {
    return ret;
  }

  // Set prescaler
  // calculation on page 25 of datasheet
  uint8_t prescale_val = round((CLOCK_FREQ / 4096 / (0.9 * freq)) - 1 + 0.5);
  ret = generic_write_i2c_register(dev, PRE_SCALE, prescale_val);
  if (ret != ESP_OK) {
    return ret;
  }

  // reset again
  PCA9685reset(dev);

  // Send to sleep again
  ret = generic_write_i2c_register(dev, MODE1, 0x10);
  if (ret != ESP_OK) {
    return ret;
  }

  // wait
  vTaskDelay(5 / portTICK_PERIOD_MS);

  // Write 0xa0 for auto increment LED0_x after received cmd
  ret = generic_write_i2c_register(dev, MODE1, 0xa0);
  if (ret != ESP_OK) {
    return ret;
  }
  return ret;
}

esp_err_t PCA9685setOutputType(PCA9685_t dev, Output_t output) {
  uint8_t vala = 0;
  uint8_t valb = 0;
  esp_err_t ret;
  ret = generic_read_two_i2c_register(dev, MODE1, &vala, &valb);
  if (ret != ESP_OK) {
    return ret;
  }
  // setting the Use of INVRT and OUTDRV for LEDn outputs
  // TABLE12 of PCA9685 documentation
  // INVRT bit 4
  // OUTDRV bit 2
  switch (output) {
  case NPN:
    ret = generic_write_i2c_register(dev, MODE2,
                                     (valb | 0b00000100) & 0b11101111);
    break;
  case PNP:
    ret = generic_write_i2c_register(dev, MODE2, (valb | 0b00010100));
    break;
  case DIRECT:
    ret = generic_write_i2c_register(dev, MODE2,
                                     (valb | 0b00010000) & 0b11111011);
    break;
  }
  return ret;
}

esp_err_t PCA9685setPWM(PCA9685_t dev, uint8_t channel, uint16_t on,
                        uint16_t off) {
  esp_err_t ret;

  uint8_t pinAddress = LED0_ON_L + LED_MULTIPLYER * channel;
  ret = generic_write_i2c_register_two_words(dev, pinAddress & 0xff, on, off);

  return ret;
}

esp_err_t generic_write_i2c_register(PCA9685_t dev, uint8_t regaddr,
                                     uint8_t value) {
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev.addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(dev.i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t generic_read_two_i2c_register(PCA9685_t dev, uint8_t regaddr,
                                        uint8_t *valueA, uint8_t *valueB) {
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev.addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(dev.i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    return ret;
  }
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, dev.addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, valueA, ACK_VAL);
  i2c_master_read_byte(cmd, valueB, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(dev.i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t generic_write_i2c_register_two_words(PCA9685_t dev, uint8_t regaddr,
                                               uint16_t valueA,
                                               uint16_t valueB) {
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev.addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, valueA & 0xff, ACK_VAL);
  i2c_master_write_byte(cmd, valueA >> 8, NACK_VAL);
  i2c_master_write_byte(cmd, valueB & 0xff, ACK_VAL);
  i2c_master_write_byte(cmd, valueB >> 8, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(dev.i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}
