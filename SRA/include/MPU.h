#ifndef MPU_H
#define MPU_H

#include <stdio.h>
#include <math.h>
#include <time.h>
#include "esp_system.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO    21   /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    22    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

#define MPU6050_ADDR  0x68    /*!< slave address for BH1750 sensor */
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define ALPHA 0.9834

#define RAD_TO_DEG 57.27272727

#define BUFF_SIZE 6
#define DELAY_TIME_BETWEEN_ITEMS_MS   5 /*!< delay time between different test items, dt should be small */

#define ACCE_START_ADD 0x3B
#define GYRO_START_ADD 0x43

esp_err_t mpu6050_init(i2c_port_t i2c_num);

esp_err_t mpu6050_read_acce(i2c_port_t i2c_num, uint8_t* data_rd, size_t size);

esp_err_t mpu6050_read_gyro(i2c_port_t i2c_num, uint8_t* data_rd, size_t size);

void i2c_master_init();

void disp_buf(int16_t* buf, int len);

void shift_buf(uint8_t* buf_1, int16_t* buf_2, int len);

uint32_t msec();

esp_err_t complimentory_filter(int16_t* acce_raw_value, int16_t* gyro_raw_value, float complimentary_angle[], int len, float initial_acce_angle);

void start_mpu();

void calculate_pitch_angle(uint8_t* acce_rd ,uint8_t* gyro_rd,int16_t* acce_raw_value,int16_t* gyro_raw_value, float initial_acce_angle, float complimentary_angle[2],float *pitch_angle);

#endif