#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"

esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read_data(float* accel, float* gyro, float* temp);

#endif // MPU6050_H