#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>
#include <stdbool.h>

// Константы I2C
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_SDA_IO        21
#define I2C_MASTER_SCL_IO        22
#define I2C_MASTER_FREQ_HZ       400000

// Адреса устройств
#define MPU6050_ADDR             0x68
#define QMC5883L_ADDR            0x0D

// Регистры MPU6050
#define MPU6050_ACCEL_XOUT_H     0x3B
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_WHO_AM_I         0x75
#define MPU6050_WHO_AM_I_VALUE   0x68

// Регистры QMC5883L
#define QMC5883L_X_LSB           0x00
#define QMC5883L_X_MSB           0x01
#define QMC5883L_Y_LSB           0x02
#define QMC5883L_Y_MSB           0x03
#define QMC5883L_Z_LSB           0x04
#define QMC5883L_Z_MSB           0x05
#define QMC5883L_STATUS          0x06
#define QMC5883L_CTRL1           0x09
#define QMC5883L_CTRL2           0x0A

// Структура для данных датчиков
typedef struct {
    struct {
        float x, y, z;
    } accel;

    struct {
        float x, y, z;
    } gyro;

    struct {
        int x, y, z;
        bool overflow;
    } mag;

    struct {
        float pitch, roll, yaw;
        const char* direction;
    } angles;

    float temperature;
    uint32_t timestamp;
    uint32_t counter;
} sensor_data_t;

#endif // SENSOR_DATA_H