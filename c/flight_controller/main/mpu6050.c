#include "mpu6050.h"
#include "i2c_manager.h"
#include "sensor_data.h"
#include "esp_log.h"

static const char* TAG = "MPU6050";

esp_err_t mpu6050_init(void) {
    ESP_LOGI(TAG, "Инициализация MPU6050...");

    esp_err_t err = i2c_write_register(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) return err;

    uint8_t who_am_i;
    err = i2c_read_registers(MPU6050_ADDR, MPU6050_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) return err;

    if (who_am_i != MPU6050_WHO_AM_I_VALUE) return ESP_FAIL;

    ESP_LOGI(TAG, "MPU6050 обнаружен");
    return ESP_OK;
}

esp_err_t mpu6050_read_data(float* accel, float* gyro, float* temp) {
    uint8_t data[14];

    esp_err_t err = i2c_read_registers(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, 14);
    if (err != ESP_OK) return err;

    // Акселерометр
    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];

    // Температура
    int16_t raw_temp = (data[6] << 8) | data[7];
    *temp = raw_temp / 340.0 + 36.53;

    // Гироскоп
    int16_t gx = (data[8] << 8) | data[9];
    int16_t gy = (data[10] << 8) | data[11];
    int16_t gz = (data[12] << 8) | data[13];

    // Конвертация
    accel[0] = ax / 16384.0 * 9.81;
    accel[1] = ay / 16384.0 * 9.81;
    accel[2] = az / 16384.0 * 9.81;

    gyro[0] = gx / 131.0;
    gyro[1] = gy / 131.0;
    gyro[2] = gz / 131.0;

    return ESP_OK;
}