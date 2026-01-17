#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensor_data.h"
#include "i2c_manager.h"
#include "mpu6050.h"
#include "qmc5883l.h"
#include "sensor_fusion.h"
#include "json_creator.h"

static const char* TAG = "FLIGHT_CONTROLLER";

void app_main(void) {
    ESP_LOGI(TAG, "=== Flight Controller JSON Output ===");

    // Инициализация I2C
    if (i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка инициализации I2C");
        return;
    }

    // Инициализация датчиков
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 не инициализирован");
        return;
    }

    if (qmc5883l_init() != ESP_OK) {
        ESP_LOGE(TAG, "QMC5883L не инициализирован");
        return;
    }

    ESP_LOGI(TAG, "Все датчики инициализированы");
    ESP_LOGI(TAG, "Начинаю отправку JSON данных...");

    sensor_data_t data;
    uint32_t counter = 0;

    float accel[3], gyro[3];
    int16_t mag[3];
    bool overflow;
    float temperature;

    while (1) {
        counter++;

        // Чтение данных
        esp_err_t err_mpu = mpu6050_read_data(accel, gyro, &temperature);
        esp_err_t err_mag = qmc5883l_read_data(mag, &overflow);

        if (err_mpu == ESP_OK) {
            // Заполняем структуру данных
            data.counter = counter;
            data.timestamp = esp_log_timestamp();

            data.accel.x = accel[0];
            data.accel.y = accel[1];
            data.accel.z = accel[2];

            data.gyro.x = gyro[0];
            data.gyro.y = gyro[1];
            data.gyro.z = gyro[2];

            data.temperature = temperature;

            // Расчет углов наклона
            calculate_angles(accel, &data.angles.pitch, &data.angles.roll);

            // Расчет курса если магнитометр доступен
            if (err_mag == ESP_OK && !overflow) {
                data.mag.x = mag[0];
                data.mag.y = mag[1];
                data.mag.z = mag[2];
                data.mag.overflow = overflow;

                data.angles.yaw = calculate_heading(mag, data.angles.pitch, data.angles.roll);
                data.angles.direction = get_direction(data.angles.yaw);
            }
            else {
                data.mag.x = 0;
                data.mag.y = 0;
                data.mag.z = 0;
                data.mag.overflow = true;
                data.angles.yaw = 0.0;
                data.angles.direction = "N/A";
            }

            // Создаем JSON
            char* json_str = create_simple_json(&data);

            // Выводим JSON
            ESP_LOGI(TAG, "%s", json_str);

        }
        else {
            ESP_LOGW(TAG, "Ошибка чтения датчиков: MPU=%d, MAG=%d", err_mpu, err_mag);
        }

        // Задержка 100 мс (10 Гц)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}