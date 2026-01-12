#include <stdio.h>
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "cJSON.h"

static const char *TAG = "FLIGHT_JSON";

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

// Глобальный буфер для JSON
static char json_buffer[2048];

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

// Функция записи в регистр
esp_err_t i2c_write_register(uint8_t dev_addr, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Функция чтения из регистров
esp_err_t i2c_read_registers(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Инициализация MPU6050
esp_err_t mpu6050_init_simple(void) {
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

// Инициализация QMC5883L
esp_err_t qmc5883l_init_simple(void) {
    ESP_LOGI(TAG, "Инициализация QMC5883L...");
    
    esp_err_t err = i2c_write_register(QMC5883L_ADDR, QMC5883L_CTRL1, 0x1D);
    if (err != ESP_OK) return err;
    
    err = i2c_write_register(QMC5883L_ADDR, QMC5883L_CTRL2, 0x00);
    if (err != ESP_OK) return err;
    
    ESP_LOGI(TAG, "QMC5883L инициализирован");
    return ESP_OK;
}

// Чтение данных MPU6050
esp_err_t mpu6050_read_data(float *accel, float *gyro, float *temp) {
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

// Чтение данных QMC5883L
esp_err_t qmc5883l_read_data(int16_t *mag, bool *overflow) {
    uint8_t data[7];
    
    esp_err_t err = i2c_read_registers(QMC5883L_ADDR, QMC5883L_STATUS, data, 7);
    if (err != ESP_OK) return err;
    
    uint8_t status = data[0];
    *overflow = (status & 0x40) != 0;
    
    if ((status & 0x01) == 0) return ESP_ERR_NOT_FINISHED;
    
    mag[0] = (int16_t)((data[2] << 8) | data[1]);
    mag[1] = (int16_t)((data[4] << 8) | data[3]);
    mag[2] = (int16_t)((data[6] << 8) | data[5]);
    
    return ESP_OK;
}

// Расчет углов наклона
void calculate_angles(float accel[3], float *pitch, float *roll) {
    *pitch = atan2(-accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2])) * 180.0 / M_PI;
    *roll = atan2(accel[1], accel[2]) * 180.0 / M_PI;
}

// Расчет курса
float calculate_heading(int16_t mag[3], float pitch, float roll) {
    float pitch_rad = pitch * M_PI / 180.0;
    float roll_rad = roll * M_PI / 180.0;
    
    float mag_x = mag[0] * cos(pitch_rad) + mag[2] * sin(pitch_rad);
    float mag_y = mag[0] * sin(roll_rad) * sin(pitch_rad) + 
                  mag[1] * cos(roll_rad) - 
                  mag[2] * sin(roll_rad) * cos(pitch_rad);
    
    float heading = atan2(mag_y, mag_x) * 180.0 / M_PI;
    
    if (heading < 0) heading += 360.0;
    return heading;
}

// Определение направления
const char* get_direction(float heading) {
    if (heading >= 337.5 || heading < 22.5) return "N";
    if (heading >= 22.5 && heading < 67.5) return "NE";
    if (heading >= 67.5 && heading < 112.5) return "E";
    if (heading >= 112.5 && heading < 157.5) return "SE";
    if (heading >= 157.5 && heading < 202.5) return "S";
    if (heading >= 202.5 && heading < 247.5) return "SW";
    if (heading >= 247.5 && heading < 292.5) return "W";
    return "NW";
}

// Создание JSON из данных датчиков
char* create_sensor_json(const sensor_data_t *data) {
    cJSON *root = cJSON_CreateObject();
    
    // Добавляем счетчик и метку времени
    cJSON_AddNumberToObject(root, "counter", data->counter);
    cJSON_AddNumberToObject(root, "timestamp", data->timestamp);
    
    // Акселерометр
    cJSON *accel = cJSON_CreateObject();
    cJSON_AddNumberToObject(accel, "x", data->accel.x);
    cJSON_AddNumberToObject(accel, "y", data->accel.y);
    cJSON_AddNumberToObject(accel, "z", data->accel.z);
    cJSON_AddItemToObject(root, "accel", accel);
    
    // Гироскоп
    cJSON *gyro = cJSON_CreateObject();
    cJSON_AddNumberToObject(gyro, "x", data->gyro.x);
    cJSON_AddNumberToObject(gyro, "y", data->gyro.y);
    cJSON_AddNumberToObject(gyro, "z", data->gyro.z);
    cJSON_AddItemToObject(root, "gyro", gyro);
    
    // Магнитометр
    cJSON *mag = cJSON_CreateObject();
    cJSON_AddNumberToObject(mag, "x", data->mag.x);
    cJSON_AddNumberToObject(mag, "y", data->mag.y);
    cJSON_AddNumberToObject(mag, "z", data->mag.z);
    cJSON_AddBoolToObject(mag, "overflow", data->mag.overflow);
    cJSON_AddItemToObject(root, "mag", mag);
    
    // Углы
    cJSON *angles = cJSON_CreateObject();
    cJSON_AddNumberToObject(angles, "pitch", data->angles.pitch);
    cJSON_AddNumberToObject(angles, "roll", data->angles.roll);
    cJSON_AddNumberToObject(angles, "yaw", data->angles.yaw);
    cJSON_AddStringToObject(angles, "direction", data->angles.direction);
    cJSON_AddItemToObject(root, "angles", angles);
    
    // Температура
    cJSON_AddNumberToObject(root, "temperature", data->temperature);
    
    // Конвертируем в строку
    char *json_str = cJSON_PrintUnformatted(root);
    
    // Копируем в буфер
    strncpy(json_buffer, json_str, sizeof(json_buffer) - 1);
    json_buffer[sizeof(json_buffer) - 1] = '\0';
    
    // Освобождаем память
    free(json_str);
    cJSON_Delete(root);
    
    return json_buffer;
}

// Альтернатива: простой JSON без cJSON
char* create_simple_json(const sensor_data_t *data) {
    snprintf(json_buffer, sizeof(json_buffer),
        "{\"counter\":%lu,\"timestamp\":%lu,"
        "\"accel\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
        "\"gyro\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f},"
        "\"mag\":{\"x\":%d,\"y\":%d,\"z\":%d,\"overflow\":%s},"
        "\"angles\":{\"pitch\":%.1f,\"roll\":%.1f,\"yaw\":%.1f,\"direction\":\"%s\"},"
        "\"temperature\":%.1f}",
        data->counter,
        data->timestamp,
        data->accel.x, data->accel.y, data->accel.z,
        data->gyro.x, data->gyro.y, data->gyro.z,
        data->mag.x, data->mag.y, data->mag.z,
        data->mag.overflow ? "true" : "false",
        data->angles.pitch, data->angles.roll, data->angles.yaw,
        data->angles.direction,
        data->temperature
    );
    
    return json_buffer;
}

void app_main(void) {
    ESP_LOGI(TAG, "=== Flight Controller JSON Output ===");
    
    // Инициализация I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "I2C инициализирован");
    
    // Инициализация датчиков
    if (mpu6050_init_simple() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 не инициализирован");
        return;
    }
    
    if (qmc5883l_init_simple() != ESP_OK) {
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
            } else {
                data.mag.x = 0;
                data.mag.y = 0;
                data.mag.z = 0;
                data.mag.overflow = true;
                data.angles.yaw = 0.0;
                data.angles.direction = "N/A";
            }
            
            // Создаем JSON
            char *json_str = create_simple_json(&data);
            
            // Выводим JSON
            ESP_LOGI(TAG, "%s", json_str);
            
        } else {
            ESP_LOGW(TAG, "Ошибка чтения датчиков: MPU=%d, MAG=%d", err_mpu, err_mag);
        }
        
        // Задержка 100 мс (10 Гц)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}