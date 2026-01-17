#include "qmc5883l.h"
#include "i2c_manager.h"
#include "sensor_data.h"
#include "esp_log.h"

static const char* TAG = "QMC5883L";

esp_err_t qmc5883l_init(void) {
    ESP_LOGI(TAG, "Инициализация QMC5883L...");

    esp_err_t err = i2c_write_register(QMC5883L_ADDR, QMC5883L_CTRL1, 0x1D);
    if (err != ESP_OK) return err;

    err = i2c_write_register(QMC5883L_ADDR, QMC5883L_CTRL2, 0x00);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "QMC5883L инициализирован");
    return ESP_OK;
}

esp_err_t qmc5883l_read_data(int16_t* mag, bool* overflow) {
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