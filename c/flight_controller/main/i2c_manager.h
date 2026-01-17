#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

esp_err_t i2c_init(void);
esp_err_t i2c_write_register(uint8_t dev_addr, uint8_t reg, uint8_t value);
esp_err_t i2c_read_registers(uint8_t dev_addr, uint8_t reg, uint8_t* data, size_t len);

#endif // I2C_MANAGER_H