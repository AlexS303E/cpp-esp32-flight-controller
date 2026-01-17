#ifndef QMC5883L_H
#define QMC5883L_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

esp_err_t qmc5883l_init(void);
esp_err_t qmc5883l_read_data(int16_t* mag, bool* overflow);

#endif // QMC5883L_H