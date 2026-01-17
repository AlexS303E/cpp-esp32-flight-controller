#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>

void calculate_angles(float accel[3], float* pitch, float* roll);
float calculate_heading(int16_t mag[3], float pitch, float roll);
const char* get_direction(float heading);

#endif // SENSOR_FUSION_H