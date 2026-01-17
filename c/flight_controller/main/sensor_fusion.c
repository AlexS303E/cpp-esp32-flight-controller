#include "sensor_fusion.h"
#include <math.h>

void calculate_angles(float accel[3], float* pitch, float* roll) {
    *pitch = atan2(-accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2])) * 180.0 / M_PI;
    *roll = atan2(accel[1], accel[2]) * 180.0 / M_PI;
}

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