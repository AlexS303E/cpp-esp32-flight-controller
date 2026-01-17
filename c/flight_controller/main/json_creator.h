#ifndef JSON_CREATOR_H
#define JSON_CREATOR_H

#include "sensor_data.h"

char* create_sensor_json(const sensor_data_t* data);
char* create_simple_json(const sensor_data_t* data);

#endif // JSON_CREATOR_H