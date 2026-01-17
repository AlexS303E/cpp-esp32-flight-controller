#include <stdio.h>
#include <string.h>
#include "json_creator.h"
#include "cJSON.h"

static char json_buffer[2048];

char* create_sensor_json(const sensor_data_t* data) {
    cJSON* root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "counter", data->counter);
    cJSON_AddNumberToObject(root, "timestamp", data->timestamp);

    cJSON* accel = cJSON_CreateObject();
    cJSON_AddNumberToObject(accel, "x", data->accel.x);
    cJSON_AddNumberToObject(accel, "y", data->accel.y);
    cJSON_AddNumberToObject(accel, "z", data->accel.z);
    cJSON_AddItemToObject(root, "accel", accel);

    cJSON* gyro = cJSON_CreateObject();
    cJSON_AddNumberToObject(gyro, "x", data->gyro.x);
    cJSON_AddNumberToObject(gyro, "y", data->gyro.y);
    cJSON_AddNumberToObject(gyro, "z", data->gyro.z);
    cJSON_AddItemToObject(root, "gyro", gyro);

    cJSON* mag = cJSON_CreateObject();
    cJSON_AddNumberToObject(mag, "x", data->mag.x);
    cJSON_AddNumberToObject(mag, "y", data->mag.y);
    cJSON_AddNumberToObject(mag, "z", data->mag.z);
    cJSON_AddBoolToObject(mag, "overflow", data->mag.overflow);
    cJSON_AddItemToObject(root, "mag", mag);

    cJSON* angles = cJSON_CreateObject();
    cJSON_AddNumberToObject(angles, "pitch", data->angles.pitch);
    cJSON_AddNumberToObject(angles, "roll", data->angles.roll);
    cJSON_AddNumberToObject(angles, "yaw", data->angles.yaw);
    cJSON_AddStringToObject(angles, "direction", data->angles.direction);
    cJSON_AddItemToObject(root, "angles", angles);

    cJSON_AddNumberToObject(root, "temperature", data->temperature);

    char* json_str = cJSON_PrintUnformatted(root);

    strncpy(json_buffer, json_str, sizeof(json_buffer) - 1);
    json_buffer[sizeof(json_buffer) - 1] = '\0';

    free(json_str);
    cJSON_Delete(root);

    return json_buffer;
}

char* create_simple_json(const sensor_data_t* data) {
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