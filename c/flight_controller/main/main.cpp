#include <stdio.h>
#include "esp_log.h"

extern "C" {
    #include "cJSON.h"
}

// Определение тега для логирования (КЛЮЧЕВОЕ ИСПРАВЛЕНИЕ)
static const char *TAG = "JSON_DEMO";

// Точка входа должна быть объявлена с extern "C"
extern "C" void app_main(void) {
    // 1. Парсинг строки
    const char *json_string = "{\"name\":\"ESP32\", \"value\":42}";
    cJSON *root = cJSON_Parse(json_string);
    if (root == NULL) {
        ESP_LOGE(TAG, "Ошибка парсинга JSON");
        return;
    }

    // 2. Извлечение данных
    cJSON *name_item = cJSON_GetObjectItem(root, "name");
    cJSON *value_item = cJSON_GetObjectItem(root, "value");
    if (cJSON_IsString(name_item) && cJSON_IsNumber(value_item)) {
        ESP_LOGI(TAG, "Name: %s, Value: %d", name_item->valuestring, value_item->valueint);
    }

    // 3. Создание нового JSON
    cJSON *new_json = cJSON_CreateObject();
    cJSON_AddStringToObject(new_json, "sensor", "MPU6050");
    cJSON_AddNumberToObject(new_json, "x", 123);

    char *output = cJSON_Print(new_json);
    ESP_LOGI(TAG, "Созданный JSON: %s", output);

    // 4. Очистка памяти
    free(output);
    cJSON_Delete(new_json);
    cJSON_Delete(root);
}