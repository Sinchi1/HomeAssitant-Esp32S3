#include <cstdio>
#include <cstdlib>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_spiffs.h"
#include "cJSON.h"
#include "data_module.h"

using namespace EnvironmentalSensor;
using namespace DataModule;

static QueueHandle_t s_data_queue = nullptr;
static std::vector<EnvironmentalData> s_storage; 

static void init_fs()
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = nullptr,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
        printf("[SPIFFS] SPIFFS mounting error (%d)\n", ret);
    else
        printf("[SPIFFS] fs is ready.\n");
}

bool DataModule::save_to_flash(const std::vector<EnvironmentalData>& data)
{
    FILE *f = fopen("/spiffs/env_data.json", "w");
    if (!f) {
        printf("[DataModule] error opening file from flash\n");
        return false;
    }

    cJSON *root = cJSON_CreateArray();
    for (const auto &entry : data) {
        cJSON *obj = cJSON_CreateObject();
        cJSON_AddNumberToObject(obj, "temperature", entry.temperature.value);
        cJSON_AddNumberToObject(obj, "humidity", entry.humidity.value);
        cJSON_AddNumberToObject(obj, "pressure", entry.pressure.value);
        cJSON_AddNumberToObject(obj, "co2", entry.co2.value);
        cJSON_AddNumberToObject(obj, "timestamp", entry.temperature.timestamp);
        cJSON_AddItemToArray(root, obj);
    }

    char *json_str = cJSON_PrintUnformatted(root);
    fputs(json_str, f);
    fclose(f);
    free(json_str);
    cJSON_Delete(root);

    printf("[DataModule] data saved successfully (%zu entries)\n", data.size());
    return true;
}

bool DataModule::load_from_flash(std::vector<EnvironmentalData>& data)
{
    FILE *f = fopen("/spiffs/env_data.json", "r");
    if (!f) {
        printf("[DataModule] file not found (first boot?)\n");
        return false;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);

    std::vector<char> buffer(size + 1);
    fread(buffer.data(), 1, size, f);
    fclose(f);

    cJSON *root = cJSON_Parse(buffer.data());
    if (!root) {
        printf("[DataModule] json parse error\n");
        return false;
    }

    int count = cJSON_GetArraySize(root);
    for (int i = 0; i < count; ++i) {
        cJSON *obj = cJSON_GetArrayItem(root, i);
        EnvironmentalData entry;
        entry.temperature.value = cJSON_GetObjectItem(obj, "temperature")->valuedouble;
        entry.humidity.value    = cJSON_GetObjectItem(obj, "humidity")->valuedouble;
        entry.pressure.value    = cJSON_GetObjectItem(obj, "pressure")->valuedouble;
        entry.co2.value         = cJSON_GetObjectItem(obj, "co2")->valuedouble;
        entry.temperature.timestamp = cJSON_GetObjectItem(obj, "timestamp")->valueint;
        data.push_back(entry);
    }

    cJSON_Delete(root);
    printf("[DataModule]  Successfully loaded %d entries from flash storage\n", count);
    return true;
}

void DataModule::init(QueueHandle_t queue)
{
    if (!queue) {
        return;
    }
    s_data_queue = queue;

    init_fs();
    load_from_flash(s_storage);
}

void DataModule::task(void *pvParameters)
{
    if (s_data_queue == nullptr) {
        vTaskDelete(nullptr);
        return;
    }

    EnvironmentalData received{};

    for (;;)
    {
        if (xQueueReceive(s_data_queue, &received, portMAX_DELAY))
        {
            s_storage.push_back(received);

            printf("[DataModule] Got data: "
                   "T=%.2f°C, H=%.1f%%, P=%.1f, CO2=%.1f\n",
                   received.temperature.value,
                   received.humidity.value,
                   received.pressure.value,
                   received.co2.value);
            

            if (s_storage.size() % 10 == 0)
                save_to_flash(s_storage); //save every 10 records

            /* ------------------------------------------------------------------
             there will be recomendation module integration
             * ------------------------------------------------------------------ */
        }
    }
}