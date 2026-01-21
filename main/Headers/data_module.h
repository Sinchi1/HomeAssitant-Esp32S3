#pragma once

#include "EnvironmentalSensorData.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_spiffs.h"
#include "cJSON.h"

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <ctime>

#define IDX_BATT      2
#define IDX_TEMPL     4
#define IDX_TEMPH     5
#define IDX_HUML      7
#define IDX_HUMH      8
#define IDX_PRESSUREL 10
#define IDX_PRESSUREH 12
#define IDX_CO2L      14
#define IDX_CO2H      15

namespace DataModule {

using namespace EnvironmentalSensor;

static const char* Data_Module_Tag = "Module";

struct EnvironmentalData {
    TemperatureSample temperature;
    HumiditySample    humidity;
    PressureSample    pressure;
    CO2Sample         co2;
};

// Очередь для входящих данных
extern QueueHandle_t dataQueue;

// API модуля
void init(QueueHandle_t queue);
void task(void *pvParameters);

// Сохранение и загрузка
bool save_to_flash(const std::vector<EnvironmentalData>& data);
bool load_from_flash(std::vector<EnvironmentalData>& data);

} // namespace DataModule
