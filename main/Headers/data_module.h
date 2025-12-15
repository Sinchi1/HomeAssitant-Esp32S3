#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <vector>
#include "EnvironmentalSensorData.h"

namespace DataModule {

using namespace EnvironmentalSensor;

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
