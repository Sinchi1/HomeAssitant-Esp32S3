
#include "Main.h"

using namespace DataModule;
using namespace EnvironmentalSensor;

QueueHandle_t DataModule::dataQueue = nullptr;

extern "C" void app_main(void)
{
    int DATA_QUEUE_LENGTH = 120;
    dataQueue = xQueueCreate(DATA_QUEUE_LENGTH, sizeof(EnvironmentalData));

    assert(dataQueue != NULL);

    DataModule::init(dataQueue);

    ESP_LOGI(mainTag, "Starting BLE application");

    BLE& bleInstance = BLE::instance();

    radioMutex = xSemaphoreCreateMutex();

    if (radioMutex == nullptr) {
        ESP_LOGE(mainTag, "Failed to create radio mutex");
        return;
    }

    bleInstance.init(&radioMutex);


    
    // xTaskCreate(DataModule::task, "DataTask", 8192, nullptr, 5, nullptr);// SIMULATION
    // xTaskCreate([](void *pv){
    //     EnvironmentalData data;
    //     data.temperature.flags.set_source(Source::BLE);
    //     data.humidity.flags.set_source(Source::BLE);
    //     data.pressure.flags.set_source(Source::BLE);
    //     data.co2.flags.set_source(Source::BLE);

    //     for (;;) {
    //         data.temperature.value = 20.0f + (rand() % 100) / 10.0f;
    //         data.humidity.value    = 40.0f + (rand() % 600) / 10.0f;
    //         data.pressure.value    = 1000.0f + (rand() % 200) / 10.0f;
    //         data.co2.value         = 400.0f + (rand() % 300);
    //         data.temperature.timestamp = xTaskGetTickCount();

    //         xQueueSend(dataQueue, &data, portMAX_DELAY);
    //         vTaskDelay(pdMS_TO_TICKS(3000));
    //     }
    // }, "MockSensor", 8192, nullptr, 4, nullptr);
}
