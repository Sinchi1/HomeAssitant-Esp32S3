#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "cJSON.h"
#include "data_module.h"
extern QueueHandle_t dataQueue;

void data_module_init(void) {
    dataQueue = xQueueCreate(10, sizeof(SensorData_t));
}

void data_task(void *pvParameters) {
    SensorData_t sensorData;
    for(;;) {
        if(xQueueReceive(dataQueue, &sensorData, portMAX_DELAY)) {
            cJSON *json = cJSON_CreateObject();
            cJSON_AddNumberToObject(json, "temperature", sensorData.temperature);
            cJSON_AddNumberToObject(json, "humidity", sensorData.humidity);
            cJSON_AddNumberToObject(json, "heart_rate", sensorData.heart_rate);

            char *json_str = cJSON_Print(json);
            printf("Data JSON: %s\n", json_str);

            // there will be sending to recommendation module logic
            // xQueueSend(recommendationQueue, &sensorData, portMAX_DELAY);

            cJSON_Delete(json);
            free(json_str);
        }
    }
}
