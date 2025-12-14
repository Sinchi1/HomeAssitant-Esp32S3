#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    float temperature;
    float humidity;
    int heart_rate;
} SensorData_t;

extern QueueHandle_t dataQueue;

void data_module_init(void);
