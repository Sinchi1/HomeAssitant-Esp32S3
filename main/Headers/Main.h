#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "BtHeader.h"
#include "data_module.h"
#include "EnvironmentalSensorData.h"


static const char* mainTag = "Main"; 

static SemaphoreHandle_t radioMutex = nullptr;
