#pragma once

#include <string>
#include <unordered_set>

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "portmacro.h"

#include "Aggregator.h"
#include "EnvironmentalSensorData.h"
#include "settings.h"

#include "NimBLEDevice.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
#include "esp_log_level.h"

#include <cstdint>
#include <cmath>
#include <sys/time.h>
#include <time.h>

#define DEVICE_CONNECT_TIMEOUT_MS     5000    //5 Sek
#define DEVICE_ADV_UPDATE_PERIOD_MS   30000   
#define BLE_CONNECT_ATTEMPT_COUNT     3       
#define DEVICE_HISTORY_QUERY_SIZE     72      
#define DEVICE_CHAR_UPDATE_PERIOD_MS  10000   
#define DEVICE_NUM                    10      
#define NIMBLE_MAX_CONNECTIONS        3      

#define BLE_EVENT_SCANNER_STOPPED_MSK BIT0
#define BLE_EVENT_CLIENT_READY_MSK BIT1
#define BLE_EVENT_CLIENT_START_MSK BIT2
#define BLE_EVENT_CLIENT_STOPPED_MSK BIT3

#define BLE_ADV_SERVICE_UUID "FCD2"
#define BLE_ESS_SERVICE_UUID "181A"
#define BLE_TIME_SERVICE_UUID "1805"
#define BLE_TIME_CHAR_UUID "2A2B"
#define BLE_VOC_CHAR_UUID "2BE7"
#define BLE_IAQ_CHAR_UUID "E2890598-1286-43D6-82BA-121248BDA7DA"
#define BLE_HIST_READ_UUID "E3890598-1286-43D6-82BA-121248BDA7DA"
#define BLE_HIST_CONF_UUID "E4890598-1286-43D6-82BA-121248BDA7DA"

#define IDX_BATT 2       /* Index of battery data in service data*/
#define IDX_TEMPL 4      /* Index of lo byte of temp in service data*/
#define IDX_TEMPH 5      /* Index of hi byte of temp in service data*/
#define IDX_HUML 7       /* Index of lo byte of humidity in service data*/
#define IDX_HUMH 8       /* Index of hi byte of humidity in service data*/
#define IDX_PRESSUREL 10 /* Index of lo byte of pressure in service data*/
#define IDX_PRESSUREH 12 /* Index of hi byte of pressure in service data*/
#define IDX_CO2L 14      /* Index of lo byte of co2 in service data*/
#define IDX_CO2H 15      /* Index of hi byte of co2 in service data*/

#define BLE_CYCLE_ENTRIES 24 // 480 byte per cycle

static const char* TAG = "ble";

struct HistorySensorMeasurement {
    uint32_t time_s;
    int32_t  pressure;
    int16_t  temperature;
    uint16_t co2;
    uint16_t voc;
    uint16_t iaq;
    uint16_t humidity;
    bool     is_time_set;
    bool     reserved;
};

// Interpret a 32-bit value as IDs, integer, or bytes (little-endian assumed)
struct bt_cts_time_format {
    uint16_t year;
    uint8_t  mon;
    uint8_t  mday;
    uint8_t  hours;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  wday;
    uint8_t  fractions256;
    uint8_t  reason;
} __packed;

class BLE
{
    std::unordered_set<std::string> known_devices;
    EventGroupHandle_t              ble_event_bits;
    QueueHandle_t                   ble_adv_dev_queue;
    static void                     ble_scan_task(void* pvParameters);
    static void                     ble_connect_task(void* pvParameters);
    SemaphoreHandle_t*              radioMutex;

public:
    static BLE& instance()
    {
        static BLE instance;
        return instance;
    }
    void init(SemaphoreHandle_t* radioMutex_);
};