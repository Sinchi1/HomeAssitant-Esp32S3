#include "BtHeader.h"

// Structure of advertisement service data without uuid
// #define SERVICE_DATA_LEN 16
// static uint8_t service_data[SERVICE_DATA_LEN] = {
//   0x40,
//   0x01, [> Battery <]
//   0x00,
//   0x02, [> Temperature <]
//   0x00, [> Low byte <]
//   0x00, [> High byte <]
//   0x03, [> Humidity <]
//   0x00, [> Low byte <]
//   0x00, [> High byte <]
//   0x04, [> Pressure <]
//   0x00, [> Low byte <]
//   0x00,
//   0x00, [> High byte <]
//   0x12, [> CO2 <]
//   0x00, [> Low byte <]
//   0x00, [> High byte <]
// };

static NimBLEScan* pScan = nullptr;

class ClientCallbacks : public NimBLEClientCallbacks
{
    void onConnect(NimBLEClient* pClient) override{
        ESP_LOGD(TAG, "connected to: %s", pClient->getPeerAddress().toString().c_str());
    }
    void onDisconnect(NimBLEClient* pClient, int reason) override{
        ESP_LOGD(TAG, "%s disconnected, reason = %d", pClient->getPeerAddress().toString().c_str(),
                 reason);
    }
} static clientCallbacks;

class ScanCallbacks : public NimBLEScanCallbacks{

    EventGroupHandle_t event_bits_;
    QueueHandle_t      adv_dev_queue_;

public:
    ScanCallbacks(EventGroupHandle_t event_bits, QueueHandle_t adv_dev_queue)
        : event_bits_(event_bits), adv_dev_queue_(adv_dev_queue)
    {}

    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override
    {
        ESP_LOGD(TAG, "Advertised Device found: %s", advertisedDevice->toString().c_str());
        ESP_LOGD(TAG, "data_count=%d", advertisedDevice->getServiceDataCount());

        const std::string dev_name = advertisedDevice->getName();
        if (dev_name.find("ES_") == std::string::npos) {
            ESP_LOGD(TAG, "Found unknown device: %s", dev_name.c_str());
            return;
        }

        for (size_t i = 0; i < advertisedDevice->getServiceDataCount(); i++) {
            NimBLEUUID uuid = advertisedDevice->getServiceDataUUID(i);
            if (uuid == NimBLEUUID(BLE_ADV_SERVICE_UUID)) {
                ESP_LOGI(TAG, "Found our service data from: %s", dev_name.c_str());
                std::string service_data = advertisedDevice->getServiceData(i);
                if (service_data.size() < 16) {
                    ESP_LOGW(TAG, "Service data too short: %d bytes", int(service_data.size()));
                    break;
                }
                const uint8_t* data = reinterpret_cast<const uint8_t*>(service_data.data());
                ESP_LOG_BUFFER_HEXDUMP(TAG, data, service_data.size(), ESP_LOG_DEBUG);

                const uint8_t* new_data = data; 

                if (xQueueSend(g_sensor_queue, &new_data, 0) == pdTRUE) {
                ESP_LOGI(TAG, "temp to ui %s", dev_name.c_str());
                } else {
                    ESP_LOGW(TAG, "Failed to send to UI queue (full?)");
                }

                uint8_t battery = data[IDX_BATT];
                int16_t temperature = (int16_t(data[IDX_TEMPH]) << 8) | data[IDX_TEMPL];
                uint16_t humidity = (uint16_t(data[IDX_HUMH]) << 8) | data[IDX_HUML];
                uint32_t pressure =
                    (uint32_t(data[IDX_PRESSUREH]) << 16) |
                    (uint32_t(data[IDX_PRESSUREL+1]) << 8) |
                    data[IDX_PRESSUREL];
                uint16_t co2 = (uint16_t(data[IDX_CO2H]) << 8) | data[IDX_CO2L];

                time_t t = time(NULL);

                ESP_LOGI(TAG, "Device=[%s] t=%ld battery=%u temp=%.2f°C hum=%.2f%% pres=%lu co2=%u",
                    dev_name.c_str(), long(t), battery,
                    float(temperature)/100, float(humidity)/100, pressure, co2);

                EnvironmentalSensor::Flags flags;
                flags.set_source(EnvironmentalSensor::Source::BLE);

                Aggregator::instance().addDevice(dev_name);
                Aggregator::instance().addBatteryData(dev_name, battery);
                Aggregator::instance().addTemperatureData(dev_name,
                    { .timestamp=uint32_t(t), .flags=flags, .value=float(temperature)/100 });
                Aggregator::instance().addHumidityData(dev_name,
                    { .timestamp=uint32_t(t), .flags=flags, .value=float(humidity)/100 });
                Aggregator::instance().addPressureData(dev_name,
                    { .timestamp=uint32_t(t), .flags=flags, .value=float(pressure) });
                Aggregator::instance().addCO2Data(dev_name,
                    { .timestamp=uint32_t(t), .flags=flags, .value=float(co2) });

                break; 
            }
        }

        NimBLEAdvertisedDevice* advCopy = new NimBLEAdvertisedDevice(*advertisedDevice);
        BaseType_t sendResult = xQueueSend(adv_dev_queue_, &advCopy, pdMS_TO_TICKS(1000));
        if (sendResult != pdPASS) {
            ESP_LOGW(TAG, "Failed to enqueue advertised device, queue full?");
            delete advCopy;
        } else {
            ESP_LOGI(TAG, "Advertised device enqueued: %s", dev_name.c_str());
        }
    }

    void onScanEnd(const NimBLEScanResults& results, int reason) override
    {
        ESP_LOGI(TAG, "Scan ended callback, reason: %d, count: %d", reason, results.getCount());
        xEventGroupSetBits(event_bits_, BLE_EVENT_SCANNER_STOPPED_MSK);
    }
};


static NimBLEClient* connectToDevice(NimBLEAdvertisedDevice* advDevice){
    NimBLEClient* pClient = nullptr;
    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getCreatedClientCount())
    {
        /**
         *  Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service
         * database. This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if (pClient)
        {
            if (!pClient->connect(advDevice, false))
            {
                ESP_LOGW(TAG, "reconnect failed");
                return nullptr;
            }
            pClient->setConnectTimeout(DEVICE_CONNECT_TIMEOUT_MS);
            ESP_LOGD(TAG, "reconnected client");
        } else
        {
            /**
             *  We don't already have a client that knows this device,
             *  check for a client that is disconnected that we can use.
             */
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if (!pClient)
    {
        if (NimBLEDevice::getCreatedClientCount() >= NIMBLE_MAX_CONNECTIONS)
        {
            ESP_LOGW(TAG, "Max clients reached - no more connections available");
            return nullptr;
        }

        pClient = NimBLEDevice::createClient();

        ESP_LOGD(TAG, "new client created");

        pClient->setClientCallbacks(&clientCallbacks, false);
        /**
         *  Set initial connection parameters:
         *  These settings are safe for 3 clients to connect reliably, can go
         * faster if you have less connections. Timeout should be a multiple of
         * the interval, minimum is 100ms. Min interval: 12 * 1.25ms = 15, Max
         * interval: 12 * 1.25ms = 15, 0 latency, 150 * 10ms = 1500ms timeout
         */
        pClient->setConnectionParams(12, 12, 0, 150);
        pClient->setConnectTimeout(DEVICE_CONNECT_TIMEOUT_MS);
        if (!pClient->connect(advDevice))
        {
            /** Created a client but failed to connect, don't need to keep it as
             * it has no data */
            NimBLEDevice::deleteClient(pClient);
            ESP_LOGW(TAG, "failed to connect, deleted client");
            return nullptr;
        }
    }

    if (!pClient->isConnected())
    {
        if (!pClient->connect(advDevice))
        {
            ESP_LOGW(TAG, "failed to connect");
            return nullptr;
        }
    }
    return pClient;
}

static bool reachChr(NimBLEClient* pClient, const std::string& dev_name)
{
    NimBLERemoteService* pSvc = pClient->getService(BLE_ESS_SERVICE_UUID);
    if (pSvc)
    {
        NimBLERemoteCharacteristic* pVOCChar = pSvc->getCharacteristic(BLE_VOC_CHAR_UUID);
        uint16_t                    voc      = 0;
        uint16_t                    iaq      = 0;
        if (pVOCChar)
        {
            if (pVOCChar->canRead())
            {
                const NimBLEAttValue attr_val = pVOCChar->readValue();
                ESP_LOGI(TAG, "voc, %s:", pVOCChar->getUUID().toString().c_str());
                ESP_LOG_BUFFER_HEXDUMP(TAG, attr_val.data(), attr_val.size(), ESP_LOG_INFO);
                if (attr_val.size() == sizeof(voc))
                {
                    memcpy(&voc, attr_val.data(), attr_val.size());
                } else
                {
                    ESP_LOGW(TAG, "unable to read value");
                }
            }
        } else
        {
            ESP_LOGW(TAG, "voc characteristic not found");
        }
        NimBLERemoteCharacteristic* pIAQ_Chr = pSvc->getCharacteristic(BLE_IAQ_CHAR_UUID);
        if (pIAQ_Chr)
        {
            if (pIAQ_Chr->canRead())
            {
                const NimBLEAttValue attr_val = pIAQ_Chr->readValue();
                ESP_LOGI(TAG, "IAQ, %s:", pIAQ_Chr->getUUID().toString().c_str());
                ESP_LOG_BUFFER_HEXDUMP(TAG, attr_val.data(), attr_val.size(), ESP_LOG_INFO);
                if (attr_val.size() == sizeof(iaq))
                {
                    memcpy(&iaq, attr_val.data(), attr_val.size());
                } else
                {
                    ESP_LOGW(TAG, "unable to read value");
                }
            }
        } else
        {
            ESP_LOGW(TAG, "IAQ characteristic not found");
        }

        EnvironmentalSensor::Flags flags;
        flags.set_source(EnvironmentalSensor::Source::BLE);
        time_t   time_val = time(NULL);
        uint32_t t        = (uint32_t)time_val;
        ESP_LOGI(TAG, "t=%lu, voc=%u, iaq=%u", t, voc, iaq);
        pClient->disconnect();
    } else
    {
        ESP_LOGW(TAG, "ESS service not found");
        return false;
    }

    return true;
}

void BLE::ble_scan_task(void* pvParameters)
{
    BLE*             ble           = static_cast<BLE*>(pvParameters);
    const TickType_t xFrequency    = pdMS_TO_TICKS(DEVICE_ADV_UPDATE_PERIOD_MS);
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        xSemaphoreTake(*(ble->radioMutex), portMAX_DELAY);

        ESP_LOGD(TAG, "start scan");
        if (pScan->start(DEVICE_ADV_UPDATE_PERIOD_MS, false, true))
        {
            ESP_LOGI(TAG, "scan started successfully");
            ESP_LOGD(TAG, "wait until the scan stopped");
            EventBits_t xBits = xEventGroupWaitBits(
                ble->ble_event_bits, BLE_EVENT_SCANNER_STOPPED_MSK, pdTRUE, pdTRUE, portMAX_DELAY);
            ESP_LOGD(TAG, "found %u devices", uxQueueMessagesWaiting(ble->ble_adv_dev_queue));
            xBits = xEventGroupWaitBits(ble->ble_event_bits, BLE_EVENT_CLIENT_READY_MSK, pdTRUE,
                                        pdTRUE, 0);
            if (xBits & BLE_EVENT_CLIENT_READY_MSK)
            {
                xEventGroupSetBits(ble->ble_event_bits, BLE_EVENT_CLIENT_START_MSK);
                ESP_LOGD(TAG, "wait until we finish with the clients");
                xBits = xEventGroupWaitBits(ble->ble_event_bits, BLE_EVENT_CLIENT_STOPPED_MSK,
                                            pdTRUE, pdTRUE, portMAX_DELAY);
            } else
            {
                xQueueReset(ble->ble_adv_dev_queue);
            }
        } else
        {
            ESP_LOGW(TAG, "failed to start scan");
        }

        xSemaphoreGive(*(ble->radioMutex));

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void BLE::ble_connect_task(void* pvParameters)
{
    BLE* ble          = static_cast<BLE*>(pvParameters);
    bool is_completed = false;

    for(;;)
    {
        xEventGroupSetBits(ble->ble_event_bits, BLE_EVENT_CLIENT_READY_MSK);
        ESP_LOGI(TAG, "Waiting for device from scanner...");
        xEventGroupWaitBits(
            ble->ble_event_bits,
            BLE_EVENT_CLIENT_START_MSK, pdTRUE, pdTRUE, portMAX_DELAY
        );

        NimBLEAdvertisedDevice* advDevice = nullptr;
        while (xQueueReceive(ble->ble_adv_dev_queue, &advDevice, pdMS_TO_TICKS(100))) {
            if (!advDevice) continue;

            std::string dev_name = advDevice->getName();
            if (dev_name.find("ES_") == std::string::npos) {
                ESP_LOGD(TAG, "Unknown device in connect task: %s", dev_name.c_str());
                delete advDevice;
                continue;
            }

            ESP_LOGI(TAG, "Connecting to device: %s", dev_name.c_str());
            NimBLEClient* pClient = nullptr;
            uint16_t try_counter = 0;
            do {
                ESP_LOGI(TAG, "Connection try: %d", try_counter);
                pClient = connectToDevice(advDevice);
            } while (!pClient && (++try_counter) < BLE_CONNECT_ATTEMPT_COUNT);

            if (pClient) {
                auto res = ble->known_devices.insert(dev_name);
                reachChr(pClient, dev_name);
                is_completed = true;
            } else {
                ESP_LOGW(TAG, "Failed to connect to: %s", dev_name.c_str());
            }
            ESP_LOGI(TAG, "Done with the client: %s", dev_name.c_str());

            delete advDevice; 
        }

        xEventGroupSetBits(ble->ble_event_bits, BLE_EVENT_CLIENT_STOPPED_MSK);

        if (is_completed) {
            vTaskDelay(pdMS_TO_TICKS(DEVICE_CHAR_UPDATE_PERIOD_MS));
            is_completed = false;
        }
    }
    vTaskDelete(NULL);
}


void BLE::init(SemaphoreHandle_t* radioMutex_) {
    radioMutex = radioMutex_;

    ble_adv_dev_queue = xQueueCreate(DEVICE_NUM, sizeof(NimBLEAdvertisedDevice*));
    ble_event_bits = xEventGroupCreate();

    NimBLEDevice::init("NimBLE-Client");

    pScan = NimBLEDevice::getScan();
    if (!pScan) {
        ESP_LOGE(TAG, "Failed to get NimBLE scan object");
        return;
    }

    static ScanCallbacks scanCallback(ble_event_bits, ble_adv_dev_queue);
    pScan->setScanCallbacks(&scanCallback, false);

    pScan->setInterval(100);
    pScan->setWindow(100);
    pScan->setActiveScan(true);

    NimBLEDevice::init("NimBLE-Client");

    pScan = NimBLEDevice::getScan();
    if (!pScan) {
        ESP_LOGE(TAG, "Failed to get NimBLE scan object");
        return;
    }

    BaseType_t ret;

    ret = xTaskCreate(
        ble_scan_task,
        "ble_scan_task",
        8192,
        this,
        3,
        nullptr
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ble_scan_task! Return code: %d", (int)ret);
        return;
    }
    ESP_LOGI(TAG, "ble_scan_task created successfully");

    ret = xTaskCreate(
        ble_connect_task,
        "ble_connect_task",
        8192,
        this,
        3,
        nullptr
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ble_connect_task! Return code: %d", (int)ret);
        return;
    }
    ESP_LOGI(TAG, "ble_connect_task created successfully");
}