#include "Aggregator.h"

static const char* TAG_AGGR = "Aggregator";

Aggregator& Aggregator::instance() {
    static Aggregator inst;
    return inst;
}

Aggregator::Aggregator() {
    mutex_ = xSemaphoreCreateRecursiveMutex();
    if (!mutex_) {
        ESP_LOGE(TAG_AGGR, "Failed to create mutex");
    }
}

Aggregator::~Aggregator() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

void Aggregator::lock() const {
    if (mutex_) {
        xSemaphoreTakeRecursive(mutex_, portMAX_DELAY);
    }
}

void Aggregator::unlock() const {
    if (mutex_) {
        xSemaphoreGiveRecursive(mutex_);
    }
}

void Aggregator::addDevice(const std::string& id) {
    lock();
    auto it = devices_.find(id);
    if (it == devices_.end()) {
        devices_.emplace(id, DeviceData{});
        ESP_LOGI(TAG_AGGR, "Device added: %s", id.c_str());
    }
    unlock();
}

bool Aggregator::hasDevice(const std::string& id) const {
    lock();
    bool exists = devices_.find(id) != devices_.end();
    unlock();
    return exists;
}

void Aggregator::addBatteryData(const std::string& id, uint8_t batteryPercent) {
    lock();
    auto& dev = devices_[id];
    dev.lastBattery = batteryPercent;
    unlock();
}

void Aggregator::addTemperatureData(const std::string& id, const EnvironmentalSensor::TemperatureSample& s) {
    lock();
    auto& dev = devices_[id];
    dev.temperature = s;
    unlock();
}

void Aggregator::addHumidityData(const std::string& id, const EnvironmentalSensor::HumiditySample& s) {
    lock();
    auto& dev = devices_[id];
    dev.humidity = s;
    unlock();
}

void Aggregator::addPressureData(const std::string& id, const EnvironmentalSensor::PressureSample& s) {
    lock();
    auto& dev = devices_[id];
    dev.pressure = s;
    unlock();
}

void Aggregator::addCO2Data(const std::string& id, const EnvironmentalSensor::CO2Sample& s) {
    lock();
    auto& dev = devices_[id];
    dev.co2 = s;
    unlock();
}


void Aggregator::setReadEntries(size_t count) {
    lock();
    lastReadEntries_ = count;
    unlock();
}

size_t Aggregator::getLastReadEntries() const {
    lock();
    size_t v = lastReadEntries_;
    unlock();
    return v;
}