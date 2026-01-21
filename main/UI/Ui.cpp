#include "Header/Ui.h"

static TaskHandle_t lvgl_task_handle = NULL;

QueueHandle_t g_sensor_queue = nullptr;

lv_obj_t* UI_ESP::lbl_temp = nullptr; 
lv_obj_t* UI_ESP::lbl_hum = nullptr;
lv_obj_t* UI_ESP::lbl_bat = nullptr;
lv_obj_t* UI_ESP::lbl_pres = nullptr;

void UI_ESP::lvgl_init(lv_disp_t*  dispp, lv_theme_t* theme){

    ESP_LOGI("ui", "UI init start");

    lv_disp_set_theme(dispp, theme);
    ESP_LOGI("ui", "Theme applied");

    lv_obj_t* screen = lv_disp_get_scr_act(dispp);
    // lv_obj_t* label = lv_label_create(screen);
    // lv_label_set_text(label, "Hello esp!");
    // lv_obj_align(label, LV_ALIGN_CENTER,0,0);
    // ESP_LOGI("ui", "Static test label created");

    create_text_panel();
    ESP_LOGI("ui", "Text panel created");

    lvgl_port_unlock();
    ESP_LOGI("ui", "UI init finished, entering queue loop");

    for(;;){
        ESP_LOGI("ui", "Всё ок")
    }

    // const uint8_t* data = nullptr;
    // for (;;) {
    //     ESP_LOGD("ui", "Waiting for sensor data...");
    //     if (xQueueReceive(g_sensor_queue, &data, portMAX_DELAY) == pdTRUE && data, pdMS_TO_TICKS(1000)) {
    //         ESP_LOGI("ui", "Got data from queue, updating panel");
    //         if (lvgl_port_lock(pdMS_TO_TICKS(50))) {
    //             update_text_panel(data);
    //             lvgl_port_unlock();
    //             ESP_LOGD("ui", "UI updated successfully");
    //         } else {
    //             ESP_LOGW("ui", "lvgl_port_lock timeout in update");
    //         }
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }
}

void UI_ESP::create_text_panel()
{
    ESP_LOGI("ui", "Creating text panel labels...");

    lv_obj_t *scr = lv_disp_get_scr_act(nullptr);

    lbl_temp = lv_label_create(scr);
    lv_label_set_text(lbl_temp, "Temp: --.- °C");
    lv_obj_align(lbl_temp, LV_ALIGN_CENTER, 0, 10);
    ESP_LOGI("ui", "lbl_temp created");

    lbl_hum = lv_label_create(scr);
    lv_label_set_text(lbl_hum, "Hum : --.- %");
    lv_obj_align(lbl_hum, LV_ALIGN_CENTER, 0, 30);
    ESP_LOGI("ui", "lbl_hum created");

    lbl_bat = lv_label_create(scr);
    lv_label_set_text(lbl_bat, "Battery : --.- %");
    lv_obj_align(lbl_bat, LV_ALIGN_CENTER, 0, 50);
    ESP_LOGI("ui", "lbl_bat created");

    lbl_pres = lv_label_create(scr);
    lv_label_set_text(lbl_pres, "Pressure : --.- hPa");
    lv_obj_align(lbl_pres, LV_ALIGN_CENTER, 0, 70);
    ESP_LOGI("ui", "lbl_pres created");

    ESP_LOGI("ui", "Text panel created OK");
}

void UI_ESP::update_text_panel(const uint8_t *data){
    if (!data) {
        ESP_LOGW("ui", "update_text_panel called with null data");
        return;
    }

   
    uint8_t battery = data[IDX_BATT];
                int16_t temperature = (int16_t(data[IDX_TEMPH]) << 8) | data[IDX_TEMPL];
                uint16_t humidity = (uint16_t(data[IDX_HUMH]) << 8) | data[IDX_HUML];
                uint32_t pressure =
                    (uint32_t(data[IDX_PRESSUREH]) << 16) |
                    (uint32_t(data[IDX_PRESSUREL+1]) << 8) |
                    data[IDX_PRESSUREL];
                uint16_t co2 = (uint16_t(data[IDX_CO2H]) << 8) | data[IDX_CO2L];

    ESP_LOGI("ui", "Updating panel: batt=%u temp=%.2f hum=%.2f pres=%lu co2=%u",
             battery, float(temperature)/100, float(humidity)/100, pressure, co2);

        
    lv_label_set_text_fmt(lbl_temp, "Temp: %.2f °C", float(temperature)/100);
    lv_label_set_text_fmt(lbl_hum , "Hum : %.2f%%", float(humidity)/100);
    lv_label_set_text_fmt(lbl_bat, "Battery: %u%%", battery);
    lv_label_set_text_fmt(lbl_pres , "Pressure : %lu hPa", pressure);
    ESP_LOGI("ui", "Labels updated");
}