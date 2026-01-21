#pragma once

#include "esp_display_panel.hpp"

#include "esp_log.h"

#include "lvgl_port_v8.h"

// #include "adapt/i2c_bus.h"

// #ifndef LVGL_PORT_TASK_CORE -1

#include "BtHeader.h"
#include "Aggregator.h"

#include <ctime>
#include <sys/time.h>

#define IDX_BATT 2       /* Index of battery data in service data*/
#define IDX_TEMPL 4      /* Index of lo byte of temp in service data*/
#define IDX_TEMPH 5      /* Index of hi byte of temp in service data*/
#define IDX_HUML 7       /* Index of lo byte of humidity in service data*/
#define IDX_HUMH 8       /* Index of hi byte of humidity in service data*/
#define IDX_PRESSUREL 10 /* Index of lo byte of pressure in service data*/
#define IDX_PRESSUREH 12 /* Index of hi byte of pressure in service data*/
#define IDX_CO2L 14      /* Index of lo byte of co2 in service data*/
#define IDX_CO2H 15      /* Index of hi byte of co2 in service data*/

static bool connected = false;

class UI_ESP{
  public:
    void lvgl_init(lv_disp_t*  dispp, lv_theme_t* theme);

    static UI_ESP& instance(){
        static UI_ESP instance;
        return instance;
    }
    ~UI_ESP() noexcept = default;

  private:
    UI_ESP() = default;
    static void lvgl_task();
    void create_text_panel();
    void update_text_panel(const uint8_t *data);

    static lv_obj_t *lbl_temp;
    static lv_obj_t *lbl_hum;
    static lv_obj_t *lbl_bat;
    static lv_obj_t *lbl_pres;

};
extern QueueHandle_t g_sensor_queue; 