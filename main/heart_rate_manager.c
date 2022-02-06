#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "state_manager.h"
#include "heart_rate_manager.h"
#include "heart_rate_manager_int.h"

#define TAG "hrm"

static void hrm_heart_beat_task(void* param);

esp_err_t hrm_init(state_manager_handle_t sm_handle, hrm_handle_t* hrm_handle)
{
    heart_rate_driver_t* driver = malloc(sizeof(heart_rate_driver_t));
    memset(driver, 0, sizeof(heart_rate_driver_t));

    driver->sm_handle = sm_handle;
    portMUX_INITIALIZE(&driver->mux);
    // driver->sources[0].source_id = HRM_SOURCE_ANT;
    // driver->sources[1].source_id = HRM_SOURCE_BLE;
    // driver->sources[2].source_id = HRM_SOURCE_S4;

    xTaskCreate(hrm_heart_beat_task, "hrm_heart_beat_task", 4096, driver, 23, NULL);

    *hrm_handle = driver;

    return ESP_OK;
}

esp_err_t hrm_set_rate(hrm_handle_t hrm_handle, uint8_t source, uint8_t heart_rate)
{
    heart_rate_driver_t* driver = (heart_rate_driver_t*)hrm_handle;

    if (source >= HRM_NUM_SOURCES)
    {
        return ESP_FAIL;
    }

    bool set = false;

    portENTER_CRITICAL(&driver->mux);
    driver->sources[source].heart_rate = heart_rate;
    driver->sources[source].heart_rate_ts = esp_timer_get_time();
    portEXIT_CRITICAL(&driver->mux);

    //ESP_DRAM_LOGI(TAG, "set source %u hr %u", source, heart_rate);

    return ESP_OK;
}

esp_err_t hrm_get_rate(hrm_handle_t hrm_handle, uint8_t* heart_rate)
{
    heart_rate_driver_t* driver = (heart_rate_driver_t*)hrm_handle;

    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&driver->mux);
    uint8_t hr;
    int source = -1;

    for (int i = 0; i < HRM_NUM_SOURCES; i++)
    {
        if (now - driver->sources[i].heart_rate_ts < HRM_TIMEOUT * 1000000llu && driver->sources[i].heart_rate > 0)
        {
            hr = driver->sources[i].heart_rate;
            source = i;
            break;
        }
    }    

    portEXIT_CRITICAL(&driver->mux);

    if (source != -1)
    {
        *heart_rate = hr;
        //ESP_LOGI(TAG, "got hr %u from source %u", hr, source);        
    }
    else
    {
        *heart_rate = 0;
       
    }

    return ESP_OK;
}

static void hrm_heart_beat_task(void* param)
{
    heart_rate_driver_t* driver = (heart_rate_driver_t*)param;

    for (;;) 
    {
        uint8_t hr;
        if (hrm_get_rate(driver, &hr) == ESP_OK && hr > 0)
        {
            uint16_t hr_period = 60000 / hr;

            vTaskDelay(hr_period / 2 / portTICK_PERIOD_MS);
            state_manager_set_component_state(driver->sm_handle, STATE_MANAGER_COMPONENT_HR, true);
            
            vTaskDelay(hr_period / 2 / portTICK_PERIOD_MS);
            state_manager_set_component_state(driver->sm_handle, STATE_MANAGER_COMPONENT_HR, false);
        }
        else
        {
            state_manager_set_component_state(driver->sm_handle, STATE_MANAGER_COMPONENT_HR, false);

            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}
