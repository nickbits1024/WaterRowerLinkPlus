#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "heart_rate_manager.h"
#include "heart_rate_manager_int.h"

esp_err_t hrm_init(hrm_handle_t* hrm_handle)
{
    heart_rate_driver_t* driver = malloc(sizeof(heart_rate_driver_t));
    memset(driver, 0, sizeof(heart_rate_driver_t));

    portMUX_INITIALIZE(&driver->mux);

    *hrm_handle = driver;

    return ESP_OK;
}

esp_err_t hrm_set_rate(hrm_handle_t hrm_handle, int source, uint8_t heart_rate)
{
    heart_rate_driver_t* driver = (heart_rate_driver_t*)hrm_handle;

    if (source >= HRM_SOURCE_MAX)
    {
        return ESP_FAIL;
    }

    ets_printf("set source %u hr %u\n", source, heart_rate);

    portENTER_CRITICAL(&driver->mux);
    driver->sources[source].heart_rate = heart_rate;
    driver->sources[source].heart_rate_ts = esp_timer_get_time();
    portEXIT_CRITICAL(&driver->mux);

    return ESP_OK;
}

esp_err_t hrm_get_rate(hrm_handle_t hrm_handle, uint8_t* heart_rate)
{
    heart_rate_driver_t* driver = (heart_rate_driver_t*)hrm_handle;

    portENTER_CRITICAL(&driver->mux);
    uint8_t hr = driver->sources[0].heart_rate;
    int64_t ts = driver->sources[0].heart_rate_ts;
    int source = 0;

    for (int i = 1; i < HRM_SOURCE_MAX; i++)
    {
        if (driver->sources[i].heart_rate_ts < ts && driver->sources[i].heart_rate > 0)
        {
            hr = driver->sources[i].heart_rate;
            ts = driver->sources[i].heart_rate_ts;
            source = i;
        }
    }    

    portEXIT_CRITICAL(&driver->mux);

    if (esp_timer_get_time() - ts < HRM_TIMEOUT * 1000000llu)
    {
        *heart_rate = hr;
        printf("hr %u source %u\n", hr, source);
    }
    else
    {
        *heart_rate = 0;
    }

    return ESP_OK;
}