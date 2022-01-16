#include "usb/usb_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define TAG "usb"

static void usb_host_lib_task(void* arg);

static void usb_host_lib_task(void* arg)
{
    for (;;)
    {
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, NULL));
    }
}

esp_err_t usb_init()
{
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    xTaskCreate(usb_host_lib_task, "usb_host_lib_task", 4096, NULL, 23, NULL);

    //xSemaphoreTake(usb_ready_sem, portMAX_DELAY);

    return ESP_OK;
}
