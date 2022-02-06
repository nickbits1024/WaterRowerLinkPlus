/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This demo showcases creating a GATT database using a predefined attribute table.
* It acts as a GATT server and can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
* Client demo will enable GATT server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"
#include "state_manager.h"
#include "heart_rate_manager.h"
#include "antplus.h"
#include "usb.h"
#include "s4.h"
#include "ble.h"
#include "main.h"

#define TAG "main"

//extern "C"
void app_main(void)
{
    antplus_handle_t antplus_handle;
    s4_handle_t s4_handle;
    hrm_handle_t hrm_handle;
    ble_handle_t ble_handle;
    state_manager_handle_t sm_handle;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(s4_preinit());
    ESP_ERROR_CHECK(state_manager_init(&sm_handle));
    ESP_ERROR_CHECK(hrm_init(sm_handle, &hrm_handle));
    ESP_ERROR_CHECK(usb_init());
    ESP_ERROR_CHECK(s4_init(sm_handle, hrm_handle, &s4_handle));
    ESP_ERROR_CHECK(ble_init(hrm_handle, s4_handle, &ble_handle));
    ESP_ERROR_CHECK(antplus_init(sm_handle, hrm_handle, s4_handle, &antplus_handle));

}
