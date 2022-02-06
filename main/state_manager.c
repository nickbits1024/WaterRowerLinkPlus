#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "led_strip.h"
#include "state_manager_int.h"
#include "state_manager.h"

#define TAG "state_manager"

esp_err_t state_manager_init(state_manager_handle_t* sm_handle)
{
    *sm_handle = NULL;

    state_manager_driver_t* driver = (state_manager_driver_t*)malloc(sizeof(state_manager_driver_t));
    memset(driver, 0, sizeof(state_manager_driver_t));

    portMUX_INITIALIZE(&driver->mux);

    driver->led_strip = led_strip_init(STATE_LED_CHANNEL, STATE_LED_GPIO, 1);
    ets_delay_us(5000);

    ESP_ERROR_CHECK(driver->led_strip->set_pixel(driver->led_strip, 0, 0, 0, 0));
    ESP_ERROR_CHECK(driver->led_strip->refresh(driver->led_strip, 100));

    *sm_handle = driver;

    return ESP_OK;
}

esp_err_t state_manager_set_component_state(state_manager_handle_t* sm_handle, uint8_t component, bool state)
{
    state_manager_driver_t* driver = (state_manager_driver_t*)sm_handle;

    uint8_t old_state;
    uint8_t new_state;

    portENTER_CRITICAL(&driver->mux);
    old_state = driver->state;
    if (state)
    {
        driver->state |= component;
    }
    else
    {
        driver->state &= ~component;
    }
    new_state = driver->state;
    portEXIT_CRITICAL(&driver->mux);

    if (old_state != new_state)
    {
        uint8_t red = 0, green = 0, blue = 0;

        bool hr = (new_state & STATE_MANAGER_COMPONENT_HR) != 0;
        bool s4 = (new_state & STATE_MANAGER_COMPONENT_S4) != 0;
        bool ant = (new_state & STATE_MANAGER_COMPONENT_ANT) != 0;

        if (hr && s4 && ant)
        {
            red = 255;
            green = 255;
            blue = 255;
        }
        else
        {
            red = hr ? 128 : 0;
            green = s4 ? 128 : 0;
            blue = ant ? 128 : 0;
        }

        if (old_state != new_state)
        {
            ESP_LOGI(TAG, "led r: %u g: %u b: %u", red, green, blue);

            ESP_ERROR_CHECK(driver->led_strip->set_pixel(driver->led_strip, 0, red, green, blue));
            ESP_ERROR_CHECK(driver->led_strip->refresh(driver->led_strip, 100));
        }
    }

    return ESP_OK;
}