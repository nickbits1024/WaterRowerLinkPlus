#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "driver/gpio.h"
#include "heart_rate_manager.h"
#include "s4.h"
#include "s4_int.h"

#define TAG "waterrower"

s4_variable_t memory_map[] =
{
    [S4_DISTANCE] = { 0x055, S4_DOUBLE_VALUE, "distance" },
    //[S4_POWER] = { 0x088, S4_DOUBLE_VALUE, "power" },
    [S4_CALORIES] = { 0x08a, S4_TRIPLE_VALUE, "calories" },
    [S4_STROKE_COUNT] = { 0x140, S4_DOUBLE_VALUE, "stroke_count" },
    [S4_STROKE_AVERAGE] = { 0x142, S4_SINGLE_VALUE, "stroke_average" },
    [S4_CURRENT_SPEED] = { 0x14a, S4_DOUBLE_VALUE, "current_speed" },
    [S4_HEART_RATE] = { 0x1a0, S4_SINGLE_VALUE, "heart_rate" },
    [S4_500_PACE] = { 0x1a5, S4_DOUBLE_VALUE, "500_pace" },
    [S4_TIMER_SECOND_DEC] = { 0x1e0, S4_SINGLE_VALUE, "timer_second_dec" },
    [S4_TIMER_SECOND] = { 0x1e1, S4_SINGLE_VALUE, "timer_second" },
    [S4_TIMER_MINUTE] = { 0x1e2, S4_SINGLE_VALUE, "timer_minute" },
    [S4_TIMER_HOUR] = { 0x1e3, S4_SINGLE_VALUE, "timer_hour" }
};

static void s4_in_event(s4_driver_t* driver);
static esp_err_t s4_setup(s4_driver_t* driver);
static esp_err_t s4_shutdown(s4_driver_t* driver);
static void s4_read_memory(s4_driver_t* driver, const s4_variable_t* var);
static void s4_heart_beat_task(void* param);

static void s4_client_event_handler(const usb_host_client_event_msg_t* event_msg, void* arg)
{
    s4_driver_t* driver = (s4_driver_t*)arg;
    switch (event_msg->event)
    {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(TAG, "Device #%d connected", event_msg->new_dev.address);
            if (driver->dev_addr == 0)
            {
                driver->dev_addr = event_msg->new_dev.address;
                xSemaphoreGive(driver->device_sem);
            }
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(TAG, "Device #%d disconnected", driver->dev_addr);
            driver->dev_addr = 0;
            break;
        default:
            ESP_LOGE(TAG, "Unkown USB client event");
            abort();
            break;
    }
}

static esp_err_t s4_command(s4_driver_t* driver, const char* cmd, const char* resp)
{
    int length = strlen(cmd);
    if (length + 2 >= S4_MAX_PACKET_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(driver->cmd_sem, portMAX_DELAY);

    char* buffer = (char*)driver->out_transfer->data_buffer;
    driver->out_transfer->num_bytes = length + 2;
    memcpy(buffer, cmd, length);

    buffer[length] = 0;
    ESP_LOGD(TAG, "Command: %s", cmd);

    buffer[length] = '\r';
    buffer[length + 1] = '\n';

    if (resp == NULL)
    {
        resp = "OK";
    }

    strcpy(driver->wait_buffer, resp);
    driver->wait_buffer_size = strlen(resp);

    xSemaphoreGive(driver->out_sem);
    xSemaphoreTake(driver->out_resp_sem, portMAX_DELAY);

    return ESP_OK;
}

static void s4_usb_client_task(void* arg)
{
    s4_driver_t* driver = (s4_driver_t*)arg;

    for (;;)
    {
        usb_host_client_handle_events(driver->client_hdl, portMAX_DELAY);
        //ESP_LOGI(TAG, "client event");
    }
}

static void s4_out_transfer_task(void* arg)
{
    s4_driver_t* driver = (s4_driver_t*)arg;
    for (;;)
    {
        xSemaphoreTake(driver->out_sem, portMAX_DELAY);
        ESP_ERROR_CHECK(usb_host_transfer_submit(driver->out_transfer));
    }
}

static void s4_in_callback(usb_transfer_t* transfer)
{
    s4_driver_t* driver = (s4_driver_t*)transfer->context;

    if (transfer->status != USB_TRANSFER_STATUS_COMPLETED)
    {
        ESP_LOGI(TAG, "Inbound USB transfer failure %d~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", transfer->status);
        if (transfer->status != USB_TRANSFER_STATUS_CANCELED)
        {
            //abort();
        }
        return;
    }

    char last_char = driver->in_buffer_size > 0 ? driver->in_buffer[driver->in_buffer_size - 1] : 0;

    for (int i = 0; i < transfer->actual_num_bytes; i++)
    {
        if (last_char == '\r' && transfer->data_buffer[i] == '\n')
        {
            driver->in_buffer_size--;
            driver->in_buffer[driver->in_buffer_size] = 0;
            s4_in_event(driver);
            driver->in_buffer_size = 0;
            continue;
        }

        if (driver->in_buffer_size < S4_MAX_COMMAND_SIZE)
        {
            driver->in_buffer[driver->in_buffer_size++] = transfer->data_buffer[i];
        }
        else
        {
            ESP_LOGE(TAG, "Input buffer overflow");
            abort();
        }

        last_char = transfer->data_buffer[i];
    }

    ESP_ERROR_CHECK(usb_host_transfer_submit(driver->in_transfer));
}

static void s4_out_callback(usb_transfer_t* transfer)
{
    s4_driver_t* driver = (s4_driver_t*)transfer->context;

    if (transfer->status != USB_TRANSFER_STATUS_COMPLETED)
    {
        ESP_LOGI(TAG, "Output USB transfer failure %d~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", transfer->status);
        if (transfer->status != USB_TRANSFER_STATUS_CANCELED)
        {
            abort();
        }
        return;
    }

    xSemaphoreGive(driver->out_resp_sem);
}

static void s4_session_task(void* arg)
{
    s4_driver_t* driver = (s4_driver_t*)arg;

    for (;;)
    {
        ESP_LOGI(TAG, "Waiting for power button press");
        while (gpio_get_level(S4_POWER_BUTTON_GPIO_NUM) == 1)
        {
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Power button pressed");
        gpio_set_level(S4_USB_POWER_GPIO_NUM, 1);
        driver->power_on = true;

        xSemaphoreTake(driver->device_sem, portMAX_DELAY);

        ESP_ERROR_CHECK(usb_host_device_open(driver->client_hdl, driver->dev_addr, &driver->dev_hdl));

        usb_device_info_t dev_info;
        ESP_ERROR_CHECK(usb_host_device_info(driver->dev_hdl, &dev_info));

        uint16_t wr_marker[] = { 'W', 'R', '-', 'S', '4' };

        if (dev_info.str_desc_product != NULL)
        {
            for (int i = 0; i < (dev_info.str_desc_product->bLength - sizeof(wr_marker)) / 2; i++)
            {
                if (memcmp(&dev_info.str_desc_product->wData[i], wr_marker, sizeof(wr_marker)) == 0)
                {
                    ESP_LOGI(TAG, "Found WR-S4 device");

                    ESP_ERROR_CHECK(s4_setup(driver));
                    break;
                }
            }
        }
        else
        {
            ESP_LOGI(TAG, "Found unknown device");
            ESP_ERROR_CHECK(usb_host_device_close(driver->client_hdl, driver->dev_hdl));
            continue;
        }

        s4_values_t values;
        int64_t last_last_stroke_start_ts;
        ESP_ERROR_CHECK(s4_get_values(driver, &values));
        last_last_stroke_start_ts = values.last_stroke_start_ts;
        ESP_LOGI(TAG, "Waiting for inactivity...");

        int64_t inactive_ts = esp_timer_get_time();

        do
        {
            for (int i = 0; i < S4_MEMORY_MAP_SIZE; i++)
            {
                s4_read_memory(driver, &memory_map[i]);
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(s4_get_values(driver, &values));
            if (last_last_stroke_start_ts != values.last_stroke_start_ts)
            {
                inactive_ts = esp_timer_get_time();
                last_last_stroke_start_ts = values.last_stroke_start_ts;
            }
        }
        while (esp_timer_get_time() - inactive_ts < S4_INACTIVTY_TIMEOUT * 1000000llu);
        ESP_LOGI(TAG, "Inactivity detected. Shutting down...");
        ESP_ERROR_CHECK(s4_shutdown(driver));
        gpio_set_level(S4_USB_POWER_GPIO_NUM, 0);
        driver->power_on = false;

        driver->dev_hdl = NULL;
    }
}

static esp_err_t s4_setup(s4_driver_t* driver)
{
    ESP_ERROR_CHECK(usb_host_interface_claim(driver->client_hdl, driver->dev_hdl, S4_USB_INTERFACE_NUMBER, 0));
    ESP_ERROR_CHECK(usb_host_transfer_alloc(S4_MAX_PACKET_SIZE, 0, &driver->in_transfer));

    driver->in_transfer->timeout_ms = 1000;
    driver->in_transfer->device_handle = driver->dev_hdl;
    driver->in_transfer->callback = s4_in_callback;
    driver->in_transfer->context = driver;
    driver->in_transfer->bEndpointAddress = S4_USB_ENDPOINT_IN_ADDRESS;
    driver->in_transfer->num_bytes = S4_MAX_PACKET_SIZE;

    ESP_ERROR_CHECK(usb_host_transfer_submit(driver->in_transfer));

    ESP_ERROR_CHECK(usb_host_transfer_alloc(S4_MAX_PACKET_SIZE, 0, &driver->out_transfer));

    driver->out_transfer->timeout_ms = 1000;
    driver->out_transfer->device_handle = driver->dev_hdl;
    driver->out_transfer->callback = s4_out_callback;
    driver->out_transfer->context = driver;
    driver->out_transfer->bEndpointAddress = S4_USB_ENDPOINT_OUT_ADDRESS;

    xTaskCreate(s4_out_transfer_task, "wr_out_transfer_task", 4096, driver, 23, &driver->out_transfer_task_handle);

    s4_command(driver, S4_COMMAND_USB, S4_COMMAND_USB_RESPONSE);

    //xTaskCreate(s4_poll_task, "s4_poll_task", 4096, driver, 23, &driver->poll_task_handle);

    return ESP_OK;
}

static esp_err_t s4_shutdown(s4_driver_t* driver)
{
    ESP_ERROR_CHECK(usb_host_endpoint_halt(driver->dev_hdl, S4_USB_ENDPOINT_IN_ADDRESS));
    ESP_ERROR_CHECK(usb_host_endpoint_flush(driver->dev_hdl, S4_USB_ENDPOINT_IN_ADDRESS));
    ESP_ERROR_CHECK(usb_host_endpoint_halt(driver->dev_hdl, S4_USB_ENDPOINT_OUT_ADDRESS));
    ESP_ERROR_CHECK(usb_host_endpoint_flush(driver->dev_hdl, S4_USB_ENDPOINT_OUT_ADDRESS));
    vTaskDelete(driver->out_transfer_task_handle);
    driver->out_transfer_task_handle = NULL;

    // process above events
    usb_host_client_handle_events(driver->client_hdl, 0);

    ESP_ERROR_CHECK(usb_host_transfer_free(driver->in_transfer));
    driver->in_transfer = NULL;
    ESP_ERROR_CHECK(usb_host_transfer_free(driver->out_transfer));
    driver->out_transfer = NULL;

    ESP_ERROR_CHECK(usb_host_interface_release(driver->client_hdl, driver->dev_hdl, S4_USB_INTERFACE_NUMBER));
    ESP_ERROR_CHECK(usb_host_device_close(driver->client_hdl, driver->dev_hdl));
    driver->dev_hdl = NULL;
    driver->dev_addr = 0;

    return ESP_OK;
}

esp_err_t s4_init(hrm_handle_t hrm_handle, s4_handle_t* s4_handle)
{
    *s4_handle = NULL;

    s4_driver_t* driver = malloc(sizeof(s4_driver_t));
    memset(driver, 0, sizeof(s4_driver_t));

    driver->hrm_handle = hrm_handle;

    driver->device_sem = xSemaphoreCreateBinary();
    driver->out_sem = xSemaphoreCreateBinary();
    driver->out_resp_sem = xSemaphoreCreateBinary();
    driver->cmd_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(driver->cmd_sem);
    portMUX_INITIALIZE(&driver->values_mux);

    gpio_config_t io_conf;

    io_conf.pin_bit_mask = S4_USB_POWER_GPIO_SEL | S4_HEART_BEAT_TX_GPIO_SEL;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&io_conf);

    gpio_set_level(S4_USB_POWER_GPIO_NUM, 0);
    gpio_set_level(S4_HEART_BEAT_TX_GPIO_NUM, 0);

    io_conf.pin_bit_mask = S4_POWER_BUTTON_GPIO_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&io_conf);
    //vTaskDelay(50 / portTICK_PERIOD_MS);

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = s4_client_event_handler,
            .callback_arg = (void*)driver,
        }
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver->client_hdl));

    xTaskCreate(s4_usb_client_task, "s4_usb_client_task", 4096, (void*)driver, 23, NULL);
    xTaskCreate(s4_session_task, "s4_session_task", 4096, (void*)driver, 23, NULL);
    xTaskCreate(s4_heart_beat_task, "s4_heart_beat_task", 4096, driver, 23, NULL);

    *s4_handle = driver;

    return ESP_OK;
}

static void s4_read_memory(s4_driver_t* driver, const s4_variable_t* var)
{
    char cmd[S4_MAX_COMMAND_SIZE];
    snprintf(cmd, S4_MAX_COMMAND_SIZE, "IR%c%03X", var->size, var->address);

    char resp[S4_MAX_COMMAND_SIZE];
    snprintf(resp, S4_MAX_COMMAND_SIZE, "ID%c%03X", var->size, var->address);

    s4_command(driver, cmd, resp);
}

// static void s4_poll_task(void* arg)
// {
//     s4_driver_t* driver = (s4_driver_t*)arg;
//     for (;;)
//     {
//         for (int i = 0; i < S4_MEMORY_MAP_SIZE; i++)
//         {
//             s4_read_memory(driver, &memory_map[i]);
//         }

//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }

esp_err_t s4_get_values(s4_handle_t s4_handle, s4_values_t* values)
{
    if (s4_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }  

    s4_driver_t* driver = (s4_driver_t*)s4_handle;

    if (!driver->power_on)
    {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&driver->values_mux);
    memcpy(values, &driver->values, sizeof(s4_values_t));
    portEXIT_CRITICAL(&driver->values_mux);

    return ESP_OK;
}

int s4_variable_compare(const void* a, const void* b)
{
    const s4_variable_t* va = (const s4_variable_t*)a;
    const s4_variable_t* vb = (const s4_variable_t*)b;

    return (int)va->address - (int)vb->address;
}

static void s4_update_stroke_rate(s4_driver_t* driver)
{
    uint16_t stroke_period = driver->values.stroke_average * 25; // 25ms units;
    int8_t stroke_rate_x2 = 0;
    if (stroke_period != 0)
    {
        int64_t stalled_time = driver->values.last_stroke_start_ts != 0 ?
            (esp_timer_get_time() - driver->values.last_stroke_start_ts) / 1000 : 0;

        // if (stalled_time > 0)
        // {
        //     ets_printf("Stalled for %llums period %ums factor %u\r\n", stalled_time, stroke_period, (unsigned int)(stalled_time / stroke_period));
        // }

        stroke_rate_x2 = (int8_t)(120000.0 / stroke_period + 0.5);
        uint16_t stalled_threshold = 2 * stroke_period;
        if (stalled_time >= stalled_threshold)
        {
            uint32_t n = (stalled_time - stalled_threshold) / 1000 + 1;
            uint8_t stroke_rate_adjustment = n * log2(n);
            //ets_printf("Stalled for %llums n %u adj -%u\r\n", stalled_time, n, stroke_rate_adjustment);
            if (stroke_rate_x2 >= stroke_rate_adjustment)
            {
                stroke_rate_x2 -= stroke_rate_adjustment;
            }
            else
            {
                stroke_rate_x2 = 0;
            }
        }
    }
    driver->values.stroke_rate_x2 = stroke_rate_x2;
}

static void s4_set_value(s4_driver_t* driver, uint16_t address, uint32_t value)
{
    //ESP_LOGI(TAG, "Set [%03x] = %d", address, value);

    s4_variable_t* var;
    s4_variable_t key = { .address = address };

    // for (int i = 0; i < S4_MEMORY_MAP_SIZE; i++)
    // {
    //     printf("%03x ", memory_map[i].address);
    // }
    // printf("\n");
    // qsort(memory_map, S4_MEMORY_MAP_SIZE, sizeof(s4_variable_t), s4_variable_compare);
    // for (int i = 0; i < S4_MEMORY_MAP_SIZE; i++)
    // {
    //     printf("%03x ", memory_map[i].address);
    // }
    // printf("\n");
    // abort();

    var = bsearch(&key, memory_map, S4_MEMORY_MAP_SIZE, sizeof(s4_variable_t), s4_variable_compare);

    if (var != NULL)
    {
        int index = var - memory_map;

        //ESP_LOGI(TAG, "index: %d base: %p var: %p diff: %u size: %u", index, memory_map, var, var - memory_map, sizeof(s4_variable_t));

        portENTER_CRITICAL(&driver->values_mux);

        switch (index)
        {
            case S4_DISTANCE:
                driver->values.distance = value;
                break;
            case S4_STROKE_COUNT:
                driver->values.stroke_count = value;
                break;
            case S4_STROKE_AVERAGE:
                driver->values.stroke_average = value;
                s4_update_stroke_rate(driver);
                break;
                // case S4_POWER:
                //     if (driver->values.power != value)
                //     {
                //         if (value != 0 || driver->values.stroke_rate_x2 == 0)
                //         {
                //             driver->values.power = value;
                //             //changed = true;
                //         }
                //     }
                //     break;
            case S4_500_PACE:
                // flip bytes.  500pace byte order reversed?!
                value = ((value & 0xff) << 8) | (value >> 8);
                // concept 2 calc
                driver->values.power = value != 0 ? (uint16_t)(2.80 / pow(value / 500.0, 3)) : 0;
                //fprintf("500 pace: %u (%u:%02u) power: %uW\n", value, value / 60 , value % 60, driver->values.power);
                break;
            case S4_CALORIES:
                value = (value + 500) / 1000;
                driver->values.calories = value;
                break;
            case S4_CURRENT_SPEED:
                driver->values.current_speed = value;
                break;
            case S4_TIMER_SECOND_DEC:
                driver->values.timer_second_dec = value;
                break;
            case S4_TIMER_SECOND:
                driver->values.timer_second = value;
                break;
            case S4_TIMER_MINUTE:
                driver->values.timer_minute = value;
                break;
            case S4_TIMER_HOUR:
                driver->values.timer_hour = value;
                break;
            case S4_HEART_RATE:
                driver->values.heart_rate = value;
                hrm_set_rate(driver->hrm_handle, HRM_SOURCE_S4, value);
                break;
        }

        driver->values.timer = driver->values.timer_hour * 3600 +
            driver->values.timer_minute * 60 +
            driver->values.timer_second +
            driver->values.timer_second_dec / 10.0;

        portEXIT_CRITICAL(&driver->values_mux);
    }
}

static void s4_in_event(s4_driver_t* driver)
{
    const int prefix_size = 6;

    if (driver->in_buffer_size >= prefix_size)
    {
        driver->in_buffer[driver->in_buffer_size] = 0;
        if (driver->in_buffer[0] == 'I' && driver->in_buffer[1] == 'D')
        {
            uint32_t offset;
            uint8_t size;
            if (sscanf(driver->in_buffer, "ID%c%03X", &size, &offset) == 2)
            {
                if (size == S4_SINGLE_VALUE && driver->in_buffer_size == 8)
                {
                    uint32_t value;
                    if (sscanf(driver->in_buffer + prefix_size, "%02X", &value) == 1)
                    {
                        s4_set_value(driver, offset, value);
                    }
                }
                else if (size == S4_DOUBLE_VALUE && driver->in_buffer_size == 10)
                {
                    uint32_t high;
                    uint32_t low;
                    if (sscanf(driver->in_buffer + prefix_size, "%02X%02X", &high, &low) == 2)
                    {
                        uint32_t value = high << 8 | low;
                        s4_set_value(driver, offset, value);
                    }
                }
                else if (size == S4_TRIPLE_VALUE && driver->in_buffer_size == 12)
                {
                    uint32_t higher;
                    uint32_t high;
                    uint32_t low;
                    if (sscanf(driver->in_buffer + prefix_size, "%02X%02X%02X", &higher, &high, &low) == 3)
                    {
                        uint32_t value = higher << 16 | high << 8 | low;
                        s4_set_value(driver, offset, value);
                    }
                }
            }
        }
    }

    if (strcmp(driver->in_buffer, "SS") == 0)
    {
        portENTER_CRITICAL(&driver->values_mux);
        driver->values.last_stroke_start_ts = esp_timer_get_time();
        driver->values.stroke_start_count++;
        portEXIT_CRITICAL(&driver->values_mux);

        ESP_LOGI(TAG, "stroke #%u start @ %lld", driver->values.stroke_start_count, driver->values.last_stroke_start_ts);
    }

    if (driver->wait_buffer_size != 0)
    {
        if (strncmp(driver->in_buffer, driver->wait_buffer, driver->wait_buffer_size) == 0 ||
            strcmp(driver->in_buffer, "ERROR") == 0)
        {
            driver->wait_buffer_size = 0;
            xSemaphoreGive(driver->cmd_sem);
        }
    }
}

static void s4_heart_beat_task(void* param)
{
    s4_driver_t* driver = (s4_driver_t*)param;

    for (;;)
    {
        uint8_t hr;
        if (driver->power_on &&
            hrm_get_rate(driver->hrm_handle, &hr) == ESP_OK && hr > 0)
        {
            uint16_t hr_period = 60000 / hr;

            gpio_set_level(S4_HEART_BEAT_TX_GPIO_NUM, 1);
            ets_delay_us(10000);
            gpio_set_level(S4_HEART_BEAT_TX_GPIO_NUM, 0);

            //ESP_LOGI(TAG, "s4 hr %u period %u\n", hr, hr_period);

            vTaskDelay((hr_period - 10) / portTICK_PERIOD_MS);
        }
        else
        {
            gpio_set_level(S4_HEART_BEAT_TX_GPIO_NUM, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}
