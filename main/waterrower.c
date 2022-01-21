#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "driver/gpio.h"
#include "waterrower.h"

#define TAG "waterrower"

#define WATERROWER_MAX_PACKET_SIZE          64
#define WATERROWER_OUT_ENDPOINT_ADDRESS     0x03
#define WATERROWER_IN_ENDPOINT_ADDRESS      0x83
#define WATERROWER_INTERFACE_NUMBER         0x01

#define WATERROWER_MAX_COMMAND_SIZE         62

#define WATERROWER_SINGLE_VALUE             'S'
#define WATERROWER_DOUBLE_VALUE             'D'
#define WATERROWER_TRIPLE_VALUE             'T'

#define WATERROWER_COMMAND_USB              "USB"
#define WATERROWER_COMMAND_USB_RESPONSE     "_WR_"
#define WATERROWER_COMMAND_RESET            "RESET"

typedef struct
{
    uint16_t address;
    uint8_t size;
    const char* name;
}
waterrower_variable_t;

// these must be in address order!
enum
{
    WATERROWER_DISTANCE,
    WATERROWER_POWER,
    WATERROWER_CALORIES,
    WATERROWER_STROKE_COUNT,
    WATERROWER_STROKE_AVERAGE,
    WATERROWER_CURRENT_SPEED,
    WATERROWER_HEART_RATE,
    WATERROWER_TIMER_SECOND_DEC,
    WATERROWER_TIMER_SECOND,
    WATERROWER_TIMER_MINUTE,
    WATERROWER_TIMER_HOUR,
};

waterrower_variable_t memory_map[] =
{
    [WATERROWER_DISTANCE] = { 0x055, WATERROWER_DOUBLE_VALUE, "distance" },
    [WATERROWER_POWER] = { 0x088, WATERROWER_DOUBLE_VALUE, "power" },
    [WATERROWER_CALORIES] = { 0x08a, WATERROWER_TRIPLE_VALUE, "calories" },
    [WATERROWER_STROKE_COUNT] = { 0x140, WATERROWER_DOUBLE_VALUE, "stroke_count" },
    [WATERROWER_STROKE_AVERAGE] = { 0x142, WATERROWER_SINGLE_VALUE, "stroke_average" },
    [WATERROWER_CURRENT_SPEED] = { 0x14a, WATERROWER_DOUBLE_VALUE, "current_speed" },
    [WATERROWER_HEART_RATE] = { 0x1a0, WATERROWER_SINGLE_VALUE, "heart_rate" },
    [WATERROWER_TIMER_SECOND_DEC] = { 0x1e0, WATERROWER_SINGLE_VALUE, "timer_second_dec" },
    [WATERROWER_TIMER_SECOND] = { 0x1e1, WATERROWER_SINGLE_VALUE, "timer_second" },
    [WATERROWER_TIMER_MINUTE] = { 0x1e2, WATERROWER_SINGLE_VALUE, "timer_minute" },
    [WATERROWER_TIMER_HOUR] = { 0x1e3, WATERROWER_SINGLE_VALUE, "timer_hour" }
};
#define WATERROWER_MEMORY_MAP_SIZE  (sizeof(memory_map) / sizeof(waterrower_variable_t))

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;

    SemaphoreHandle_t device_sem;

    usb_transfer_t* in_transfer;
    char in_buffer[WATERROWER_MAX_COMMAND_SIZE + 1];
    uint8_t in_buffer_size;

    usb_transfer_t* out_transfer;
    SemaphoreHandle_t out_sem;
    SemaphoreHandle_t out_resp_sem;

    SemaphoreHandle_t cmd_sem;
    char wait_buffer[WATERROWER_MAX_COMMAND_SIZE + 1];
    uint8_t wait_buffer_size;

    portMUX_TYPE values_mux;
    waterrower_values_t values;
} waterrower_driver_t;


static void waterrower_in_event(waterrower_driver_t* driver);
static void waterrower_poll_task(void* arg);

static void waterrower_client_event_handler(const usb_host_client_event_msg_t* event_msg, void* arg)
{
    waterrower_driver_t* driver = (waterrower_driver_t*)arg;
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
            esp_restart();
            break;
        default:
            ESP_LOGE(TAG, "Unkown USB client event");
            abort();
            break;
    }
}

static esp_err_t waterrower_command(waterrower_driver_t* driver, const char* cmd, const char* resp)
{
    int length = strlen(cmd);
    if (length + 2 >= WATERROWER_MAX_PACKET_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    //printf("take cmd_sem\r\n");
    xSemaphoreTake(driver->cmd_sem, portMAX_DELAY);
    //printf("took cmd_sem\r\n");

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

    //printf("wait %s (%d)\r\n", driver->wait_buffer, driver->wait_buffer_size);

    xSemaphoreGive(driver->out_sem);
    xSemaphoreTake(driver->out_resp_sem, portMAX_DELAY);

    return ESP_OK;
}

static void waterrower_event_handler_task(void* arg)
{
    waterrower_driver_t* driver = (waterrower_driver_t*)arg;

    for (;;)
    {
        usb_host_client_handle_events(driver->client_hdl, portMAX_DELAY);
    }
}

static void waterrower_out_transfer_task(void* arg)
{
    waterrower_driver_t* driver = (waterrower_driver_t*)arg;
    for (;;)
    {
        xSemaphoreTake(driver->out_sem, portMAX_DELAY);
        ESP_ERROR_CHECK(usb_host_transfer_submit(driver->out_transfer));
    }
}

static void waterrower_in_callback(usb_transfer_t* transfer)
{
    waterrower_driver_t* driver = (waterrower_driver_t*)transfer->context;

    char last_char = driver->in_buffer_size > 0 ? driver->in_buffer[driver->in_buffer_size - 1] : 0;

    for (int i = 0; i < transfer->actual_num_bytes; i++)
    {
        if (last_char == '\r' && transfer->data_buffer[i] == '\n')
        {
            driver->in_buffer_size--;
            driver->in_buffer[driver->in_buffer_size] = 0;
            waterrower_in_event(driver);
            driver->in_buffer_size = 0;
            continue;
        }

        if (driver->in_buffer_size < WATERROWER_MAX_COMMAND_SIZE)
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

static void waterrower_out_callback(usb_transfer_t* transfer)
{
    waterrower_driver_t* driver = (waterrower_driver_t*)transfer->context;
    xSemaphoreGive(driver->out_resp_sem);
}

esp_err_t waterrower_setup(waterrower_driver_t* driver)
{
    ESP_ERROR_CHECK(usb_host_interface_claim(driver->client_hdl, driver->dev_hdl, WATERROWER_INTERFACE_NUMBER, 0));
    ESP_ERROR_CHECK(usb_host_transfer_alloc(WATERROWER_MAX_PACKET_SIZE, 0, &driver->in_transfer));

    driver->in_transfer->timeout_ms = 1000;
    driver->in_transfer->device_handle = driver->dev_hdl;
    driver->in_transfer->callback = waterrower_in_callback;
    driver->in_transfer->context = driver;
    driver->in_transfer->bEndpointAddress = WATERROWER_IN_ENDPOINT_ADDRESS;
    driver->in_transfer->num_bytes = WATERROWER_MAX_PACKET_SIZE;

    ESP_ERROR_CHECK(usb_host_transfer_submit(driver->in_transfer));

    ESP_ERROR_CHECK(usb_host_transfer_alloc(WATERROWER_MAX_PACKET_SIZE, 0, &driver->out_transfer));

    driver->out_transfer->timeout_ms = 1000;
    driver->out_transfer->device_handle = driver->dev_hdl;
    driver->out_transfer->callback = waterrower_out_callback;
    driver->out_transfer->context = driver;
    driver->out_transfer->bEndpointAddress = WATERROWER_OUT_ENDPOINT_ADDRESS;

    xTaskCreate(waterrower_out_transfer_task, "wr_out_transfer_task", 4096, driver, 23, NULL);

    waterrower_command(driver, WATERROWER_COMMAND_USB, WATERROWER_COMMAND_USB_RESPONSE);

    xTaskCreate(waterrower_poll_task, "waterrower_poll_task", 4096, driver, 23, NULL);

    return ESP_OK;
}

esp_err_t waterrower_init(waterrower_handle_t* waterrower_handle)
{
    *waterrower_handle = NULL;

    waterrower_driver_t* driver = malloc(sizeof(waterrower_driver_t));
    memset(driver, 0, sizeof(waterrower_driver_t));

    driver->device_sem = xSemaphoreCreateBinary();
    driver->out_sem = xSemaphoreCreateBinary();
    driver->out_resp_sem = xSemaphoreCreateBinary();
    driver->cmd_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(driver->cmd_sem);
    portMUX_INITIALIZE(&driver->values_mux);

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = waterrower_client_event_handler,
            .callback_arg = (void*)driver,
        }
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver->client_hdl));

    xTaskCreate(waterrower_event_handler_task, "waterrower_event_handler_task", 4096, (void*)driver, 23, NULL);

    gpio_config_t io_conf;

    io_conf.pin_bit_mask = GPIO_SEL_8;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&io_conf);

    // ESP_LOGI(TAG, "Waiting to power on...");
    // vTaskDelay(10000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Power on");
    gpio_set_level(GPIO_NUM_8, 1);

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

                ESP_ERROR_CHECK(waterrower_setup(driver));
                break;
            }
        }
    }
    else
    {
        ESP_LOGI(TAG, "Found unknown device");
        return ESP_FAIL;
    }

    *waterrower_handle = driver;

    return ESP_OK;
}

static void waterrower_read_memory(waterrower_driver_t* driver, const waterrower_variable_t* var)
{
    char cmd[WATERROWER_MAX_COMMAND_SIZE];
    snprintf(cmd, WATERROWER_MAX_COMMAND_SIZE, "IR%c%03X", var->size, var->address);

    char resp[WATERROWER_MAX_COMMAND_SIZE];
    snprintf(resp, WATERROWER_MAX_COMMAND_SIZE, "ID%c%03X", var->size, var->address);

    waterrower_command(driver, cmd, resp);
}

static void waterrower_poll_task(void* arg)
{
    waterrower_driver_t* driver = (waterrower_driver_t*)arg;
    for (;;)
    {
        for (int i = 0; i < WATERROWER_MEMORY_MAP_SIZE; i++)
        {
            waterrower_read_memory(driver, &memory_map[i]);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

esp_err_t waterrower_get_values(waterrower_handle_t waterrower_handle, waterrower_values_t* values)
{
    if (waterrower_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    waterrower_driver_t* driver = (waterrower_driver_t*)waterrower_handle;

    portENTER_CRITICAL(&driver->values_mux);
    memcpy(values, &driver->values, sizeof(waterrower_values_t));
    portEXIT_CRITICAL(&driver->values_mux);

    return ESP_OK;
}

int waterrower_variable_compare(const void* a, const void* b)
{
    const waterrower_variable_t* va = (const waterrower_variable_t*)a;
    const waterrower_variable_t* vb = (const waterrower_variable_t*)b;

    return (int)va->address - (int)vb->address;
}

static void waterrower_update_stroke_rate(waterrower_driver_t* driver)
{
    uint16_t stroke_period = driver->values.stroke_average * 25; // 25ms units;
    int8_t stroke_rate_x2 = 0;
    if (stroke_period != 0)
    {
        int64_t stalled_time = driver->values.last_stroke_start_ts != 0 ? 
            (esp_timer_get_time() - driver->values.last_stroke_start_ts) / 1000 : 0;

        if (stalled_time > 0)
        {
            ets_printf("Stalled for %llum period %ums\r\n", stalled_time, stroke_period);
        }

        stroke_rate_x2 = stroke_period != 0 ? (int8_t)(120000.0 / stroke_period + 0.5) : 0;
        if (stalled_time >= 2 * stroke_period)
        {
            uint8_t stroke_rate_adjustment = stalled_time / stroke_period * 6;
            ets_printf("Stalled for %llums adj -%u\r\n", stalled_time / 1000, stroke_rate_adjustment);
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

static void waterrower_set_value(waterrower_driver_t* driver, uint16_t address, uint32_t value)
{
    //ESP_LOGI(TAG, "Set [%03x] = %d", address, value);

    waterrower_variable_t* var;
    waterrower_variable_t key = { .address = address };

    // for (int i = 0; i < WATERROWER_MEMORY_MAP_SIZE; i++)
    // {
    //     printf("%03x ", memory_map[i].address);
    // }
    // printf("\n");
    // qsort(memory_map, WATERROWER_MEMORY_MAP_SIZE, sizeof(waterrower_variable_t), waterrower_variable_compare);
    // for (int i = 0; i < WATERROWER_MEMORY_MAP_SIZE; i++)
    // {
    //     printf("%03x ", memory_map[i].address);
    // }
    // printf("\n");
    // abort();

    var = bsearch(&key, memory_map, WATERROWER_MEMORY_MAP_SIZE, sizeof(waterrower_variable_t), waterrower_variable_compare);

    if (var != NULL)
    {
        int index = var - memory_map;

        //ESP_LOGI(TAG, "index: %d base: %p var: %p diff: %u size: %u", index, memory_map, var, var - memory_map, sizeof(waterrower_variable_t));

        portENTER_CRITICAL(&driver->values_mux);
        bool changed = false;
        bool timer_changed = false;

        switch (index)
        {
            case WATERROWER_DISTANCE:
                if (driver->values.distance != value)
                {
                    driver->values.distance = value;
                    changed = true;
                }
                break;
            case WATERROWER_STROKE_COUNT:
                if (driver->values.stroke_count != value)
                {
                    driver->values.stroke_count = value;
                    changed = true;
                }
                break;
            case WATERROWER_STROKE_AVERAGE:
                if (driver->values.stroke_average != value)
                {        
                    driver->values.stroke_average = value;
                    changed = true;
                }
                waterrower_update_stroke_rate(driver);

                break;
            case WATERROWER_POWER:
                if (driver->values.power != value)
                {
                    if (value != 0 || driver->values.stroke_rate_x2 == 0)
                    {
                        driver->values.power = value;
                        changed = true;
                    }
                }
                break;
            case WATERROWER_CALORIES:
                value = (value + 500) / 1000;
                if (driver->values.calories != value)
                {
                    driver->values.calories = value;
                    changed = true;
                }
                break;
            case WATERROWER_CURRENT_SPEED:
                if (driver->values.current_speed != value)
                {
                    driver->values.current_speed = value;
                    changed = true;
                }
                break;
            case WATERROWER_TIMER_SECOND_DEC:
                if (driver->values.timer_second_dec != value)
                {
                    driver->values.timer_second_dec = value;
                    timer_changed = true;
                    changed = true;
                }
                break;
            case WATERROWER_TIMER_SECOND:
                if (driver->values.timer_second != value)
                {
                    driver->values.timer_second = value;
                    timer_changed = true;
                    changed = true;
                }
                break;
            case WATERROWER_TIMER_MINUTE:
                if (driver->values.timer_minute != value)
                {
                    driver->values.timer_minute = value;
                    timer_changed = true;
                    changed = true;
                }
                break;
            case WATERROWER_TIMER_HOUR:
                if (driver->values.timer_hour != value)
                {
                    driver->values.timer_hour = value;
                    timer_changed = true;
                    changed = true;
                }
                break;
            case WATERROWER_HEART_RATE:
                if (driver->values.heart_rate != value)
                {
                    driver->values.heart_rate = value;
                    changed = true;
                }
                break;
        }

        driver->values.timer = driver->values.timer_hour * 3600 +
            driver->values.timer_minute * 60 +
            driver->values.timer_second +
            driver->values.timer_second_dec / 10.0;

        portEXIT_CRITICAL(&driver->values_mux);

        // if (timer_changed)
        // {
        //     ESP_LOGI(TAG, "timer %u:%02u:%02u.%u (%.1f)", driver->values.timer_hour, driver->values.timer_minute, 
        //         driver->values.timer_second, driver->values.timer_second_dec, driver->values.timer);
        // }
        // else if (changed)
        // {
        //     ESP_LOGI(TAG, "%s changed => %u", var->name, value);
        // }

    }
}

static void waterrower_in_event(waterrower_driver_t* driver)
{
    ESP_LOGD(TAG, "Event: %s", driver->in_buffer);

    const int prefix_size = 6;

    if (driver->in_buffer_size >= prefix_size)
    {
        if (driver->in_buffer[0] == 'I' && driver->in_buffer[1] == 'D')
        {
            uint32_t offset;
            uint8_t size;
            if (sscanf(driver->in_buffer, "ID%c%03X", &size, &offset) == 2)
            {
                if (size == WATERROWER_SINGLE_VALUE && driver->in_buffer_size == 8)
                {
                    uint32_t value;
                    if (sscanf(driver->in_buffer + prefix_size, "%02X", &value) == 1)
                    {
                        waterrower_set_value(driver, offset, value);
                    }
                }
                else if (size == WATERROWER_DOUBLE_VALUE && driver->in_buffer_size == 10)
                {
                    uint32_t high;
                    uint32_t low;
                    if (sscanf(driver->in_buffer + prefix_size, "%02X%02X", &high, &low) == 2)
                    {
                        uint32_t value = high << 8 | low;
                        waterrower_set_value(driver, offset, value);
                    }
                }
                else if (size == WATERROWER_TRIPLE_VALUE && driver->in_buffer_size == 12)
                {
                    uint32_t higher;
                    uint32_t high;
                    uint32_t low;
                    if (sscanf(driver->in_buffer + prefix_size, "%02X%02X%02X", &higher, &high, &low) == 3)
                    {
                        uint32_t value = higher << 16 | high << 8 | low;
                        waterrower_set_value(driver, offset, value);
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
            //printf("give cmd_sem\r\n");
            driver->wait_buffer_size = 0;
            xSemaphoreGive(driver->cmd_sem);
        }
    }
}
