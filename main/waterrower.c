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

#define WATERROWER_INPUT_BUFFER_SIZE        64
#define WATERROWER_OUTPUT_BUFFER_SIZE       64

#define WATERROWER_SINGLE_VALUE             'S'
#define WATERROWER_DOUBLE_VALUE             'D'
#define WATERROWER_TRIPLE_VALUE             'T'

#define WATERROWER_COMMAND_USB              "USB"
#define WATERROWER_COMMAND_RESET            "RESET"

typedef struct
{
    uint16_t offset;
    uint8_t size;
    uint8_t base;
}
waterrower_address_t;

enum
{
    WATERROWER_DISTANCE,
    WATERROWER_STROKE_COUNT,
    WATERROWER_WATTS,
    WATERROWER_CALORIES,
    WATERROWER_CURRENT_SPEED,
    WATERROWER_TIMER_SECOND_DECIMAL,
    WATERROWER_TIMER_SECOND,
    WATERROWER_TIMER_MINUTE,
    WATERROWER_TIMER_HOUR,
    WATERROWER_HEART_RATE,
    WATERROWER_500_PACE,
    WATERROWER_STROKE_RATE
};

waterrower_address_t memory_map[] =
{
    [WATERROWER_DISTANCE]             = { 0x055, WATERROWER_DOUBLE_VALUE, 16 },
    [WATERROWER_WATTS]                = { 0x088, WATERROWER_DOUBLE_VALUE, 16 },
    [WATERROWER_CALORIES]             = { 0x08a, WATERROWER_TRIPLE_VALUE, 16 },
    [WATERROWER_STROKE_COUNT]         = { 0x140, WATERROWER_DOUBLE_VALUE, 16 },
    [WATERROWER_CURRENT_SPEED]        = { 0x14a, WATERROWER_DOUBLE_VALUE, 16 },
    [WATERROWER_HEART_RATE]           = { 0x1a0, WATERROWER_SINGLE_VALUE, 16 },
    [WATERROWER_500_PACE]             = { 0x1a6, WATERROWER_DOUBLE_VALUE, 16 },
    [WATERROWER_STROKE_RATE]          = { 0x1a9, WATERROWER_SINGLE_VALUE, 16 },
    [WATERROWER_TIMER_SECOND_DECIMAL] = { 0x1e0, WATERROWER_SINGLE_VALUE, 10 },
    [WATERROWER_TIMER_SECOND]         = { 0x1e1, WATERROWER_SINGLE_VALUE, 10 },
    [WATERROWER_TIMER_MINUTE]         = { 0x1e2, WATERROWER_SINGLE_VALUE, 10 },
    [WATERROWER_TIMER_HOUR]           = { 0x1e3, WATERROWER_SINGLE_VALUE, 10 }
};
#define WATERROWER_MEMORY_MAP_SIZE  (sizeof(memory_map) / sizeof(waterrower_address_t))

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;

    SemaphoreHandle_t device_sem;

    usb_transfer_t* in_transfer;
    char in_buffer[WATERROWER_INPUT_BUFFER_SIZE];
    uint8_t in_buffer_used;

    usb_transfer_t* out_transfer;
    SemaphoreHandle_t out_sem;
    SemaphoreHandle_t out_resp_sem;

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
            abort();
            break;
    }
}

static esp_err_t waterrower_command(waterrower_driver_t* driver, const char* cmd)
{
    int length = strlen(cmd);
    if (length + 2 >= WATERROWER_MAX_PACKET_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }
    char* buffer = (char*)driver->out_transfer->data_buffer;
    driver->out_transfer->num_bytes = length + 2;
    memcpy(buffer, cmd, length);
    
    buffer[length] = 0;
    ESP_LOGI(TAG, "Command: %s", cmd);

    buffer[length] = '\r';
    buffer[length + 1] = '\n';    

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

    char last_char = driver->in_buffer_used > 0 ? driver->in_buffer[driver->in_buffer_used - 1] : 0;

    for (int i = 0; i < transfer->actual_num_bytes; i++)
    {
        if (last_char == '\r' && transfer->data_buffer[i] == '\n')
        {
            driver->in_buffer_used--;
            driver->in_buffer[driver->in_buffer_used] = 0;
            waterrower_in_event(driver);
            driver->in_buffer_used = 0;
            continue;
        }

        if (driver->in_buffer_used < WATERROWER_INPUT_BUFFER_SIZE)
        {
            driver->in_buffer[driver->in_buffer_used++] = transfer->data_buffer[i];
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

    waterrower_command(driver, WATERROWER_COMMAND_USB);

    xTaskCreate(waterrower_poll_task, "waterrower_poll_task", 4096, driver, 23, NULL);

    return ESP_OK;
}

esp_err_t waterrower_init()
{
    waterrower_driver_t* driver = malloc(sizeof(waterrower_driver_t));
    memset(driver, 0, sizeof(waterrower_driver_t));

    driver->device_sem = xSemaphoreCreateBinary();
    driver->out_sem = xSemaphoreCreateBinary();
    driver->out_resp_sem = xSemaphoreCreateBinary();
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

    ESP_LOGI(TAG, "Waiting 1s to power on...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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

    return ESP_OK;
}

static void waterrower_read_memory(waterrower_driver_t* driver, const waterrower_address_t* addr)
{
    char cmd[WATERROWER_OUTPUT_BUFFER_SIZE];

    snprintf(cmd, WATERROWER_OUTPUT_BUFFER_SIZE, "IR%c%03X", addr->size, addr->offset);
    waterrower_command(driver, cmd);
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

static void waterrower_set_value(waterrower_driver_t* driver, uint16_t offset, uint32_t value)
{
    ESP_LOGI(TAG, "Set value %d at offset %x", value, offset);
}

static void waterrower_in_event(waterrower_driver_t* driver)
{
    ESP_LOGI(TAG, "Event: %s (%u)", driver->in_buffer, driver->in_buffer_used);

    const int prefix_size = 6;

    if (driver->in_buffer_used >= prefix_size)
    {
        if (driver->in_buffer[0] == 'I' && driver->in_buffer[1] == 'D')
        {
            uint32_t offset;
            uint8_t size;
            if (sscanf(driver->in_buffer, "ID%c%03X", &size, &offset) == 2)
            {
                if (size == WATERROWER_SINGLE_VALUE && driver->in_buffer_used == 8)
                {
                    uint32_t value;
                    if (sscanf(driver->in_buffer + prefix_size, "%02X", &value) == 1)
                    {
                        waterrower_set_value(driver, offset, value);
                    }
                }
                else if (size == WATERROWER_SINGLE_VALUE && driver->in_buffer_used == 10)
                {
                    uint32_t high;
                    uint32_t low;
                    if (sscanf(driver->in_buffer + prefix_size, "%02X%02X", &high, &low) == 2)
                    {
                        uint32_t value = high << 8 | low;
                        waterrower_set_value(driver, offset, value);
                    }
                }
                else if (size == WATERROWER_SINGLE_VALUE && driver->in_buffer_used == 12)
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
}

