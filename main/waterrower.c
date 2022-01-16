#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "driver/gpio.h"

#define TAG "waterrower"

#define WATERROWER_MAX_PACKET_SIZE          64
#define WATERROWER_OUT_ENDPOINT_ADDRESS     0x03
#define WATERROWER_IN_ENDPOINT_ADDRESS      0x83
#define WATERROWER_INTERFACE_NUMBER         0x01

#define WATERROWER_COMMAND_USB              "USB"
#define WATERROWER_COMMAND_RESET            "RESET"

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;

    SemaphoreHandle_t device_sem;

    usb_transfer_t* in_transfer;

    usb_transfer_t* out_transfer;
    SemaphoreHandle_t out_sem;
    SemaphoreHandle_t out_resp_sem;
} waterrower_driver_t;

static esp_err_t waterrower_command(waterrower_driver_t* driver, const char* cmd)
{
    int length = strlen(cmd);
    if (length + 2 >= WATERROWER_MAX_PACKET_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }
    char* buffer = (char*)driver->out_transfer->data_buffer;
    driver->out_transfer->num_bytes = strlen(cmd) + 2;
    memcpy(buffer, cmd, length);
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
            break;
        default:
            abort();
            break;
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

    for (int i = 0; i < transfer->actual_num_bytes; i++)
    {
        printf("%c", transfer->data_buffer[i]);
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

    return ESP_OK;
}

esp_err_t waterrower_init()
{
    waterrower_driver_t* driver = malloc(sizeof(waterrower_driver_t));
    memset(driver, 0, sizeof(waterrower_driver_t));

    driver->device_sem = xSemaphoreCreateBinary();
    driver->out_sem = xSemaphoreCreateBinary();
    driver->out_resp_sem = xSemaphoreCreateBinary();

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

    ESP_LOGI(TAG, "Waiting 2s to power on...");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
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