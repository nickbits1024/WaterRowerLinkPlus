#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "antplus.h"
#include "antplus_int.h"
#include "state_manager.h"
#include "heart_rate_manager.h"
#include "s4.h"

#define TAG "ant+"

static void antplus_trainer_task(void* param);

//extern waterrower_handle_t waterrower_handle;

void antplus_decode_startup_message(antplus_driver_t* driver, antplus_message_t* msg)
{
    ESP_LOGI(TAG, "startup (type 0x%02x)", msg->data[0]);
    antplus_setup(driver);
}

void antplus_decode_version_message(antplus_driver_t* driver, antplus_version_t* msg)
{
    ESP_LOGI(TAG, "version %s", msg->data.version);
}

void antplus_decode_capabilities_message(antplus_driver_t* driver, antplus_capabilities_t* msg)
{
    ESP_LOGI(TAG, "caps (%u channels,  %u networks, %u sensrcore, standard 0x%02x, advanced 0x%02x 0x%02x 0x%02x 0x%02x)", 
        msg->data.max_channels, msg->data.max_networks, msg->data.max_sensrcore_channels, msg->data.standard_options,
        msg->data.advanced_options, msg->data.advanced_options_2, msg->data.advanced_options_3, msg->data.advanced_options_4);
}

void antplus_decode_channel_response_message(antplus_driver_t* driver, antplus_channel_reponse_message_t* msg)
{
    if (msg->data.msg_id == ANTPLUS_EVENT)
    {
        //ESP_LOGI(TAG, "event");
    }
    else
    {
        if (msg->data.error != 0)
        {
            ESP_LOGI(TAG, "channel %u request 0x%02x failed (error=%u)", msg->data.channel, msg->data.msg_id, msg->data.error);
        }
        else
        {
            ESP_LOGI(TAG, "channel %u request 0x%02x succeeded", msg->data.channel, msg->data.msg_id);
        }
    }
}

void antplus_decode_channel_broadcast_message(antplus_driver_t* driver, antplus_broadcast_message_t* msg)
{
    if (msg->data.channel == ANTPLUS_CHANNEL_HRM)
    {
        if (msg->size == ANTPLUS_MESSAGE_SIZE(antplus_broadcast_message_t))
        {
            antplus_heartrate_broadcast_t* hr_msg = (antplus_heartrate_broadcast_t*)msg;
            portENTER_CRITICAL(&driver->mux);
            switch (hr_msg->data.page_num)
            {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
                if (driver->heart_beat_ts != hr_msg->data.common.heart_beat_ts)
                {
                    hrm_set_rate(driver->hrm_handle, HRM_SOURCE_ANT, hr_msg->data.common.hr);
                    driver->heart_beat_ts = hr_msg->data.common.heart_beat_ts;
                }
                break;
            }
            portEXIT_CRITICAL(&driver->mux);
        }
    }
}

void antplus_decode_channel_id_message(antplus_driver_t* driver, antplus_channel_id_message_t* msg)
{
    ESP_LOGI(TAG, "channel %u device_num %u device_type %u tx_type 0x%02x", msg->data.channel, 
        msg->data.device_num, msg->data.device_type, msg->data.tx_type);
}

bool antplus_decode_message(antplus_driver_t* driver, uint8_t* packet, uint8_t packet_size)
{
    // printf("check");
    // for (int i = 0; i < packet_size; i++)
    // {
    //     printf(" %02x", packet[i]);
    // }
    // printf("\n");

    if (packet_size <= sizeof(antplus_message_t))
    {
        return false;
    }

    antplus_message_t* msg = (antplus_message_t*)packet;

    if (msg->sync_value != ANTPLUS_SYNC_VALUE)
    {
        abort();
    }

    uint8_t required_size = ANTPLUS_MESSAGE_REQUIRED_SIZE(msg);

    if (packet_size < required_size)
    {
        return false;
    }

    printf("msg");
    for (int i = 0; i < packet_size; i++)
    {
        printf(" %02x", packet[i]);
    }
    printf("\n");

    uint8_t checksum = 0;
    for (int i = 0; i < packet_size; i++)
    {
        checksum ^= packet[i];
    }
    if (checksum != 0)
    {
        ESP_LOGI(TAG, "checksum failure %02x != 0", checksum);
        return true;
    }

    switch (msg->msg_id)
    {
    case ANTPLUS_STARTUP:
        antplus_decode_startup_message(driver, msg);
        break;
    case ANTPLUS_VERSION:
        antplus_decode_version_message(driver, (antplus_version_t*)msg);
        break;
    case ANTPLUS_CAPABILITIES:
        antplus_decode_capabilities_message(driver, (antplus_capabilities_t*)msg);
        break;
    case ANTPLUS_CHANNEL_RESPONSE:
        antplus_decode_channel_response_message(driver, (antplus_channel_reponse_message_t*)msg);
        break;
    case ANTPLUS_CHANNEL_ID:
        antplus_decode_channel_id_message(driver, (antplus_channel_id_message_t*)msg);
        break;
    case ANTPLUS_BROADCAST:
        antplus_decode_channel_broadcast_message(driver, (antplus_broadcast_message_t*)msg);
        break;
    case ANTPLUS_ACKNOWEDGED_DATA:
        ESP_LOGI(TAG, "data acknowledged on channel %u", ((antplus_broadcast_message_t*)msg)->data.channel);
        break;
    default:
        ESP_LOGI(TAG, "unknown msg 0x%02x", msg->msg_id);
        break;
    }

    return true;
}

void antplus_recv_task(void* param)
{
    antplus_driver_t* driver = (antplus_driver_t*)param;

    uint8_t msg[ANTPLUS_MAX_MESSAGE_SIZE];
    uint8_t msg_size = 0;
    uint8_t buffer[ANTPLUS_MAX_MESSAGE_SIZE];
    
    for (;;)
    {
        size_t to_read;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(driver->uart_num, &to_read));
        if (to_read == 0)
        {
            to_read = 1;
        }
        else if (to_read > ANTPLUS_MAX_MESSAGE_SIZE)
        {
            to_read = ANTPLUS_MAX_MESSAGE_SIZE;
        }
        //printf("request %u\n", to_read);
        int read = uart_read_bytes(driver->uart_num, buffer, to_read, portMAX_DELAY);
        if (read > 0)
        {
            // printf("read");
            // for (int i = 0; i < read; i++)
            // {
            //     printf(" %02x", buffer[i]);
            // }
            // printf("\n");
            for (int i = 0; i < read; i++)
            {
                if ((msg_size == 0 && (ANTPLUS_SYNC_BYTE_MASK & buffer[i]) == ANTPLUS_SYNC_BYTE_VALUE) || msg_size > 0)
                {
                    msg[msg_size++] = buffer[i];
                    if (antplus_decode_message(driver, msg, msg_size))
                    {
                        msg_size = 0;
                    }
                }
            }

        }
    }
}

esp_err_t antplus_send_message(antplus_driver_t* driver, antplus_message_t* msg)
{
    //msg->direction = ANTPLUS_DIRECTION_OUTPUT;
    //msg->sync_value = ANTPLUS_SYNC_VALUE;

    msg->data[msg->size] = 0;
    uint8_t* msg_buffer = (uint8_t*)msg;
    uint8_t msg_size = ANTPLUS_MESSAGE_REQUIRED_SIZE(msg);
    for (int i = 0; i < msg_size - 1; i++)
    {
        msg->data[msg->size] ^= msg_buffer[i];
    }

    uint8_t padding[2] = { 0 };

    // printf("write (size %u, cts %u): ", msg_size, gpio_get_level(ANTPLUS_CTS_GPIO_NUM));
    // for (int i = 0; i < msg_size; i++)
    // {
    //     printf(" %02x", msg_buffer[i]);
    // }
    // printf("\n");
    int written = uart_write_bytes(driver->uart_num, msg, msg_size);
    uart_write_bytes(driver->uart_num, padding, 2);

    return ESP_OK;
}

esp_err_t antplus_request_message(antplus_driver_t* driver, uint8_t channel, uint8_t msg_id)
{
    antplus_request_message_t msg = ANTPLUS_MESSAGE(antplus_request_message_t, ANTPLUS_REQUEST_MESSAGE, ANTPLUS_INPUT);

    msg.data.channel = channel;
    msg.data.msg_id = msg_id;

    return antplus_send_message(driver, (antplus_message_t*)&msg);
}

esp_err_t antplus_set_networkkey(antplus_driver_t* driver, uint8_t network, uint8_t* network_key)
{
    antplus_set_network_key_message_t msg = ANTPLUS_MESSAGE(antplus_set_network_key_message_t, ANTPLUS_SET_NETWORK_KEY, ANTPLUS_INPUT);

    msg.data.network = network;
    memcpy(msg.data.key, network_key, ANTPLUS_NETWORK_KEY_SIZE);

    return antplus_send_message(driver, (antplus_message_t*)&msg);
}

esp_err_t antplus_assign_channel(antplus_driver_t* driver, uint8_t channel, uint8_t channel_type, uint8_t network)
{
    antplus_assign_channel_message_t msg = ANTPLUS_MESSAGE(antplus_assign_channel_message_t, ANTPLUS_ASSIGN_CHANNEL, ANTPLUS_INPUT);

    msg.data.channel = channel;
    msg.data.channel_type = channel_type;
    msg.data.network = network;

    return antplus_send_message(driver, (antplus_message_t*)&msg);
}

esp_err_t antplus_set_channel_id(antplus_driver_t* driver, uint8_t channel, uint16_t device, uint8_t device_type, uint8_t tx_type)
{
    antplus_channel_id_message_t msg = ANTPLUS_MESSAGE(antplus_channel_id_message_t, ANTPLUS_CHANNEL_ID, ANTPLUS_INPUT);

    msg.data.channel = channel;
    msg.data.device_num = device;
    msg.data.device_type = device_type;
    msg.data.tx_type = tx_type;

    return antplus_send_message(driver, (antplus_message_t*)&msg);
}

esp_err_t antplus_set_channel_rf_freq(antplus_driver_t* driver, uint8_t channel, uint8_t rf_freq)
{
    antplus_set_channel_rf_freq_message_t msg = ANTPLUS_MESSAGE(antplus_set_channel_rf_freq_message_t, ANTPLUS_SET_CHANNEL_RF_FREQ, ANTPLUS_INPUT);

    msg.data.channel = channel;
    msg.data.rf_freq = rf_freq;

    return antplus_send_message(driver, (antplus_message_t*)&msg);
}

esp_err_t antplus_set_channel_period(antplus_driver_t* driver, uint8_t channel, uint16_t period)
{
    antplus_set_channel_period_message_t msg = ANTPLUS_MESSAGE(antplus_set_channel_period_message_t, ANTPLUS_SET_CHANNEL_PERIOD, ANTPLUS_INPUT);

    msg.data.channel = channel;
    msg.data.period = period;

    return antplus_send_message(driver, (antplus_message_t*)&msg);
}

esp_err_t antplus_set_channel_tx_power(antplus_driver_t* driver, uint8_t channel, uint8_t tx_power)
{
    antplus_set_channel_tx_power_message_t msg = ANTPLUS_MESSAGE(antplus_set_channel_tx_power_message_t, ANTPLUS_SET_CHANNEL_TX_POWER, ANTPLUS_INPUT);

    msg.data.channel = channel;
    msg.data.tx_power = tx_power;

    return antplus_send_message(driver, (antplus_message_t*)&msg);
}

esp_err_t antplus_set_channel_search_timeout(antplus_driver_t* driver, uint8_t channel, uint8_t time_out)
{
    antplus_set_channel_search_timeout_message_t msg = ANTPLUS_MESSAGE(antplus_set_channel_search_timeout_message_t, ANTPLUS_SET_CHANNEL_SEARCH_TIMEOUT, ANTPLUS_INPUT);

    msg.data.channel = channel;
    msg.data.time_out = time_out;

    return antplus_send_message(driver, (antplus_message_t*)&msg);
}

esp_err_t antplus_open_channel(antplus_driver_t* driver, uint8_t channel)
{
    antplus_open_channel_message_t msg = ANTPLUS_MESSAGE(antplus_open_channel_message_t, ANTPLUS_OPEN_CHANNEL, ANTPLUS_INPUT);

    msg.data.channel = channel;

    return antplus_send_message(driver, (antplus_message_t*)&msg);
}

esp_err_t antplus_init(state_manager_handle_t sm_handle, hrm_handle_t* hrm_handle, s4_handle_t s4_handle, antplus_handle_t* antplus_handle)
{
    antplus_driver_t* driver = malloc(sizeof(antplus_driver_t));
    memset(driver, 0, sizeof(antplus_driver_t));

    driver->eg_handle = xEventGroupCreate();
    portMUX_INITIALIZE(&driver->mux);
    driver->sm_handle = sm_handle;
    driver->hrm_handle = hrm_handle;
    driver->s4_handle = s4_handle;
    driver->uart_num = ANTPLUS_UART_NUM;
    
    gpio_config_t io_conf;

    io_conf.pin_bit_mask = ANTPLUS_RESET_GPIO_SEL;
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&io_conf);

    uart_config_t uart_config = {
        .baud_rate = ANTPLUS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS,
        //.rx_flow_ctrl_thresh = 122
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_param_config(driver->uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(driver->uart_num, ANTPLUS_TX_GPIO_NUM, ANTPLUS_RX_GPIO_NUM, -1, ANTPLUS_CTS_GPIO_NUM));

    ESP_ERROR_CHECK(uart_driver_install(driver->uart_num,  ANTPLUS_UART_RX_BUFFER_SIZE, ANTPLUS_UART_TX_BUFFER_SIZE, 
        ANTPLUS_UART_QUEUE_SIZE, &driver->uart_queue_handle, 0));

    xTaskCreate(antplus_recv_task, "antplus_recv_task", 4096, driver, 23, &driver->recv_task_handle);

    ESP_ERROR_CHECK(antplus_reset(driver));

    xTaskCreate(antplus_trainer_task, "antplus_trainer_task", 4096, driver, 23, NULL);

    *antplus_handle = driver;

    printf("ANT+ initialized\n");

    return ESP_OK;
}


esp_err_t antplus_reset(antplus_handle_t antplus_handle)
{
    antplus_driver_t* driver = (antplus_driver_t*)antplus_handle;

    xEventGroupClearBits(driver->eg_handle, ANTPLUS_EVENT_READY);
    ESP_ERROR_CHECK(uart_flush(driver->uart_num));

    ESP_ERROR_CHECK(gpio_set_level(ANTPLUS_RESET_GPIO_NUM, 0));
    vTaskDelay(500 / portTICK_PERIOD_MS);   

    ESP_ERROR_CHECK(gpio_set_level(ANTPLUS_RESET_GPIO_NUM, 1));
    vTaskDelay(500 / portTICK_PERIOD_MS);

    return ESP_OK;
}

void antplus_trainer_task(void* param)
{
    antplus_driver_t* driver = (antplus_driver_t*)param;

    for (uint8_t i = 0; ; i++)
    {
        xEventGroupWaitBits(driver->eg_handle, ANTPLUS_EVENT_READY, pdFALSE, pdTRUE, portMAX_DELAY);

        s4_values_t values;
        if (s4_get_values(driver->s4_handle, &values) == ESP_OK)
        {
            uint8_t heart_rate;
            hrm_get_rate(driver->hrm_handle, &heart_rate);

            state_manager_set_component_state(driver->sm_handle, STATE_MANAGER_COMPONENT_ANT, values.stroke_rate_x2 > 0);

            antplus_broadcast_message_t msg = ANTPLUS_MESSAGE(antplus_broadcast_message_t, ANTPLUS_BROADCAST, ANTPLUS_INPUT);
            msg.data.channel = ANTPLUS_CHANNEL_FE;

            if (i % 64 == 30 || i % 64 == 31)
            {
                antplus_manufacturer_id_page_t* page = (antplus_manufacturer_id_page_t*)&msg;
                page->data.page_num = ANTPLUS_PAGE_MANUFACTURER_ID;
                page->data.reserved0 = 0xff;
                page->data.reserved1 = 0xff;
                page->data.hw_revision = 123;
                page->data.manufacturer_id = 123;
                page->data.model_no = 123;
            }
            else if (i % 64 == 62 || i % 64 == 63)
            {
                antplus_product_info_page_t* page = (antplus_product_info_page_t*)&msg;
                page->data.page_num = ANTPLUS_PAGE_PRODUCT_INFO;
                page->data.reserved = 0xff;
                page->data.sw_rev_supp = 0xff;
                page->data.sw_rev_main = 123;
                page->data.serial_no = 123;
            }
            else if (i % 3 == 0 || i % 4 == 0)
            {
                antplus_rowing_data_page_t* page = (antplus_rowing_data_page_t*)&msg;
                page->data.page_num = ANTPLUS_PAGE_ROWING_DATA;
                page->data.reserved0 = 0xff;
                page->data.reserved1 = 0xff;
                page->data.stroke_count = values.stroke_count % 256;
                page->data.cadence = values.stroke_rate_x2 / 2;
                page->data.power = values.power;
                page->data.flags = ANTPLUS_CAPS_STROKES;
                //ESP_LOGI(TAG, "ANT+ power %u", values.power);
            }
            else
            {
                antplus_general_fe_page_t* page = (antplus_general_fe_page_t*)&msg;
                page->data.page_num = ANTPLUS_PAGE_GENERAL_FE;
                page->data.equipment_type = ANTPLUS_ROWER;
                page->data.elasped = ((uint32_t)(values.timer * 4)) % 256;
                page->data.distance = (values.distance % 256);
                page->data.speed = values.current_speed * 10;
                page->data.heart_rate = heart_rate;
                page->data.flags = ANTPLUS_CAPS_HR_ANT | ANTPLUS_CAPS_DISTANCE;
            }

            antplus_send_message(driver, (antplus_message_t*)&msg);
        }
        else
        {
            state_manager_set_component_state(driver->sm_handle, STATE_MANAGER_COMPONENT_ANT, false);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

esp_err_t antplus_setup(antplus_driver_t* driver)
{
    antplus_request_message(driver, 0, ANTPLUS_CAPABILITIES);
    antplus_request_message(driver, 0, ANTPLUS_VERSION);

    uint8_t antplus_network_key[] = ANTPLUS_NETWORK_KEY;
    antplus_set_networkkey(driver, ANTPLUS_PUBLIC_NETWORK, antplus_network_key);

    // hrm
    antplus_assign_channel(driver, ANTPLUS_CHANNEL_HRM, ANTPLUS_BIDIRECTIONAL_RECEIVE, ANTPLUS_PUBLIC_NETWORK);
    antplus_set_channel_id(driver, ANTPLUS_CHANNEL_HRM, ANTPLUS_ANY, ANTPLUS_HRM, ANTPLUS_ANY);
    antplus_set_channel_rf_freq(driver, ANTPLUS_CHANNEL_HRM, ANTPLUS_2457MHZ);
    antplus_set_channel_period(driver, ANTPLUS_CHANNEL_HRM, 8070);
    antplus_set_channel_tx_power(driver, ANTPLUS_CHANNEL_HRM, ANTPLUS_0DBM);
    antplus_set_channel_search_timeout(driver, ANTPLUS_CHANNEL_HRM, ANTPLUS_INFINITE_TIMEOUT);
    antplus_open_channel(driver, ANTPLUS_CHANNEL_HRM);
    antplus_request_message(driver, ANTPLUS_CHANNEL_HRM, ANTPLUS_CHANNEL_ID);

    //fe
    antplus_assign_channel(driver, ANTPLUS_CHANNEL_FE, ANTPLUS_BIDIRECTIONAL_TRANSMIT, ANTPLUS_PUBLIC_NETWORK);
    antplus_set_channel_id(driver, ANTPLUS_CHANNEL_FE, ANTPLUS_FE_DEVICE_NUM, ANTPLUS_FE, ANTPLUS_IC | ANTPLUS_GDP);
    antplus_set_channel_rf_freq(driver, ANTPLUS_CHANNEL_FE, ANTPLUS_2457MHZ);
    antplus_set_channel_period(driver, ANTPLUS_CHANNEL_FE, 8192);
    antplus_set_channel_tx_power(driver, ANTPLUS_CHANNEL_FE, ANTPLUS_0DBM);
    antplus_open_channel(driver, ANTPLUS_CHANNEL_FE);
    xEventGroupSetBits(driver->eg_handle, ANTPLUS_EVENT_READY);

    return ESP_OK;
}
