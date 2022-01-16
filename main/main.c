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
#include "usb.h"
#include "waterrower.h"
#include "main.h"

#define TAG "main"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define WATERROWER_APP_ID           0x55
//#define BIKE_APP_ID                 0x57
//#define HR_APP_ID                   0x58
#define WATERROWER_DEVICE_NAME      "WaterRowerLink+"
#define CSC_SVC_INST_ID             0
#define FTMS_SVC_INST_ID            1
#define HR_SVC_INST_ID              2

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done = 0;

uint16_t csc_handle_table[CSC_IDX_NB];
uint16_t ftms_handle_table[FTMS_IDX_NB];
uint16_t hr_handle_table[HR_IDX_NB];

typedef struct
{
    uint8_t* prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

static uint8_t raw_adv_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power*/
    //0x02, 0x0a, 0xeb,
    /* service uuid */
    0x07, 0x03, 0x16, 0x18, 0x0d, 0x18, 0x26, 0x18,
    /* device name */
    0x05, 0x09, 'W', 'R', 'L', '+', 
    /* ftms service data rower*/
    //0x06, 0x16, 0x26, 0x18, 0x01, 0b00110000, 0b00000000,
    /* ftms service rower*/
    0x06, 0x16, 0x26, 0x18, 0x01, 0b00010000, 0b00000000,
    /* manufactuer data */
    0x05, 0xff,0xff, 0xff, 0x77, 0x72
};

static uint8_t raw_scan_rsp_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power */
    0x02, 0x0a, 0xeb,
    /* service uuid */
    //0x05, 0x03, 0x16, 0x18, 0x0d, 0x18
    //0x0f, 0x09, 'W', 'a', 't', 'e', 'r', 'R', 'o', 'w', 'e', 'r', 'L', 'i', 'n', 'k', '+', 
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
    esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

#if 0
/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst csc_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};
#endif

static const uint16_t cycling_speed_cadence_svc_uuid = ESP_GATT_UUID_CYCLING_SPEED_CADENCE_SVC;
static const uint16_t csc_measurement_uuid = ESP_GATT_UUID_CSC_MEASUREMENT;
static const uint16_t csc_feature_uuid = ESP_GATT_UUID_CSC_FEATURE;
static const uint16_t sensor_location_uuid = ESP_GATT_UUID_SENSOR_LOCATION;
static const uint16_t sc_control_point_uuid = ESP_GATT_UUID_SC_CONTROL_POINT;

static const uint16_t hr_svc_uuid = ESP_GATT_UUID_HEART_RATE_SVC;
static const uint16_t hr_measurement_uuid = ESP_GATT_HEART_RATE_MEAS;
static const uint16_t body_sensor_location_uuid = ESP_GATT_BODY_SENSOR_LOCATION;
static const uint16_t hr_control_point_uuid = ESP_GATT_HEART_RATE_CNTL_POINT;

static const uint16_t ftms_svc_uuid = 0x1826;
static const uint16_t ftms_feature_uuid = 0x2acc;
static const uint16_t ftms_rower_data_uuid = 0x2ad1;
//static const uint16_t ftms_indoor_bike_data_uuid = 0x2ad2;
static const uint16_t ftms_control_point_uuid = 0x2ad9;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
//static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static uint8_t csc_feature_value[2] = { 0x02, 0x00 };
static uint8_t csc_sensor_location_value[1] = { 0x05 };
//static uint8_t sc_control_point_value[1] = { 0x00 };
static const uint8_t csc_measurement_ccc[2] = { 0x00, 0x00 };
//static const uint8_t csc_sc_control_point_ccc[2] = { 0x00, 0x00 };

static const uint8_t hr_measurement_ccc[2] = { 0x00, 0x00 };
static const uint8_t hr_body_sensor_location_value[1] = { 0x00 };
//static const uint8_t hr_control_point_value[1] = { 0x00 };

static const uint8_t ftms_rower_data_ccc[2] = { 0x00, 0x00 };
//static const uint8_t ftms_indoor_bike_data_ccc[2] = { 0x00, 0x00 };
                                                                      
static const uint8_t ftms_feature_value[8] = 
{
    0b00100110, 0b01010110, 0b00000000, 0b00000000, 
    0b00000000, 0b00000000, 0b00000000, 0b00000000, 
};


static const esp_gatts_attr_db_t gatt_ftms_db[FTMS_IDX_NB] =
{
    // FTMS Service
    [IDX_FTMS_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ,
        sizeof(uint16_t), sizeof(hr_svc_uuid), (uint8_t*)&ftms_svc_uuid}},

    /// FTMS Feature
    [IDX_FTMS_FEATURE] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read}},

    // FTMS Feature Value
    [IDX_FTMS_FEATURE_VAL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&ftms_feature_uuid, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(ftms_feature_value), (uint8_t*)ftms_feature_value}},

    // Rower Data
    [IDX_FTMS_ROWER_DATA] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_notify}},

    // Rower Data Value
    [IDX_FTMS_ROWER_DATA_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t*)&ftms_rower_data_uuid, 0,
        sizeof(uint16_t), 0, NULL}},

    // // Indoor Bike Data
    // [IDX_FTMS_INDOOR_BIKE_DATA] =
    //     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
    //     CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_notify}},

    // // Indoor Bike Value
    // [IDX_FTMS_INDOOR_BIKE_DATA_VAL] =
    //     {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t*)&ftms_indoor_bike_data_uuid, 0,
    //     sizeof(uint16_t), 0, NULL}},

    // // Indoor Bike Config
    // [IDX_FTMS_INDOOR_BIKE_DATA_CFG] =
    //     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    //     sizeof(uint16_t), sizeof(ftms_indoor_bike_data_ccc), (uint8_t*)ftms_indoor_bike_data_ccc}},

    // FTMS Control Point
    [IDX_FTMS_CONTROL_POINT] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_write}},

    // FTMS Control Point Value
    [IDX_FTMS_CONTROL_POINT_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t*)&ftms_control_point_uuid, ESP_GATT_PERM_WRITE,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, NULL}},

};

static const esp_gatts_attr_db_t gatt_csc_db[CSC_IDX_NB] =
{
    // CSC Service
    [IDX_CSC_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ,
        sizeof(uint16_t), sizeof(cycling_speed_cadence_svc_uuid), (uint8_t*)&cycling_speed_cadence_svc_uuid}},

        // CSC Measurement
    [IDX_CSC_MEASUREMENT] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_notify}},

    // CSC Measurement Value
    [IDX_CSC_MEASUREMENT_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t*)&csc_measurement_uuid, 0,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, NULL}},

    // CSC Measurement Config
    [IDX_CSC_MEASUREMENT_CFG] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(csc_measurement_ccc), (uint8_t*)csc_measurement_ccc}},

    /// CSC Feature
    [IDX_CSC_FEATURE] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read}},

    // CSC Feature Value
    [IDX_CSC_FEATURE_VAL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&csc_feature_uuid, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(csc_feature_value), (uint8_t*)csc_feature_value}},

    // Sensor Location
    [IDX_SENSOR_LOCATION] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read}},

    // Sensor Location Value
    [IDX_SENSOR_LOCATION_VAL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&sensor_location_uuid, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(csc_sensor_location_value), (uint8_t*)csc_sensor_location_value}},

    // SC Control Point
    [IDX_CSC_SC_CONTROL_POINT] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_write}},

    // SC Control Point Value
    [IDX_CSC_SC_CONTROL_POINT_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t*)&sc_control_point_uuid, ESP_GATT_PERM_WRITE,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, NULL}},
};

static const esp_gatts_attr_db_t gatt_hr_db[HR_IDX_NB] =
{
    // HR Service
    [IDX_HR_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ,
        sizeof(uint16_t), sizeof(hr_svc_uuid), (uint8_t*)&hr_svc_uuid}},

    // HR Measurement
    [IDX_HR_MEASUREMENT] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_notify}},

    // HR Measurement Value
    [IDX_HR_MEASUREMENT_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t*)&hr_measurement_uuid, 0,
        sizeof(uint16_t), 0, NULL}},

    // HR Measurement Config
    [IDX_HR_MEASUREMENT_CFG] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(hr_measurement_ccc), (uint8_t*)hr_measurement_ccc}},

    // Body Sensor Location
    [IDX_BODY_SENSOR_LOCATION] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read}},

    // Body Sensor Location Value
    [IDX_BODY_SENSOR_LOCATION_VAL] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&body_sensor_location_uuid, ESP_GATT_PERM_READ,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(hr_body_sensor_location_value), (uint8_t*)hr_body_sensor_location_value}},

    // HR Control Point
    [IDX_HR_CONTROL_POINT] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_write}},

    // HR Control Point Value
    [IDX_HR_CONTROL_POINT_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t*)&hr_control_point_uuid, ESP_GATT_PERM_WRITE,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, NULL}},
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0)
            {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        // case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        //     adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        //     if (adv_config_done == 0)
        //     {
        //         esp_ble_gap_start_advertising(&adv_params);
        //     }
        //     break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "advertising start failed");
            }
            else
            {
                ESP_LOGI(TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Advertising stop failed");
            }
            else
            {
                ESP_LOGI(TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                param->update_conn_params.status,
                param->update_conn_params.min_int,
                param->update_conn_params.max_int,
                param->update_conn_params.conn_int,
                param->update_conn_params.latency,
                param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param)
{
    ESP_LOGI(TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL)
    {
        prepare_write_env->prepare_buf = (uint8_t*)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL)
        {
            ESP_LOGE(TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    else
    {
        if (param->write.offset > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_OFFSET;
        }
        else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp)
    {
        esp_gatt_rsp_t* gatt_rsp = (esp_gatt_rsp_t*)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL)
        {
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK)
            {
                ESP_LOGE(TAG, "Send response error");
            }
            free(gatt_rsp);
        }
        else
        {
            ESP_LOGE(TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK)
    {
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
        param->write.value,
        param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
    {
        esp_log_buffer_hex(TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

// #define NOP() asm volatile ("nop")

// void delay(uint32_t ms)
// {
//     vTaskDelay(ms / portTICK_PERIOD_MS);
// }

// void IRAM_ATTR delayMicroseconds(uint32_t us)
// {
//     uint64_t m = (uint64_t)esp_timer_get_time();
//     if (us)
//     {
//         uint64_t e = (m + us);
//         if (m > e)
//         { //overflow
//             while ((uint64_t)esp_timer_get_time() > e)
//             {
//                 NOP();
//             }
//         }
//         while ((uint64_t)esp_timer_get_time() < e)
//         {
//             NOP();
//         }
//     }
// }

void notify_hr_measurement(void* p)
{
    int hr = 60;
    bool increase = true;

    esp_gatt_if_t gatts_if = ((intptr_t)p) >> 16;
    uint16_t conn_id = ((intptr_t)p) & 0xffff;

    for (;;)
    {
        hr += increase ? 1 : -1;
        if (hr < 60 || hr > 200)
        {
            increase = !increase;
        }

        uint8_t hr_measurement_value[2];
        hr_measurement_value[0] = hr >> 8;
        hr_measurement_value[1] = hr & 0xff;

        ESP_LOGI(TAG, "gatts_if: %u conn_id: %u hr %d", gatts_if, conn_id, hr);
        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, hr_handle_table[IDX_HR_MEASUREMENT_VAL], sizeof(hr_measurement_value), hr_measurement_value, false);
        if (ret != ESP_OK)
        {
            ESP_LOGI(TAG, "HR notify error (%d)", ret);
            vTaskDelete(NULL);
        }


        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

#define FTMS_DATA_MORE_DATA       0b0000000000000001
#define FTMS_CADENCE              0b0000000000000010
#define FTMS_TOTAL_DISTANCE       0b0000000000000100
#define FTMS_CURRENT_PACE         0b0000000000001000
#define FTMS_AVG_PACE             0b0000000000010000
#define FTMS_CURRENT_POWER        0b0000000000100000
#define FTMS_AVG_POWER            0b0000000001000000
#define FTMS_RESISTANCE_LEVEL     0b0000000010000000

#define FTMS_EXPENDED_ENERGY      0b0000000100000000
#define FTMS_HEART_RATE           0b0000001000000000
#define FTMS_META_EQU             0b0000010000000000
#define FTMS_ELASPED_TIME         0b0000100000000000
#define FTMS_REMAINING_TIME       0b0001000000000000

void notify_ftms_rower_data(void* p)
{
    uint16_t stroke_rate = 22 * 2;
    uint16_t total_strokes = 0;
    uint32_t total_distance = 0;
    uint16_t pace = 130;
    uint16_t watts = 100;
    uint16_t total_cal = 55;
    uint16_t cal_hour = 1000;
    uint16_t cal_min = 22;
    uint8_t heart_rate = 88;
    uint16_t elapsed;

    esp_gatt_if_t gatts_if = ((intptr_t)p) >> 16;
    uint16_t conn_id = ((intptr_t)p) & 0xffff;

    int64_t start = esp_timer_get_time();

    for (;;)
    {

        elapsed = (uint16_t)((esp_timer_get_time() - start) / 1000000ul);

        uint8_t rower_data_value[20];

        rower_data_value[0] = FTMS_TOTAL_DISTANCE | FTMS_CURRENT_PACE | FTMS_CURRENT_POWER;
        rower_data_value[1] = (FTMS_EXPENDED_ENERGY | FTMS_HEART_RATE | FTMS_ELASPED_TIME) >> 8;
        rower_data_value[2] = stroke_rate & 0xff;
        rower_data_value[3] = total_strokes & 0xff;
        rower_data_value[4] = (total_strokes >> 8) & 0xff;
        rower_data_value[5] = total_distance & 0xff;
        rower_data_value[6] = (total_distance >> 8) & 0xff;
        rower_data_value[7] = (total_distance >> 16) & 0xff;
        rower_data_value[8] = pace & 0xff;
        rower_data_value[9] = (pace >> 8) & 0xff;
        rower_data_value[10] = watts & 0xff;
        rower_data_value[11] = (watts >> 8) & 0xff;
        rower_data_value[12] = total_cal & 0xff;
        rower_data_value[13] = (total_cal >> 8) & 0xff;
        rower_data_value[14] = cal_hour & 0xff;
        rower_data_value[15] = (cal_hour >> 8) & 0xff;
        rower_data_value[16] = cal_min & 0xff;
        rower_data_value[17] = heart_rate & 0xff;
        rower_data_value[18] = elapsed & 0xff;
        rower_data_value[19] = (elapsed >> 8) & 0xff;

        ESP_LOGI(TAG, "gatts_if: %u conn_id: %u spm %d strokes %d", gatts_if, conn_id, stroke_rate, total_strokes);
        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, ftms_handle_table[IDX_FTMS_ROWER_DATA_VAL], sizeof(rower_data_value), rower_data_value, false);
        if (ret != ESP_OK)
        {
            ESP_LOGI(TAG, "FTMS rower data notify error (%d)", ret);
            vTaskDelete(NULL);
        }


        total_strokes++;
        total_cal++;

        total_distance += 100;

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void notify_csc_measurement(void* p)
{
    int strokes_per_minute = 121;

    double strokes_per_second = strokes_per_minute / 60.0;

    esp_gatt_if_t gatts_if = ((intptr_t)p) >> 16;
    uint16_t conn_id = ((intptr_t)p) & 0xffff;

    //double raw_cranks = 0.0;
    int64_t last_now = -1;
    int32_t last_crank_revs = -1;
    uint16_t last_ts = 0;

    //double spm_ms = spm / 60.0 / 1000.0;
    double period = 1.0 / strokes_per_minute * 60.0 * 1024.0;

    ESP_LOGI(TAG, "revolution time: %f 1/1024s", period);
    double fractional_crank_revs = 0;

    for (;;)
    {
        int64_t now = esp_timer_get_time();

        if (last_now != -1)
        {
            double elasped = (now - last_now) / 1000000.0;

            double new_cranks = strokes_per_second * elasped;

            //ESP_LOGI(TAG, "elasped: %f strokes_per_second: %f new_cranks: %f total_cranks: %f", elasped, strokes_per_second, new_cranks, fractional_crank_revs);

            fractional_crank_revs += new_cranks;
        }

        int crank_revs = (int)fractional_crank_revs;
        uint16_t ts = ((int)(crank_revs * period)) & 0xffff;

        if (crank_revs != last_crank_revs)
        {
            uint8_t csc_measurement_value[5];

            csc_measurement_value[0] = 0x02;
            csc_measurement_value[1] = crank_revs & 0xff;
            csc_measurement_value[2] = (crank_revs >> 8) & 0xff;
            csc_measurement_value[3] = ts & 0xff;
            csc_measurement_value[4] = (ts >> 8) & 0xff;

            esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, csc_handle_table[IDX_CSC_MEASUREMENT_VAL], sizeof(csc_measurement_value), csc_measurement_value, false);
            if (ret != ESP_OK)
            {
                ESP_LOGI(TAG, "CSC notify error (%d)", ret);
                vTaskDelete(NULL);
            }

            if (last_ts != 0)
            {
                unsigned int cranks_diff = crank_revs - last_crank_revs;
                unsigned int time_diff = ts - last_ts;
                unsigned int cad = cranks_diff * 60 * 1024 / time_diff;

                ESP_LOGI(TAG, "gatts_if: %u conn_id: %u cranks: %u => %u (%u) time: %u => %u (%u) cadence: %u", gatts_if, conn_id, last_crank_revs, crank_revs, cranks_diff, last_ts, ts, time_diff, cad);
            }

            last_crank_revs = crank_revs;
            last_ts = ts;
            last_now = now;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
        {
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(WATERROWER_DEVICE_NAME);
            if (set_dev_name_ret)
            {
                ESP_LOGE(TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret)
            {
                ESP_LOGE(TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            // esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            // if (raw_scan_ret)
            // {
            //     ESP_LOGE(TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            // }
            // adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(gatt_csc_db, gatts_if, CSC_IDX_NB, CSC_SVC_INST_ID));
            ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(gatt_ftms_db, gatts_if, FTMS_IDX_NB, FTMS_SVC_INST_ID));
            ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(gatt_hr_db, gatts_if, HR_IDX_NB, HR_SVC_INST_ID));
            break;
        }
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_READ_EVT");
            break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep)
            {
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(TAG, param->write.value, param->write.len);
                if (csc_handle_table[IDX_CSC_MEASUREMENT_CFG] == param->write.handle && param->write.len == 2)
                {
                    ESP_LOGI(TAG, "start CSC notify");
                    uint16_t descr_value = (param->write.value[1] << 8) | param->write.value[0];
                    if ((descr_value & 1) == 1)
                    {
                        uintptr_t p = gatts_if << 16 | param->write.conn_id;
                        xTaskCreate(notify_csc_measurement, "notify_csc_measurement", 8000, (void*)p, 1, NULL);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "kill csc notify %u %u", gatts_if, param->write.conn_id);
                    }
                }
                else if (hr_handle_table[IDX_HR_MEASUREMENT_CFG] == param->write.handle && param->write.len == 2)
                {
                    ESP_LOGI(TAG, "start HR notify");
                    uint16_t descr_value = (param->write.value[1] << 8) | param->write.value[0];
                    if ((descr_value & 1) == 1)
                    {
                        uintptr_t p = gatts_if << 16 | param->write.conn_id;
                        xTaskCreate(notify_hr_measurement, "notify_hr_measurement", 8000, (void*)p, 1, NULL);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "kill hr notify %u %u", gatts_if, param->write.conn_id);
                    }
                }
                else if (ftms_handle_table[IDX_FTMS_ROWER_DATA_CFG] == param->write.handle && param->write.len == 2)
                {
                    ESP_LOGI(TAG, "start rower data notify");
                    uint16_t descr_value = (param->write.value[1] << 8) | param->write.value[0];
                    if ((descr_value & 1) == 1)
                    {
                        uintptr_t p = gatts_if << 16 | param->write.conn_id;
                        xTaskCreate(notify_ftms_rower_data, "notify_rower_data", 8000, (void*)p, 1, NULL);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "kill rower data notify %u %u", gatts_if, param->write.conn_id);
                    }
                }
                // else if (ftms_handle_table[IDX_FTMS_INDOOR_BIKE_DATA_CFG] == param->write.handle && param->write.len == 2)
                // {
                //     ESP_LOGI(TAG, "start indoor bike data notify");
                //     uint16_t descr_value = (param->write.value[1] << 8) | param->write.value[0];
                //     if ((descr_value & 1) == 1)
                //     {
                //         uintptr_t p = gatts_if << 16 | param->write.conn_id;
                //         xTaskCreate(notify_ftms_rower_data, "notify_indoor_bike_data", 8000, (void*)p, 1, NULL);
                //     }
                //     else
                //     {
                //         ESP_LOGI(TAG, "kill indoor bike data notify %u %u", gatts_if, param->write.conn_id);
                //     }
                // }
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp)
                {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            else
            {
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX. 
            ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
        {
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = { 0 };
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);

            esp_ble_gap_start_advertising(&adv_params);
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        {
            if (param->add_attr_tab.status != ESP_GATT_OK)
            {
                ESP_LOGE(TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else
            {
                ESP_LOGI(TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
                switch (param->add_attr_tab.svc_inst_id)
                {
                    case FTMS_SVC_INST_ID:
                        memcpy(ftms_handle_table, param->add_attr_tab.handles, sizeof(ftms_handle_table));
                        esp_ble_gatts_start_service(ftms_handle_table[IDX_FTMS_SVC]);
                        break;
                    case HR_SVC_INST_ID:
                        memcpy(hr_handle_table, param->add_attr_tab.handles, sizeof(hr_handle_table));
                        esp_ble_gatts_start_service(hr_handle_table[IDX_HR_SVC]);
                        break;
                    case CSC_SVC_INST_ID:
                        memcpy(csc_handle_table, param->add_attr_tab.handles, sizeof(csc_handle_table));
                        esp_ble_gatts_start_service(csc_handle_table[IDX_CSC_SVC]);
                        break;
                }
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

#if 0
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            csc_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(TAG, "reg app failed, app_id %04x, status %d",
                param->reg.app_id,
                param->reg.status);
            return;
        }
    }
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++)
    {
        /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == csc_profile_tab[idx].gatts_if)
        {
            if (csc_profile_tab[idx].gatts_cb)
            {
                csc_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}
#endif

//extern "C"
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(usb_init());
    ESP_ERROR_CHECK(waterrower_init());

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    //ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_profile_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(WATERROWER_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));
}
