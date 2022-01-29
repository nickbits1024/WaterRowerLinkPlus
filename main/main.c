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
#include "antplus.h"
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

waterrower_handle_t waterrower_handle;
antplus_handle_t antplus_handle;

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
    /* manufactuer data */
    0x05, 0xff,0xff, 0xff, 0x77, 0x72
    /* ftms service rower + indoor bike data*/
    //0x06, 0x16, 0x26, 0x18, 0x01, 0b00110000, 0b00000000,
};

// static uint8_t raw_scan_rsp_data[] = {
//     /* flags */
//     0x02, 0x01, 0x06,
//     /* tx power */
//     0x02, 0x0a, 0xeb,
//     //0x07, 0x26, 0x18,
//     /* service uuid */
//     //0x05, 0x03, 0x16, 0x18, 0x0d, 0x18
//     0x0f, 0x09, 'W', 'a', 't', 'e', 'r', 'R', 'o', 'w', 'e', 'r', 'L', 'i', 'n', 'k', '+',
// };

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

#define MAX_NOTIFY_TASKS 10

typedef struct
{
    uint16_t gatts_if;
    uint16_t conn_id;
    const char* task_name;
    TaskHandle_t task_handle;
}
notify_task_t;

notify_task_t notify_tasks[MAX_NOTIFY_TASKS];
portMUX_TYPE notify_tasks_mux = portMUX_INITIALIZER_UNLOCKED;

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
static const uint16_t ftms_indoor_bike_data_uuid = 0x2ad2;
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
static const uint8_t ftms_indoor_bike_data_ccc[2] = { 0x00, 0x00 };

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

    // Rower Data Config
    [IDX_FTMS_ROWER_DATA_CFG] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(ftms_rower_data_ccc), (uint8_t*)ftms_rower_data_ccc}},

    // Indoor Bike Data
    [IDX_FTMS_INDOOR_BIKE_DATA] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_notify}},

    // Indoor Bike Data Value
    [IDX_FTMS_INDOOR_BIKE_DATA_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t*)&ftms_indoor_bike_data_uuid, 0,
        sizeof(uint16_t), 0, NULL}},

    // Indoor Bike Data Config
    [IDX_FTMS_INDOOR_BIKE_DATA_CFG] =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        sizeof(uint16_t), sizeof(ftms_indoor_bike_data_ccc), (uint8_t*)ftms_indoor_bike_data_ccc}},

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
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0)
            {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
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

void create_notify_task(uint16_t gatts_if, uint16_t conn_id, const char* task_name, TaskFunction_t task_func)
{
    uintptr_t param = gatts_if << 16 | conn_id;
    bool task_found = false;
    bool task_created = false;

    portENTER_CRITICAL(&notify_tasks_mux);
    for (int i = 0; i < MAX_NOTIFY_TASKS; i++)
    {
        if (notify_tasks[i].task_handle != NULL &&
            notify_tasks[i].gatts_if == gatts_if &&
            notify_tasks[i].conn_id == conn_id &&
            strcmp(notify_tasks[i].task_name, task_name) == 0)
        {
            task_found = true;
        }
    }
    if (!task_found)
    {
        for (int i = 0; i < MAX_NOTIFY_TASKS; i++)
        {
            if (notify_tasks[i].task_handle == NULL)
            {
                notify_tasks[i].gatts_if = gatts_if;
                notify_tasks[i].conn_id = conn_id;
                notify_tasks[i].task_name = task_name;
                xTaskCreate(task_func, task_name, 4096, (void*)param, 23, &notify_tasks[i].task_handle);
                task_created = true;
                break;
            }

        }
    }
    portEXIT_CRITICAL(&notify_tasks_mux);

    if (task_found)
    {
        ESP_LOGW(TAG, "Skipping duplicate notify task %s creation on gatts_if %u conn_id %u", task_name, gatts_if, conn_id);
    }
    else
    {
        if (task_created)
        {
            ESP_LOGI(TAG, "Start notify task %s on gatts_if %u conn_id %u", task_name, gatts_if, conn_id);
        }
        else
        {
            ESP_LOGW(TAG, "No room for new notify task %s", task_name);
        }
    }
}

void kill_notify_task(uint16_t gatts_if, uint16_t conn_id, const char* task_name)
{
    for (int i = 0; i < MAX_NOTIFY_TASKS; i++)
    {
        bool task_killed = false;
        portENTER_CRITICAL(&notify_tasks_mux);
        if (notify_tasks[i].task_handle != NULL &&
            notify_tasks[i].gatts_if == gatts_if &&
            notify_tasks[i].conn_id == conn_id &&
            strcmp(notify_tasks[i].task_name, task_name) == 0)
        {
            vTaskDelete(notify_tasks[i].task_handle);
            memset(&notify_tasks[i], 0, sizeof(notify_task_t));
            task_killed = true;
        }
        portEXIT_CRITICAL(&notify_tasks_mux);
        if (task_killed)
        {
            ESP_LOGW(TAG, "Killing notify task %s on gatts_if %u conn_id %u", task_name, gatts_if, conn_id);
        }
    }

}

void kill_notify_tasks(uint16_t gatts_if, uint16_t conn_id)
{
    for (int i = 0; i < MAX_NOTIFY_TASKS; i++)
    {
        bool task_killed = false;
        const char* task_name;
        portENTER_CRITICAL(&notify_tasks_mux);
        if (notify_tasks[i].gatts_if == gatts_if && notify_tasks[i].conn_id == conn_id)
        {
            vTaskDelete(notify_tasks[i].task_handle);
            task_name = notify_tasks[i].task_name;
            memset(&notify_tasks[i], 0, sizeof(notify_task_t));
            task_killed = true;
        }
        portEXIT_CRITICAL(&notify_tasks_mux);
        if (task_killed)
        {
            ESP_LOGI(TAG, "kill notify task %s gatts_if %u conn_id %u", task_name, gatts_if, conn_id);
        }
    }
}

void notify_hr_measurement(void* p)
{
    esp_gatt_if_t gatts_if = ((intptr_t)p) >> 16;
    uint16_t conn_id = ((intptr_t)p) & 0xffff;

    for (;;)
    {
        waterrower_values_t values;
        esp_err_t ret = waterrower_get_values(waterrower_handle, &values);
        if (ret == ESP_OK)
        {
            uint8_t heart_rate = values.heart_rate;

            if (heart_rate == 0)
            {
                antplus_get_heart_rate(antplus_handle, &heart_rate);
            }

            uint8_t hr_measurement_value[2];
            hr_measurement_value[0] = heart_rate >> 8;
            hr_measurement_value[1] = heart_rate & 0xff;

            ESP_LOGI(TAG, "ble hr %d", heart_rate);
            esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, hr_handle_table[IDX_HR_MEASUREMENT_VAL], sizeof(hr_measurement_value), hr_measurement_value, false);
            if (ret != ESP_OK)
            {
                ESP_LOGI(TAG, "HR notify error (%d)", ret);
                vTaskDelete(NULL);
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

#define FTMS_ROWER_AVERAGE_STROKE               (1 << 1)
#define FTMS_ROWER_TOTAL_DISTANCE               (1 << 2)
#define FTMS_ROWER_INSTANTANEOUS_PACE           (1 << 3)
#define FTMS_ROWER_AVERAGE_PACE                 (1 << 4)
#define FTMS_ROWER_INSTANTANEOUS_POWER          (1 << 5)
#define FTMS_ROWER_AVERAGE_POWER                (1 << 6)
#define FTMS_ROWER_RESISTANCE_LEVEL             (1 << 7)

#define FTMS_ROWER_EXPENDED_ENERGY              (1 << 8)
#define FTMS_ROWER_HEART_RATE                   (1 << 9)
#define FTMS_ROWER_METABOLIC_EQUIVALENT         (1 << 10)
#define FTMS_ROWER_ELAPSED_TIME                 (1 << 11)
#define FTMS_ROWER_REMAINING_TIME               (1 << 12)

void notify_ftms_rower_data(void* p)
{
    esp_gatt_if_t gatts_if = ((intptr_t)p) >> 16;
    uint16_t conn_id = ((intptr_t)p) & 0xffff;

    esp_err_t ret;

    for (;;)
    {
        waterrower_values_t values;
        ret = waterrower_get_values(waterrower_handle, &values);
        if (ret != ESP_OK)
        {
            break;
        }

        uint8_t stroke_rate = values.stroke_rate_x2;        
        uint16_t stroke_count = values.stroke_count;
        uint8_t stroke_average = values.stroke_average;
        uint32_t distance = values.distance;
        uint16_t pace = values.current_speed != 0 ? 500 * 100 / values.current_speed : 0;
        uint16_t power = values.power;
        uint16_t calories = values.calories;
        uint16_t cal_hour = 0xffff;
        uint8_t cal_min = 0xff;
        uint8_t heart_rate = values.heart_rate;
        uint16_t elapsed = (uint16_t)(int)values.timer;

        if (heart_rate == 0)
        {
            antplus_get_heart_rate(antplus_handle, &heart_rate);
        }

        uint8_t rower_data_value_value[20];

        rower_data_value_value[0] = FTMS_ROWER_TOTAL_DISTANCE | FTMS_ROWER_INSTANTANEOUS_PACE | FTMS_ROWER_INSTANTANEOUS_POWER;
        rower_data_value_value[1] = (FTMS_ROWER_EXPENDED_ENERGY | FTMS_ROWER_HEART_RATE | FTMS_ROWER_ELAPSED_TIME) >> 8;
        rower_data_value_value[2] = stroke_rate & 0xff;
        rower_data_value_value[3] = stroke_count & 0xff;
        rower_data_value_value[4] = (stroke_count >> 8) & 0xff;
        rower_data_value_value[5] = distance & 0xff;
        rower_data_value_value[6] = (distance >> 8) & 0xff;
        rower_data_value_value[7] = (distance >> 16) & 0xff;
        rower_data_value_value[8] = pace & 0xff;
        rower_data_value_value[9] = (pace >> 8) & 0xff;
        rower_data_value_value[10] = power & 0xff;
        rower_data_value_value[11] = (power >> 8) & 0xff;
        rower_data_value_value[12] = calories & 0xff;
        rower_data_value_value[13] = (calories >> 8) & 0xff;
        rower_data_value_value[14] = cal_hour & 0xff;
        rower_data_value_value[15] = (cal_hour >> 8) & 0xff;
        rower_data_value_value[16] = cal_min & 0xff;
        rower_data_value_value[17] = heart_rate & 0xff;
        rower_data_value_value[18] = elapsed & 0xff;
        rower_data_value_value[19] = (elapsed >> 8) & 0xff;

        ESP_LOGI(TAG, "ble rower ftms: { timer: %u, stroke_rate: %.1f, distance: %u, strokes: %u, cal: %u, power: %u stroke_average: %u }", 
            elapsed, stroke_rate / 2.0, distance, values.stroke_count, calories, power, stroke_average);

        ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, ftms_handle_table[IDX_FTMS_ROWER_DATA_VAL], sizeof(rower_data_value_value), rower_data_value_value, false);
        if (ret != ESP_OK)
        {
            break;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "ble ftms rower data notify error (%d)", ret);
    vTaskDelete(NULL);
}

#define FTMS_INDOOR_BIKE_AVERAGE_SPEED          (1 << 1)
#define FTMS_INDOOR_BIKE_INSTANTANEOUS_CADENCE  (1 << 2)
#define FTMS_INDOOR_BIKE_AVERAGE_CADENCE        (1 << 3)
#define FTMS_INDOOR_BIKE_TOTAL_DISTANCE         (1 << 4)
#define FTMS_INDOOR_BIKE_RESISTANCE_LEVEL       (1 << 5)
#define FTMS_INDOOR_BIKE_INSTANTANEOUS_POWER    (1 << 6)
#define FTMS_INDOOR_BIKE_AVERAGE_POWER          (1 << 7)

#define FTMS_INDOOR_BIKE_EXPENDED_ENERGY        (1 << 8)
#define FTMS_INDOOR_BIKE_HEART_RATE             (1 << 9)
#define FTMS_INDOOR_BIKE_METABOLIC_EQUIVALENT   (1 << 10)
#define FTMS_INDOOR_BIKE_ELAPSED_TIME           (1 << 11)
#define FTMS_INDOOR_BIKE_REMAINING_TIME         (1 << 12)

void notify_ftms_indoor_bike_data(void* p)
{
    esp_gatt_if_t gatts_if = ((intptr_t)p) >> 16;
    uint16_t conn_id = ((intptr_t)p) & 0xffff;

    esp_err_t ret;

    for (;;)
    {
        waterrower_values_t values;
        ret = waterrower_get_values(waterrower_handle, &values);
        if (ret != ESP_OK)
        {
            break;
        }

        uint16_t speed = (uint16_t)(values.current_speed * 60 * 60 / 1000);
        ets_printf("cm/s =%u km/h=%.1f\n", values.current_speed, speed / 100.0f);
        uint8_t cadence = values.stroke_rate_x2;
        uint32_t distance = values.distance;
        uint16_t power = values.power;
        uint16_t calories = values.calories;
        uint16_t cal_hour = 0xffff;
        uint8_t cal_min = 0xff;
        uint8_t heart_rate = values.heart_rate;
        uint16_t elapsed = (uint16_t)(int)values.timer;

        uint8_t indoor_bike_data_value[19];

        indoor_bike_data_value[0] = FTMS_INDOOR_BIKE_TOTAL_DISTANCE | FTMS_INDOOR_BIKE_INSTANTANEOUS_CADENCE | FTMS_INDOOR_BIKE_INSTANTANEOUS_POWER;
        indoor_bike_data_value[1] = (FTMS_INDOOR_BIKE_EXPENDED_ENERGY | FTMS_INDOOR_BIKE_HEART_RATE | FTMS_INDOOR_BIKE_ELAPSED_TIME) >> 8;
        indoor_bike_data_value[2] = speed & 0xff;
        indoor_bike_data_value[3] = (speed >> 8) & 0xff;
        indoor_bike_data_value[4] = cadence & 0xff;
        indoor_bike_data_value[5] = (cadence >> 8) & 0xff;
        indoor_bike_data_value[6] = distance & 0xff;
        indoor_bike_data_value[7] = (distance >> 8) & 0xff;
        indoor_bike_data_value[8] = (distance >> 16) & 0xff;
        indoor_bike_data_value[9] = power & 0xff;
        indoor_bike_data_value[10] = (power >> 8) & 0xff;
        indoor_bike_data_value[11] = calories & 0xff;
        indoor_bike_data_value[12] = (calories >> 8) & 0xff;
        indoor_bike_data_value[13] = cal_hour & 0xff;
        indoor_bike_data_value[14] = (cal_hour >> 8) & 0xff;
        indoor_bike_data_value[15] = cal_min & 0xff;
        indoor_bike_data_value[16] = heart_rate & 0xff;
        indoor_bike_data_value[17] = elapsed & 0xff;
        indoor_bike_data_value[18] = (elapsed >> 8) & 0xff;

        ESP_LOGI(TAG, "ble indoor bike ftms: { timer: %u, distance: %u, cadence: %u }", elapsed, distance, cadence);

        ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, ftms_handle_table[IDX_FTMS_INDOOR_BIKE_DATA_VAL], sizeof(indoor_bike_data_value), indoor_bike_data_value, false);
        if (ret != ESP_OK)
        {
            break;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "ble ftms rower data notify error (%d)", ret);
    vTaskDelete(NULL);
}

void notify_csc_measurement(void* p)
{
    esp_err_t ret;

    esp_gatt_if_t gatts_if = ((intptr_t)p) >> 16;
    uint16_t conn_id = ((intptr_t)p) & 0xffff;

    uint16_t last_crank_revs = 0;
    uint16_t first_stroke_count;
    int64_t first_ts;

    for (int i = 0;; i++)
    {
        waterrower_values_t values;
        ret = waterrower_get_values(waterrower_handle, &values);
        if (ret != ESP_OK)
        {
            break;
        }

        int64_t now = esp_timer_get_time() / 1000 * 1024 / 1000;

        if (i == 0)
        {
            first_stroke_count = values.stroke_start_count;
            first_ts = values.last_stroke_start_ts;
        }

        uint16_t crank_revs = values.stroke_start_count - first_stroke_count;
        uint16_t ts = (values.last_stroke_start_ts - first_ts) / 1000 * 1024 / 1000;

        if (crank_revs != last_crank_revs)
        {
            uint8_t csc_measurement_value[5];

            csc_measurement_value[0] = 0x02;
            csc_measurement_value[1] = crank_revs & 0xff;
            csc_measurement_value[2] = (crank_revs >> 8) & 0xff;
            csc_measurement_value[3] = ts & 0xff;
            csc_measurement_value[4] = (ts >> 8) & 0xff;

            ESP_LOGI(TAG, "ble csc cranks %u ts %u", crank_revs, ts);

            ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, csc_handle_table[IDX_CSC_MEASUREMENT_VAL], sizeof(csc_measurement_value), csc_measurement_value, false);
            if (ret != ESP_OK)
            {
                break;
            }
            last_crank_revs = crank_revs;
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "CSC notify error (%d)", ret);
    vTaskDelete(NULL);
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
                    uint16_t descr_value = (param->write.value[1] << 8) | param->write.value[0];
                    if ((descr_value & 1) == 1)
                    {
                        create_notify_task(gatts_if, param->write.conn_id, "notify_csc_measurement", notify_csc_measurement);
                    }
                    else
                    {
                        kill_notify_task(gatts_if, param->write.conn_id, "notify_csc_measurement");
                    }
                }
                else if (hr_handle_table[IDX_HR_MEASUREMENT_CFG] == param->write.handle && param->write.len == 2)
                {
                    uint16_t descr_value = (param->write.value[1] << 8) | param->write.value[0];
                    if ((descr_value & 1) == 1)
                    {
                        create_notify_task(gatts_if, param->write.conn_id, "notify_hr_measurement", notify_hr_measurement);
                    }
                    else
                    {
                        kill_notify_task(gatts_if, param->write.conn_id, "notify_hr_measurement");
                    }
                }
                else if (ftms_handle_table[IDX_FTMS_ROWER_DATA_CFG] == param->write.handle && param->write.len == 2)
                {
                    uint16_t descr_value = (param->write.value[1] << 8) | param->write.value[0];
                    if ((descr_value & 1) == 1)
                    {
                        create_notify_task(gatts_if, param->write.conn_id, "notify_rower_data", notify_ftms_rower_data);
                    }
                    else
                    {
                        kill_notify_task(gatts_if, param->write.conn_id, "notify_rower_data");
                    }
                }
                else if (ftms_handle_table[IDX_FTMS_INDOOR_BIKE_DATA_CFG] == param->write.handle && param->write.len == 2)
                {
                    uint16_t descr_value = (param->write.value[1] << 8) | param->write.value[0];
                    if ((descr_value & 1) == 1)
                    {
                        create_notify_task(gatts_if, param->write.conn_id, "notify_indoor_bike_data", notify_ftms_indoor_bike_data);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "kill indoor bike data notify %u %u", gatts_if, param->write.conn_id);
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "write handle %x len %d", param->write.handle, param->write.len);
                }
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
            //ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
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
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, conn_id = %u reason = 0x%x", param->disconnect.conn_id, param->disconnect.reason);
            kill_notify_tasks(gatts_if, param->disconnect.conn_id);
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

    ESP_ERROR_CHECK(antplus_init(&antplus_handle));
    ESP_ERROR_CHECK(usb_init());
    ESP_ERROR_CHECK(waterrower_init(&waterrower_handle));

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
