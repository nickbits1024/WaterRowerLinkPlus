#include <string.h>
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
#include "s4.h"
#include "ble.h"
#include "ble_int.h"

#define TAG "ble"

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

notify_task_t notify_tasks[BLE_NOTIFY_TASKS_MAX];
portMUX_TYPE notify_tasks_mux = portMUX_INITIALIZER_UNLOCKED;
ble_driver_t* gatts_profile_event_handler_driver;

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

void create_notify_task(ble_driver_t* driver, uint16_t gatts_if, uint16_t conn_id, const char* task_name, TaskFunction_t task_func)
{
    bool task_found = false;
    bool task_created = false;

    portENTER_CRITICAL(&notify_tasks_mux);
    for (int i = 0; i < BLE_NOTIFY_TASKS_MAX; i++)
    {
        if (notify_tasks[i].task_handle != NULL &&
            notify_tasks[i].ctx.gatts_if == gatts_if &&
            notify_tasks[i].ctx.conn_id == conn_id &&
            strcmp(notify_tasks[i].task_name, task_name) == 0)
        {
            task_found = true;
        }
    }
    if (!task_found)
    {
        for (int i = 0; i < BLE_NOTIFY_TASKS_MAX; i++)
        {
            if (notify_tasks[i].task_handle == NULL)
            {
                notify_tasks[i].ctx.driver = driver;
                notify_tasks[i].ctx.gatts_if = gatts_if;
                notify_tasks[i].ctx.conn_id = conn_id;
                notify_tasks[i].task_name = task_name;
                xTaskCreate(task_func, task_name, 4096, (void*)&notify_tasks[i].ctx, 23, &notify_tasks[i].task_handle);
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
    for (int i = 0; i < BLE_NOTIFY_TASKS_MAX; i++)
    {
        bool task_killed = false;
        portENTER_CRITICAL(&notify_tasks_mux);
        if (notify_tasks[i].task_handle != NULL &&
            notify_tasks[i].ctx.gatts_if == gatts_if &&
            notify_tasks[i].ctx.conn_id == conn_id &&
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
    for (int i = 0; i < BLE_NOTIFY_TASKS_MAX; i++)
    {
        bool task_killed = false;
        const char* task_name;
        portENTER_CRITICAL(&notify_tasks_mux);
        if (notify_tasks[i].ctx.gatts_if == gatts_if && notify_tasks[i].ctx.conn_id == conn_id)
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
    notify_task_context_t* ctx = (notify_task_context_t*)p;

    for (;;)
    {
        uint8_t heart_rate = 0;
        hrm_get_rate(ctx->driver->hrm_handle, &heart_rate);

        uint8_t hr_measurement_value[2];
        hr_measurement_value[0] = heart_rate >> 8;
        hr_measurement_value[1] = heart_rate & 0xff;

        ESP_LOGI(TAG, "ble hr %d", heart_rate);
        esp_err_t ret = esp_ble_gatts_send_indicate(ctx->gatts_if, ctx->conn_id, hr_handle_table[IDX_HR_MEASUREMENT_VAL], sizeof(hr_measurement_value), hr_measurement_value, false);
        if (ret != ESP_OK)
        {
            ESP_LOGI(TAG, "HR notify error (%d)", ret);
            vTaskDelete(NULL);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void notify_ftms_rower_data(void* p)
{
    notify_task_context_t* ctx = (notify_task_context_t*)p;

    esp_err_t ret;

    for (;;)
    {
        s4_values_t values;
        ret = s4_get_values(ctx->driver->s4_handle, &values);
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
        uint16_t elapsed = (uint16_t)(int)values.timer;

        uint8_t heart_rate;
        hrm_get_rate(ctx->driver->hrm_handle, &heart_rate);

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

        ret = esp_ble_gatts_send_indicate(ctx->gatts_if, ctx->conn_id, ftms_handle_table[IDX_FTMS_ROWER_DATA_VAL], sizeof(rower_data_value_value), rower_data_value_value, false);
        if (ret != ESP_OK)
        {
            break;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "ble ftms rower data notify error (%d)", ret);
    vTaskDelete(NULL);
}

void notify_ftms_indoor_bike_data(void* p)
{
    notify_task_context_t* ctx = (notify_task_context_t*)p;

    esp_err_t ret;

    for (;;)
    {
        s4_values_t values;
        ret = s4_get_values(ctx->driver->s4_handle, &values);
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
        uint16_t elapsed = (uint16_t)(int)values.timer;

        uint8_t heart_rate;
        hrm_get_rate(ctx->driver->hrm_handle, &heart_rate);

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

        ret = esp_ble_gatts_send_indicate(ctx->gatts_if, ctx->conn_id, ftms_handle_table[IDX_FTMS_INDOOR_BIKE_DATA_VAL], sizeof(indoor_bike_data_value), indoor_bike_data_value, false);
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
    notify_task_context_t* ctx = (notify_task_context_t*)p;

    esp_err_t ret;

    uint16_t last_crank_revs = 0;
    uint16_t first_stroke_count;
    int64_t first_ts;

    for (int i = 0;; i++)
    {
        s4_values_t values;
        ret = s4_get_values(ctx->driver->s4_handle, &values);
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

            ret = esp_ble_gatts_send_indicate(ctx->gatts_if, ctx->conn_id, csc_handle_table[IDX_CSC_MEASUREMENT_VAL], sizeof(csc_measurement_value), csc_measurement_value, false);
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
    ble_driver_t* driver = gatts_profile_event_handler_driver;

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
                        create_notify_task(driver, gatts_if, param->write.conn_id, "notify_csc_measurement", notify_csc_measurement);
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
                        create_notify_task(driver, gatts_if, param->write.conn_id, "notify_hr_measurement", notify_hr_measurement);
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
                        create_notify_task(driver, gatts_if, param->write.conn_id, "notify_rower_data", notify_ftms_rower_data);
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
                        create_notify_task(driver, gatts_if, param->write.conn_id, "notify_indoor_bike_data", notify_ftms_indoor_bike_data);
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
            break;
        // case ESP_GATTS_EXEC_WRITE_EVT:
        //     // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX. 
        //     ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        //     example_exec_write_event_env(&prepare_write_env, param);
        //     break;
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

esp_err_t ble_init(hrm_handle_t hrm_handle, s4_handle_t s4_handle, ble_handle_t* ble_handle)
{
    *ble_handle = NULL;

    static bool initialized;

    if (initialized)
    {
        return ESP_FAIL;
    }    

    ble_driver_t* driver = malloc(sizeof(ble_driver_t));
    memset(driver, 0, sizeof(ble_driver_t));

    driver->hrm_handle = hrm_handle;
    driver->s4_handle = s4_handle;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_profile_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(WATERROWER_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));

    *ble_handle = driver;
    gatts_profile_event_handler_driver = driver;

    initialized = true;

    return ESP_OK;
}