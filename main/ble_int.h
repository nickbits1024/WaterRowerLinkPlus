#define BLE_NOTIFY_TASKS_MAX        10

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define WATERROWER_SERVER_APP_ID    0
#define WATERROWER_CLIENT_APP_ID    1
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


enum
{
    IDX_CSC_SVC,
    IDX_CSC_MEASUREMENT,
    IDX_CSC_MEASUREMENT_VAL,
    IDX_CSC_MEASUREMENT_CFG,
    IDX_CSC_FEATURE,
    IDX_CSC_FEATURE_VAL,
    IDX_SENSOR_LOCATION,
    IDX_SENSOR_LOCATION_VAL,
    IDX_CSC_SC_CONTROL_POINT,
    IDX_CSC_SC_CONTROL_POINT_VAL,
    CSC_IDX_NB
};

enum
{
    IDX_FTMS_SVC,
    IDX_FTMS_FEATURE,
    IDX_FTMS_FEATURE_VAL,
    IDX_FTMS_ROWER_DATA,
    IDX_FTMS_ROWER_DATA_VAL,
    IDX_FTMS_ROWER_DATA_CFG,
    IDX_FTMS_INDOOR_BIKE_DATA,
    IDX_FTMS_INDOOR_BIKE_DATA_VAL,
    IDX_FTMS_INDOOR_BIKE_DATA_CFG,
    IDX_FTMS_CONTROL_POINT,
    IDX_FTMS_CONTROL_POINT_VAL,
    FTMS_IDX_NB
};

enum
{
    IDX_HR_SVC,
    IDX_HR_MEASUREMENT,
    IDX_HR_MEASUREMENT_VAL,
    IDX_HR_MEASUREMENT_CFG,
    IDX_BODY_SENSOR_LOCATION,
    IDX_BODY_SENSOR_LOCATION_VAL,
    IDX_HR_CONTROL_POINT,
    IDX_HR_CONTROL_POINT_VAL,
    HR_IDX_NB
};

#define FTMS_FEATURE_AVERAGE_SPEED_SUPPORTED                    (1 << 0)
#define FTMS_FEATURE_CADENCE_SUPPORTED                          (1 << 1)
#define FTMS_FEATURE_TOTAL_DISTANCE_SUPPORTED                   (1 << 2)
#define FTMS_FEATURE_INCLINATION_SUPPORTED                      (1 << 3)
#define FTMS_FEATURE_ELEVATION_GAIN_SUPPORTED                   (1 << 4)
#define FTMS_FEATURE_PACE_SUPPORTED                             (1 << 5)
#define FTMS_FEATURE_STEP_COUNT_SUPPORTED                       (1 << 6)
#define FTMS_FEATURE_RESISTANCE_LEVEL_SUPPORTED                 (1 << 7)

#define FTMS_FEATURE_STRIDE_COUNT_SUPPORTED                     (1 << 8)
#define FTMS_FEATURE_EXPENDED_ENERGY_SUPPORTED                  (1 << 9)
#define FTMS_FEATURE_HEART_RATE_MEASUREMENT_SUPPORTED           (1 << 10)
#define FTMS_FEATURE_METABOLIC_EQUIVALENT_SUPPORTED             (1 << 11)
#define FTMS_FEATURE_ELAPSED_TIME_SUPPORTED                     (1 << 12)
#define FTMS_FEATURE_REMAINING_TIME_SUPPORTED                   (1 << 13)
#define FTMS_FEATURE_POWER_MEASUREMENT_SUPPORTED                (1 << 14)
#define FTMS_FEATURE_FORCE_ON_BELT_AND_POWER_OUTPUT_SUPPORTED   (1 << 15)
#define FTMS_FEATURE_USER_DATA_RETENTION_SUPPORTED              (1 << 16)

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

#define HEARTRATE_MEASUREMENT_FORMAT_MASK       1
#define HEARTRATE_MEASUREMENT_FORMAT_8BIT       0
#define HEARTRATE_MEASUREMENT_FORMAT_16BIT      1

#define HEARTRATE_MEASUREMENT_SENSOR_CONTACT_MASK       6
#define HEARTRATE_MEASUREMENT_SENSOR_CONTACT_NONE       4
#define HEARTRATE_MEASUREMENT_SENSOR_CONTACT_DETECTED   6

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

typedef struct
{
    hrm_handle_t hrm_handle;
    s4_handle_t s4_handle;

    uint8_t adv_config_done;
    uint16_t csc_handle_table[CSC_IDX_NB];
    uint16_t ftms_handle_table[FTMS_IDX_NB];
    uint16_t hr_handle_table[HR_IDX_NB];

    uint16_t hr_gattc_if;
    uint16_t hr_char_handle;
    uint16_t hr_conn_id;
    esp_bd_addr_t hr_remote_bda;
    uint16_t hr_service_start_handle;
    uint16_t hr_service_end_handle;
    bool hr_server_found;
    bool hr_connected;
    esp_gattc_char_elem_t* hr_char_elem_result;
    esp_gattc_descr_elem_t* hr_descr_elem_result;
}
ble_driver_t;

typedef struct
{
    ble_driver_t* driver;
    uint16_t gatts_if;
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
}
notify_task_context_t;

typedef struct
{
    notify_task_context_t ctx;
    const char* task_name;
    TaskHandle_t task_handle;    
}
notify_task_t;