#define BLE_NOTIFY_TASKS_MAX        10

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define WATERROWER_APP_ID           0x55
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

//waterrower_handle_t waterrower_handle;

typedef struct
{
    uint8_t* prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;


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

typedef struct
{
    ble_driver_t* driver;
    uint16_t gatts_if;
    uint16_t conn_id;
}
notify_task_context_t;

typedef struct
{
    notify_task_context_t ctx;
    const char* task_name;
    TaskHandle_t task_handle;    
}
notify_task_t;