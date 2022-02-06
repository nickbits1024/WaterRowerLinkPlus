#define ANTPLUS_INPUT                       0
#define ANTPLUS_OUTPUT                      1
#define ANTPLUS_ANY                         0
#define ANTPLUS_INFINITE_TIMEOUT            255

#define ANTPLUS_MESSAGE_REQUIRED_SIZE(msg)  (sizeof(antplus_message_t) + (msg)->size + 1)
#define ANTPLUS_MESSAGE_SIZE(t)             (sizeof(t) - sizeof(antplus_message_t) - 1)
#define ANTPLUS_MESSAGE(t, id, dir)         { .sync_value = ANTPLUS_SYNC_VALUE, .direction = dir, .size = ANTPLUS_MESSAGE_SIZE(t), .msg_id = id }


#define ANTPLUS_EVENT                       0x01
#define ANTPLUS_BROADCAST                   0x4e
#define ANTPLUS_ACKNOWEDGED_DATA            0x4f
#define ANTPLUS_STARTUP                     0x6f
#define ANTPLUS_REQUEST_MESSAGE             0x4d
#define ANTPLUS_CAPABILITIES                0x54
#define ANTPLUS_VERSION                     0x3e
#define ANTPLUS_CHANNEL_ID                  0x51
#define ANTPLUS_ASSIGN_CHANNEL              0x42
#define ANTPLUS_SET_NETWORK_KEY             0x46
#define ANTPLUS_SET_CHANNEL_RF_FREQ         0x45
#define ANTPLUS_SET_CHANNEL_SEARCH_TIMEOUT  0x44
#define ANTPLUS_SET_CHANNEL_PERIOD          0x43
#define ANTPLUS_SET_CHANNEL_RF_FREQ         0x45
#define ANTPLUS_SET_CHANNEL_TX_POWER        0x60
#define ANTPLUS_OPEN_CHANNEL                0x4b
#define ANTPLUS_CHANNEL_RESPONSE            0x40

#define ANTPLUS_FE_DEVICE_NUM               62282

#define ANTPLUS_UART_BAUD_RATE              115200
#define ANTPLUS_UART_NUM                    UART_NUM_1
#define ANTPLUS_UART_QUEUE_SIZE             10
#define ANTPLUS_MAX_MESSAGE_SIZE            41
#define ANTPLUS_UART_RX_BUFFER_SIZE         1024
#define ANTPLUS_UART_TX_BUFFER_SIZE         0

#define ANTPLUS_CTS_GPIO_NUM                GPIO_NUM_16
#define ANTPLUS_TX_GPIO_NUM                 GPIO_NUM_17
#define ANTPLUS_RX_GPIO_NUM                 GPIO_NUM_18

#define ANTPLUS_RESET_GPIO_NUM              GPIO_NUM_15
#define ANTPLUS_RESET_GPIO_SEL              GPIO_SEL_15

#define ANTPLUS_SYNC_BYTE_MASK              0xfe
#define ANTPLUS_SYNC_BYTE_VALUE             0xa4
#define ANTPLUS_SYNC_VALUE                  0x52

#define ANTPLUS_PAGE_MASK                   0x7f

#define ANTPLUS_NETWORK_KEY                 { 0xb9, 0xa5, 0x21, 0xfb, 0xbd, 0x72, 0xc3, 0x45 }
#define ANTPLUS_NETWORK_KEY_SIZE            8
#define ANTPLUS_BROADCAST_PAYLOAD_SIZE      8

#define ANTPLUS_PUBLIC_NETWORK              0

#define ANTPLUS_FE                          17
#define ANTPLUS_HRM                         120

#define ANTPLUS_CHANNEL_HRM                 0
#define ANTPLUS_CHANNEL_FE                  1

#define ANTPLUS_0DBM                        3
#define ANTPLUS_2457MHZ                     57

#define ANTPLUS_BIDIRECTIONAL_RECEIVE       0
#define ANTPLUS_BIDIRECTIONAL_TRANSMIT      0x10
#define ANTPLUS_IC                          0x01
#define ANTPLUS_GDP                         0x04


#define ANTPLUS_PAGE_MANUFACTURER_ID        80
#define ANTPLUS_PAGE_PRODUCT_INFO           81
#define ANTPLUS_PAGE_ROWING_DATA            22
#define ANTPLUS_PAGE_GENERAL_FE             16
#define ANTPLUS_ROWER                       22
#define ANTPLUS_CAPS_HR_FE                  0x03
#define ANTPLUS_CAPS_HR_EM                  0x02
#define ANTPLUS_CAPS_HR_ANT                 0x01
#define ANTPLUS_CAPS_DISTANCE               0x04
#define ANTPLUS_CAPS_VIRTUAL_SPEED          0x08
#define ANTPLUS_CAPS_STROKES                0x01

#define ANTPLUS_EVENT_READY                 0x01

typedef struct
{
    uint8_t uart_num;
    QueueHandle_t uart_queue_handle;
    TaskHandle_t recv_task_handle;
    state_manager_handle_t sm_handle;    
    hrm_handle_t hrm_handle;
    s4_handle_t s4_handle;
    EventGroupHandle_t eg_handle;
    portMUX_TYPE mux;
    uint16_t heart_beat_ts;
}
antplus_driver_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    uint8_t data[];
}
__attribute__((packed)) antplus_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t payload[ANTPLUS_BROADCAST_PAYLOAD_SIZE];
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_broadcast_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t page_num : 7;
        uint8_t page_change_toggle : 1;
        union
        {
            struct 
            {
                uint8_t reserved[3];
                uint16_t heart_beat_ts;
                uint8_t heart_beat_count;
                uint8_t hr;
            }
            __attribute__((packed)) common;
        }
        __attribute__((packed));
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_heartrate_broadcast_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t page_num;
        uint8_t reserved0;
        uint8_t reserved1;
        uint8_t hw_revision;
        uint16_t manufacturer_id;
        uint16_t model_no;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_manufacturer_id_page_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t page_num;
        uint8_t reserved;
        uint8_t sw_rev_supp;
        uint8_t sw_rev_main;
        uint32_t serial_no;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_product_info_page_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t page_num;
        uint8_t reserved0;
        uint8_t reserved1;
        uint8_t stroke_count;
        uint8_t cadence;
        uint16_t power;
        uint8_t flags;        
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_rowing_data_page_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t page_num;
        uint8_t equipment_type;
        uint8_t elasped;
        uint8_t distance;
        uint16_t speed;
        uint8_t heart_rate;
        uint8_t flags;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_general_fe_page_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t msg_id;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_request_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t msg_id;
        uint8_t error;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_channel_reponse_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        char version[1];
    }
    __attribute__((packed)) data;
}
__attribute__((packed)) antplus_version_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t network;
        uint8_t key[ANTPLUS_NETWORK_KEY_SIZE];
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_set_network_key_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t channel_type;
        uint8_t network;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_assign_channel_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint16_t device_num;
        uint8_t device_type;
        uint8_t tx_type;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_channel_id_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t rf_freq;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_set_channel_rf_freq_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint16_t period;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_set_channel_period_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t tx_power;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_set_channel_tx_power_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
        uint8_t time_out;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_set_channel_search_timeout_message_t;


typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t channel;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_open_channel_message_t;

typedef struct
{
    uint8_t direction : 1;
    uint8_t sync_value : 7;
    uint8_t size;
    uint8_t msg_id;
    struct
    {
        uint8_t max_channels;
        uint8_t max_networks;
        uint8_t standard_options;
        uint8_t advanced_options;
        uint8_t advanced_options_2;
        uint8_t max_sensrcore_channels;
        uint8_t advanced_options_3;
        uint8_t advanced_options_4;
    }
    __attribute__((packed)) data;
    uint8_t checksum;
}
__attribute__((packed)) antplus_capabilities_t;

esp_err_t antplus_request_message(antplus_driver_t* driver, uint8_t channel, uint8_t msg_id);
esp_err_t antplus_assign_channel(antplus_driver_t* driver, uint8_t channel, uint8_t channel_type, uint8_t network);
esp_err_t antplus_set_channel_id(antplus_driver_t* driver, uint8_t channel, uint16_t device, uint8_t device_type, uint8_t tx_type);
esp_err_t antplus_set_channel_rf_freq(antplus_driver_t* driver, uint8_t channel, uint8_t freq);
esp_err_t antplus_set_channel_period(antplus_driver_t* driver, uint8_t channel, uint16_t period);
esp_err_t antplus_set_channel_tx_power(antplus_driver_t* driver, uint8_t channel, uint8_t tx_power);
esp_err_t antplus_set_channel_search_timeout(antplus_driver_t* driver, uint8_t channel, uint8_t time_out);
esp_err_t antplus_open_channel(antplus_driver_t* driver, uint8_t channel);
esp_err_t antplus_setup(antplus_driver_t* antplus_handle);
