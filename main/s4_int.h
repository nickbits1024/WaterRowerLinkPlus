#define S4_INACTIVTY_TIMEOUT        (60 * 30)

#define S4_MAX_PACKET_SIZE          64
#define S4_USB_ENDPOINT_OUT_ADDRESS 0x03
#define S4_USB_ENDPOINT_IN_ADDRESS  0x83
#define S4_USB_INTERFACE_NUMBER     0x01

#define S4_MAX_COMMAND_SIZE         62

#define S4_SINGLE_VALUE             'S'
#define S4_DOUBLE_VALUE             'D'
#define S4_TRIPLE_VALUE             'T'

#define S4_COMMAND_USB              "USB"
#define S4_COMMAND_USB_RESPONSE     "_WR_"
#define S4_COMMAND_RESET            "RESET"

#define S4_USB_POWER_GPIO_NUM        GPIO_NUM_8
#define S4_USB_POWER_GPIO_SEL        GPIO_SEL_8
#define S4_POWER_BUTTON_GPIO_NUM     GPIO_NUM_10
#define S4_POWER_BUTTON_GPIO_SEL     GPIO_SEL_10

#define S4_HEART_BEAT_TX_GPIO_NUM  GPIO_NUM_14
#define S4_HEART_BEAT_TX_GPIO_SEL  GPIO_SEL_14


typedef struct
{
    uint16_t address;
    uint8_t size;
    const char* name;
}
s4_variable_t;

// these must be in address order!
enum
{
    S4_DISTANCE,
    //S4_POWER,
    S4_CALORIES,
    S4_STROKE_COUNT,
    S4_STROKE_AVERAGE,
    S4_CURRENT_SPEED,
    S4_HEART_RATE,
    S4_500_PACE,
    S4_TIMER_SECOND_DEC,
    S4_TIMER_SECOND,
    S4_TIMER_MINUTE,
    S4_TIMER_HOUR,
};

#define S4_MEMORY_MAP_SIZE  (sizeof(memory_map) / sizeof(s4_variable_t))

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;

    SemaphoreHandle_t device_sem;

    usb_transfer_t* in_transfer;
    char in_buffer[S4_MAX_COMMAND_SIZE + 1];
    uint8_t in_buffer_size;

    usb_transfer_t* out_transfer;
    SemaphoreHandle_t out_sem;
    SemaphoreHandle_t out_resp_sem;

    SemaphoreHandle_t cmd_sem;
    char wait_buffer[S4_MAX_COMMAND_SIZE + 1];
    uint8_t wait_buffer_size;

    portMUX_TYPE values_mux;
    s4_values_t values;

    TaskHandle_t out_transfer_task_handle;

    hrm_handle_t hrm_handle;
} s4_driver_t;
