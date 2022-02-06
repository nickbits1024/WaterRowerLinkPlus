
#define STATE_LED_GPIO     GPIO_NUM_48
#define STATE_LED_CHANNEL  0

typedef struct
{
    uint8_t state;
    led_strip_t* led_strip;
    portMUX_TYPE mux;
}
state_manager_driver_t;