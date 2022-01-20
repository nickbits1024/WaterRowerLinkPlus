
typedef struct
{
    uint8_t stroke_rate;
    uint8_t stroke_rate_x2;
    uint16_t stroke_count;
    uint16_t stroke_average;
    uint16_t stroke_start_count;
    int64_t last_stroke_start_ts;
    uint16_t distance;
    uint8_t distance_dec;
    double timer;
    uint16_t power;
    uint32_t calories;
    uint16_t current_speed;
    uint8_t heart_rate;
    uint8_t timer_second;
    uint8_t timer_second_dec;
    uint8_t timer_minute;
    uint8_t timer_hour;
} waterrower_values_t;

typedef void* waterrower_handle_t;

esp_err_t waterrower_init(waterrower_handle_t* waterrower_handle);
esp_err_t waterrower_get_values(waterrower_handle_t waterrower_handle, waterrower_values_t* values);