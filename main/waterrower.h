
typedef struct
{
    uint8_t stroke_rate;
    uint16_t stroke_count;
    uint16_t distance;
    uint8_t distance_dec;
    double count_down_timer;
    double timer;
    uint16_t watts;
    uint32_t calories;
    uint16_t meters_per_second;
    uint8_t heart_rate;
} waterrower_values_t;

esp_err_t waterrower_init();