#ifndef S4_H
#define S4_H

#include "heart_rate_manager.h"

typedef struct
{
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
} s4_values_t;

typedef void* s4_handle_t;

esp_err_t s4_preinit();
esp_err_t s4_init(hrm_handle_t hrm_handle, s4_handle_t* waterrower_handle);
esp_err_t s4_get_values(s4_handle_t waterrower_handle, s4_values_t* values);

#endif