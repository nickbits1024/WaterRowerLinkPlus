#include "heart_rate_manager.h"
#include "s4.h"

typedef void* antplus_handle_t;

esp_err_t antplus_init(hrm_handle_t* hrm_handle, s4_handle_t s4_handle, antplus_handle_t* antplus_handle);
esp_err_t antplus_reset(antplus_handle_t antplus_handle);
//esp_err_t antplus_get_heart_rate(antplus_handle_t antplus_handle, uint8_t* heart_rate);
