#include "state_manager.h"

#ifndef HEART_RATE_MANAGER_H
#define HEART_RATE_MANAGER_H

#define HRM_SOURCE_ANT      0
#define HRM_SOURCE_BLE      1
//#define HRM_SOURCE_S4       4
//#define HRM_SOURCE_ANY      0xff
#define HRM_NUM_SOURCES     2

typedef void* hrm_handle_t;

esp_err_t hrm_init(state_manager_handle_t sm_handle, hrm_handle_t* hrm_handle);
esp_err_t hrm_set_rate(hrm_handle_t hrm_handle, uint8_t source, uint8_t heart_rate);
esp_err_t hrm_get_rate(hrm_handle_t hrm_handle, uint8_t* heart_rate);

#endif