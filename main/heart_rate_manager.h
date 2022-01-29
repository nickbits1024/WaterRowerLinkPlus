#ifndef HEART_RATE_MANAGER_H
#define HEART_RATE_MANAGER_H

enum
{
    HRM_SOURCE_ANT,
    HRM_SOURCE_BLE,
    HRM_SOURCE_S4,
    HRM_SOURCE_MAX
};

typedef void* hrm_handle_t;

esp_err_t hrm_init(hrm_handle_t* hrm_handle);
esp_err_t hrm_set_rate(hrm_handle_t hrm_handle, int source, uint8_t heart_rate);
esp_err_t hrm_get_rate(hrm_handle_t hrm_handle, uint8_t* heart_rate);

#endif