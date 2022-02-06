#ifndef SM_H
#define SM_H

#define STATE_MANAGER_COMPONENT_S4    1
#define STATE_MANAGER_COMPONENT_HR    2
#define STATE_MANAGER_COMPONENT_ANT   4

typedef void* state_manager_handle_t;

esp_err_t state_manager_init(state_manager_handle_t* sm_handle);
esp_err_t state_manager_set_component_state(state_manager_handle_t* sm_handle, uint8_t component, bool state);

#endif