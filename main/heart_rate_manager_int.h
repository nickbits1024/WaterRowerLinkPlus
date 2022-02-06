#define HRM_TIMEOUT         10

typedef struct
{
    uint8_t heart_rate;
    int64_t heart_rate_ts;
}
hrm_source_t;

typedef struct
{
    hrm_source_t sources[HRM_NUM_SOURCES];
    state_manager_handle_t sm_handle;
    portMUX_TYPE mux;
}
heart_rate_driver_t;
