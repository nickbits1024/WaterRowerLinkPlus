#define HRM_TIMEOUT         (60 * 5)

typedef struct
{
    uint8_t heart_rate;
    int64_t heart_rate_ts;
}
hrm_source_t;

typedef struct
{
    hrm_source_t sources[HRM_SOURCE_MAX];
    portMUX_TYPE mux;
}
heart_rate_driver_t;
