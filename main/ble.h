
enum
{
    IDX_CSC_SVC,
    IDX_CSC_MEASUREMENT,
    IDX_CSC_MEASUREMENT_VAL,
    IDX_CSC_MEASUREMENT_CFG,
    IDX_CSC_FEATURE,
    IDX_CSC_FEATURE_VAL,
    IDX_SENSOR_LOCATION,
    IDX_SENSOR_LOCATION_VAL,
    IDX_CSC_SC_CONTROL_POINT,
    IDX_CSC_SC_CONTROL_POINT_VAL,
    CSC_IDX_NB
};

enum
{
    IDX_FTMS_SVC,
    IDX_FTMS_FEATURE,
    IDX_FTMS_FEATURE_VAL,
    IDX_FTMS_ROWER_DATA,
    IDX_FTMS_ROWER_DATA_VAL,
    IDX_FTMS_ROWER_DATA_CFG,
    IDX_FTMS_INDOOR_BIKE_DATA,
    IDX_FTMS_INDOOR_BIKE_DATA_VAL,
    IDX_FTMS_INDOOR_BIKE_DATA_CFG,
    IDX_FTMS_CONTROL_POINT,
    IDX_FTMS_CONTROL_POINT_VAL,
    FTMS_IDX_NB
};

enum
{
    IDX_HR_SVC,
    IDX_HR_MEASUREMENT,
    IDX_HR_MEASUREMENT_VAL,
    IDX_HR_MEASUREMENT_CFG,
    IDX_BODY_SENSOR_LOCATION,
    IDX_BODY_SENSOR_LOCATION_VAL,
    IDX_HR_CONTROL_POINT,
    IDX_HR_CONTROL_POINT_VAL,
    HR_IDX_NB
};

#define FTMS_ROWER_AVERAGE_STROKE               (1 << 1)
#define FTMS_ROWER_TOTAL_DISTANCE               (1 << 2)
#define FTMS_ROWER_INSTANTANEOUS_PACE           (1 << 3)
#define FTMS_ROWER_AVERAGE_PACE                 (1 << 4)
#define FTMS_ROWER_INSTANTANEOUS_POWER          (1 << 5)
#define FTMS_ROWER_AVERAGE_POWER                (1 << 6)
#define FTMS_ROWER_RESISTANCE_LEVEL             (1 << 7)

#define FTMS_ROWER_EXPENDED_ENERGY              (1 << 8)
#define FTMS_ROWER_HEART_RATE                   (1 << 9)
#define FTMS_ROWER_METABOLIC_EQUIVALENT         (1 << 10)
#define FTMS_ROWER_ELAPSED_TIME                 (1 << 11)
#define FTMS_ROWER_REMAINING_TIME               (1 << 12)

#define FTMS_INDOOR_BIKE_AVERAGE_SPEED          (1 << 1)
#define FTMS_INDOOR_BIKE_INSTANTANEOUS_CADENCE  (1 << 2)
#define FTMS_INDOOR_BIKE_AVERAGE_CADENCE        (1 << 3)
#define FTMS_INDOOR_BIKE_TOTAL_DISTANCE         (1 << 4)
#define FTMS_INDOOR_BIKE_RESISTANCE_LEVEL       (1 << 5)
#define FTMS_INDOOR_BIKE_INSTANTANEOUS_POWER    (1 << 6)
#define FTMS_INDOOR_BIKE_AVERAGE_POWER          (1 << 7)

#define FTMS_INDOOR_BIKE_EXPENDED_ENERGY        (1 << 8)
#define FTMS_INDOOR_BIKE_HEART_RATE             (1 << 9)
#define FTMS_INDOOR_BIKE_METABOLIC_EQUIVALENT   (1 << 10)
#define FTMS_INDOOR_BIKE_ELAPSED_TIME           (1 << 11)
#define FTMS_INDOOR_BIKE_REMAINING_TIME         (1 << 12)

typedef struct
{
    hrm_handle_t hrm_handle;
    s4_handle_t s4_handle;
}
ble_driver_t;

typedef void* ble_handle_t;

esp_err_t ble_init(hrm_handle_t hrm_handle, s4_handle_t s4_handle, ble_handle_t* ble_handle);