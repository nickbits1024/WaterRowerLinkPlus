/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    // IDX_FTMS_INDOOR_BIKE_DATA,
    // IDX_FTMS_INDOOR_BIKE_DATA_VAL,
    // IDX_FTMS_INDOOR_BIKE_DATA_CFG,
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
