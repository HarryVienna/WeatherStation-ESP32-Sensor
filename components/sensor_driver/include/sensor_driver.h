#pragma once

#include "esp_err.h"



/**
* @brief Sensor Type
*
*/
typedef struct sensor_driver_s sensor_driver_t;



/**
* @brief Declare of Sensor Type
*
*/
struct sensor_driver_s {

    esp_err_t (*init_sensor)(sensor_driver_t *handle);
    esp_err_t (*read_values)(sensor_driver_t *handle);
};



// |================================================================================================ |
// |                                           Initialisation                                        |
// |================================================================================================ |
esp_err_t sensor_driver_init_sensor(sensor_driver_t *handle);

// |================================================================================================ |
// |                          Functions that control hardware pins                                   |
// |================================================================================================ |
esp_err_t sensor_driver_read_values(sensor_driver_t *handle);


