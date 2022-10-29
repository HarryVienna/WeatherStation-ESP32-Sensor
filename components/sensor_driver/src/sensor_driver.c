#include "sensor_driver.h"

esp_err_t sensor_driver_init_sensor(sensor_driver_t *handle){
    return handle->init_sensor(handle);
}
esp_err_t sensor_driver_read_values(sensor_driver_t *handle, sensor_data_t *values){
    return handle->read_values(handle, values);
}

