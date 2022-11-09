#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/cdefs.h>

#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "hdc1080_sensor_driver.h"


#define RAW2HUM(raw) (((float)raw) * 100/65536)
#define RAW2TEMP(raw) (((float)raw) * 165/65536 - 40)

static const char *TAG = "hdc1080";


void swap_bytes(uint16_t *pv)
{
    uint8_t *p = (uint8_t *)pv;
    uint8_t tmp=p[0];
    p[0] = p[1];
    p[1] = tmp;
}

float conv_temperature(uint16_t temperature)
{
    return (temperature / pow(2, 16)) * 165.0 - 40.0;
}

float conv_humidity(uint16_t humidity)
{
    return (humidity / pow(2, 16)) * 100;
}

esp_err_t get_temperature_humidity(uint8_t reg_addr, uint16_t *temperature, uint16_t *humidity, uint8_t i2c_addr, uint16_t delay)
{
    esp_err_t rslt = ESP_OK;

    uint16_t data[2];
    rslt |= i2c_master_write_to_device(I2C_NUM_0, i2c_addr, &reg_addr, sizeof(uint8_t), DEVICE_TICKS_TO_WAIT);
    
    ets_delay_us(delay);
    
    rslt |= i2c_master_read_from_device(I2C_NUM_0, i2c_addr, (uint8_t *)&data, sizeof(data), DEVICE_TICKS_TO_WAIT);

    swap_bytes(&data[0]);
    swap_bytes(&data[1]);

    *temperature = data[0];
    *humidity = data[1];

    return rslt;
}

esp_err_t get_register(uint8_t reg_addr, uint16_t *reg_data, uint8_t i2c_addr)
{
    esp_err_t rslt = ESP_OK;

    rslt = i2c_master_write_read_device(I2C_NUM_0, i2c_addr, &reg_addr, sizeof(uint8_t), (uint8_t *)reg_data, sizeof(uint16_t), DEVICE_TICKS_TO_WAIT);
    swap_bytes(reg_data);

    return rslt;
}

esp_err_t write_register(uint8_t reg_addr, uint16_t reg_data,  uint8_t i2c_addr)
{
    esp_err_t rslt = ESP_OK;

    swap_bytes(&reg_data);
    uint8_t write_buf[3] = {reg_addr, reg_data};
    rslt = i2c_master_write_to_device(I2C_NUM_0, i2c_addr, write_buf, sizeof(write_buf), 10);

    return rslt;
}



// |================================================================================================ |
// |                                           Initialisation                                        |
// |================================================================================================ |

/**
 * @brief Init Sensor
 *
 * @param handle handle to sensor_driver_t type object
 */
esp_err_t hdc1080_init_sensor(sensor_driver_t *handle)
{
    esp_err_t ret = ESP_OK;

    sensor_driver_hdc1080_t *hdc1080 = __containerof(handle, sensor_driver_hdc1080_t, parent);
    sensor_driver_hdc1080_conf_t hdc1080_config = hdc1080->driver_config;

    vTaskDelay(DEVICE_STARTUP_TIME_MS / portTICK_PERIOD_MS);

    uint16_t device_id = 0;
 
    uint8_t try_count = 5;
    while (try_count)
    {
        ret = get_register(REG_DEVICE_ID, &device_id, hdc1080_config.i2c_addr);

        /* Check for chip id validity */
        if ((ret == ESP_OK) && (device_id == HDC1080_DEVICE_ID))
        {
            hdc1080->hdc1080_device.device_id = device_id;

            uint16_t manufacturer_id = 0;
            get_register(REG_MANUFACTURER_ID, &manufacturer_id, hdc1080_config.i2c_addr);
            hdc1080->hdc1080_device.manufacturer_id = manufacturer_id;

            uint16_t serial_first = 0;
            uint16_t serial_mid = 0;
            uint16_t serial_last = 0;
            get_register(REG_SERIAL_ID_FIRST, &serial_first, hdc1080_config.i2c_addr);
            get_register(REG_SERIAL_ID_MID,   &serial_mid,   hdc1080_config.i2c_addr);
            get_register(REG_SERIAL_ID_LAST,  &serial_last,  hdc1080_config.i2c_addr);
            hdc1080_serial_t serial_id;
            serial_id.value = (uint64_t)serial_first<<32 | (uint64_t)serial_mid<<16 | (uint64_t)serial_last;
            hdc1080->hdc1080_device.serial_id = serial_id;

            ESP_LOGE(TAG, "Device ID: %x   Manufacturer ID: %x", device_id, manufacturer_id);
            ESP_LOGE(TAG, "year: %d", serial_id.year);
            ESP_LOGE(TAG, "day: %d", serial_id.day);
            ESP_LOGE(TAG, "month: %d", serial_id.month);

            break;
        }
        
        /* Wait for 10 ms */
        vTaskDelay(10 / portTICK_PERIOD_MS);
        --try_count;
    }

    /* Chip id check failed */
    if (!try_count)
    {
        ret = ESP_ERR_NOT_FOUND;
    }

    if (ret == ESP_OK) 
    {
        hdc1080_configuration_register_t config = {
            .software_reset = 0,
            .heater = 0,
            .acquisition_mode = 1,
            .humidity_resolution = hdc1080->driver_config.humidity_resolution,
            .temperature_resolution = hdc1080->driver_config.temperature_resolution,
        };

        write_register(REG_CONFIGURATION, config.value, hdc1080_config.i2c_addr);
    }

    return ret;
}

/**
 * @brief Read values from sensor
 *
 * @param motor handle to sensor_driver_t type object
 */
esp_err_t hdc1080_read_values(sensor_driver_t *handle, sensor_data_t *values)
{
    esp_err_t ret = ESP_OK;

    sensor_driver_hdc1080_t *hdc1080 = __containerof(handle, sensor_driver_hdc1080_t, parent);
    sensor_driver_hdc1080_conf_t hdc1080_config = hdc1080->driver_config;

    uint16_t temperature;
    uint16_t humidity;
    ret = get_temperature_humidity(REG_TEMPERATURE, &temperature, &humidity, hdc1080_config.i2c_addr, 14000);
   
    values->temperature = conv_temperature(temperature);
    values->humidity = conv_humidity(humidity);

    return ret;
}


sensor_driver_t *sensor_driver_new_hdc1080(const sensor_driver_hdc1080_conf_t *config)
{
    sensor_driver_hdc1080_t *hdc1080 = calloc(1, sizeof(sensor_driver_hdc1080_t));

    hdc1080->driver_config.humidity_resolution = config->humidity_resolution;
    hdc1080->driver_config.temperature_resolution = config->temperature_resolution;
    hdc1080->driver_config.i2c_addr = config->i2c_addr;

    hdc1080->parent.init_sensor = hdc1080_init_sensor;
    hdc1080->parent.read_values= hdc1080_read_values;

    return &hdc1080->parent;
}


