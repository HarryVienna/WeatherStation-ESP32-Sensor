#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "sensor_driver_bme280.h"

static const char *TAG = "bme280";


int8_t BME280_I2C_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *intf_ptr)
{
    uint8_t id;

    id = *(uint8_t *)intf_ptr;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, true));

	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
	ESP_ERROR_CHECK(i2c_master_write(cmd, reg_data, cnt, true));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);

	return espRc;
}

int8_t BME280_I2C_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *intf_ptr)
{
	esp_err_t espRc;
    uint8_t id;

    id = *(uint8_t *)intf_ptr;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, true));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));

	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_READ, true));

	ESP_ERROR_CHECK(i2c_master_read(cmd, reg_data, cnt, I2C_MASTER_LAST_NACK));

	ESP_ERROR_CHECK(i2c_master_stop(cmd));

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);

	return espRc;
}

void BME280_delay_us(uint32_t delay, void *intf_ptr)
{
	//vTaskDelay(v/1000/portTICK_PERIOD_MS);
	ets_delay_us(delay);  // Be carefull, should not be used in FreeRTOS
}


// |================================================================================================ |
// |                                           Initialisation                                        |
// |================================================================================================ |

/**
 * @brief Init Stepper
 *
 * @param motor handle to stepper_driver_t type object
 */
esp_err_t bme280_init_sensor(sensor_driver_t *handle)
{
    esp_err_t ret = ESP_OK;
    sensor_driver_bme280_t *tmc2208 = __containerof(handle, sensor_driver_bme280_t, parent);

	sensor_driver_bme280_conf_t bme280_config = tmc2208->driver_config;
	//struct bme280_dev bme280_device = tmc2208->bme280_device;

	tmc2208->bme280_device.write = BME280_I2C_bus_write;
	tmc2208->bme280_device.read = BME280_I2C_bus_read;
	tmc2208->bme280_device.intf = BME280_I2C_INTF;
	tmc2208->bme280_device.delay_us = BME280_delay_us;

    tmc2208->bme280_device.intf_ptr = &tmc2208->driver_config.dev_id;


	ret = bme280_init(&tmc2208->bme280_device);

	tmc2208->bme280_device.settings.osr_p = tmc2208->driver_config.osr_p;
	tmc2208->bme280_device.settings.osr_t = tmc2208->driver_config.osr_t;
	tmc2208->bme280_device.settings.osr_h = tmc2208->driver_config.osr_h;
	tmc2208->bme280_device.settings.filter = tmc2208->driver_config.filter;

	uint8_t desired_settings = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
	ret = bme280_set_sensor_settings(desired_settings, &tmc2208->bme280_device);

    return ret;
}

/**
 * @brief Clear GSTAT register
 *
 * @param motor handle to stepper_driver_t type object
 */
esp_err_t bme280_read_values(sensor_driver_t *handle, sensor_data_t *values)
{
    esp_err_t ret = ESP_OK;
	sensor_driver_bme280_t *tmc2208 = __containerof(handle, sensor_driver_bme280_t, parent);

	sensor_driver_bme280_conf_t bme280_config = tmc2208->driver_config;
	//struct bme280_dev bme280_device = tmc2208->bme280_device;


	ret = bme280_set_sensor_mode(BME280_FORCED_MODE, &tmc2208->bme280_device);

	uint32_t delay = bme280_cal_meas_delay(&tmc2208->bme280_device.settings);
	//if (delay < 10) {
	//	delay = 10;
	//}
	ESP_LOGE(TAG, "delay: %d", delay);
	tmc2208->bme280_device.delay_us(delay * 1000, tmc2208->bme280_device.intf_ptr);


	struct bme280_data comp_data;

	ret = bme280_get_sensor_data(BME280_ALL, &comp_data, &tmc2208->bme280_device);

	if (ret == BME280_OK) {
		values->temperature = comp_data.temperature;
		values->pressure = comp_data.pressure / 100.0;
		values->humidity = comp_data.humidity;
		ESP_LOGI(TAG, "%0.2f degC / %.2f kPa / %.2f %%", comp_data.temperature, comp_data.pressure, comp_data.humidity);
	} 	

    return ret;
}


sensor_driver_t *sensor_driver_new_bme280(const sensor_driver_bme280_conf_t *config)
{
    sensor_driver_bme280_t *tmc2208 = calloc(1, sizeof(sensor_driver_bme280_t));

    tmc2208->driver_config.osr_t = config->osr_t;
    tmc2208->driver_config.osr_h = config->osr_h;
	tmc2208->driver_config.osr_p = config->osr_p;
	tmc2208->driver_config.filter = config->filter;
	tmc2208->driver_config.dev_id = config->dev_id;

    tmc2208->parent.init_sensor = bme280_init_sensor;
    tmc2208->parent.read_values= bme280_read_values;

    return &tmc2208->parent;
}


