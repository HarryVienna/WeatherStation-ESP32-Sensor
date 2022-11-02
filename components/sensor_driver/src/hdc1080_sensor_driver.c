#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "hdc1080_sensor_driver.h"

static const char *TAG = "hdc1080";


void swap_bytes(void *pv, size_t n)
{
    assert(n > 0);

    char *p = pv;
    size_t lo, hi;
    for(lo=0, hi=n-1; hi>lo; lo++, hi--)
    {
        char tmp=p[lo];
        p[lo] = p[hi];
        p[hi] = tmp;
    }
}

esp_err_t hdc1080_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, uint8_t id)
{
	esp_err_t err;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        goto end;
    }	
	
	err = i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_write_byte(cmd, reg_addr, true);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_write(cmd, reg_data, cnt, true);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_stop(cmd);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        goto end;
    }

end:	
	i2c_cmd_link_delete(cmd);
	return err;
}

esp_err_t hdc1080_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, uint8_t id)
{
	esp_err_t err;
 
 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_write_byte(cmd, reg_addr, true);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_read(cmd, reg_data, cnt, I2C_MASTER_LAST_NACK);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_stop(cmd);
    if (err != ESP_OK) {
        goto end;
    }

	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);

end:	
	i2c_cmd_link_delete(cmd);

	return err;
}


esp_err_t hdc1080_init(struct hdc1080_dev *dev)
{
    esp_err_t rslt = ESP_OK;

    /* chip id read try count */
    uint8_t try_count = 5;

    uint16_t device_id = 0;
    uint16_t manufacturer_id = 0;
    hdc1080_serial_t serial_id;
    uint16_t serial_first = 0;
    uint16_t serial_mid = 0;
    uint16_t serial_last = 0;
    
    /* Proceed if null check is fine */
    while (try_count)
    {
        /* Read the chip-id of bme280 sensor */

        rslt |= hdc1080_i2c_read(REG_DEVICE_ID, (uint8_t*) &device_id, 2, dev->i2c_addr);
        swap_bytes(&device_id, sizeof(device_id));

        rslt |= hdc1080_i2c_read(REG_MANUFACTURER_ID, (uint8_t*) &manufacturer_id, 2, dev->i2c_addr);
        swap_bytes(&manufacturer_id, sizeof(manufacturer_id));

       
       
       
        rslt |= hdc1080_i2c_read(REG_SERIAL_ID_FIRST, (uint8_t*) &serial_first, 2, dev->i2c_addr);
        swap_bytes(&serial_first, 2);
        rslt |= hdc1080_i2c_read(REG_SERIAL_ID_MID,   (uint8_t*) &serial_mid,   2, dev->i2c_addr);
        swap_bytes(&serial_mid, 2);
        rslt |= hdc1080_i2c_read(REG_SERIAL_ID_LAST,  (uint8_t*) &serial_last,  2, dev->i2c_addr);
        swap_bytes(&serial_last, 2);
        ESP_LOGE(TAG, "serial_id: fb %x fc %x fd %x", serial_first, serial_mid, serial_last);
        ESP_LOGE(TAG, "serial_id: fb %llx fc %llx fd %llx", (uint64_t)serial_first<<32, (uint64_t)serial_mid<<16, (uint64_t)serial_last);

        uint64_t s = (uint64_t)serial_first<<32 | (uint64_t)serial_mid<<16 | (uint64_t)serial_last;
        ESP_LOGE(TAG, "value: %llx", s);

         hdc1080_serial2_t x;
         x.value = s;
        ESP_LOGE(TAG, "year: %d", x.year);
        ESP_LOGE(TAG, "day: %d", x.day);
        ESP_LOGE(TAG, "month: %d", x.month);

        rslt |= hdc1080_i2c_read(REG_SERIAL_ID_FIRST, (uint8_t*) &serial_id.serialFirst, 2, dev->i2c_addr);
        swap_bytes(&serial_id.serialFirst, 2);
        rslt |= hdc1080_i2c_read(REG_SERIAL_ID_MID,   (uint8_t*) &serial_id.serialMid,   2, dev->i2c_addr);
        swap_bytes(&serial_id.serialMid, 2);
        rslt |= hdc1080_i2c_read(REG_SERIAL_ID_LAST,  (uint8_t*) &serial_id.serialLast,  2, dev->i2c_addr);
        swap_bytes(&serial_id.serialLast, 2);
        ESP_LOGE(TAG, "serial_id: fb %x fc %x fd %x", serial_id.serialFirst, serial_id.serialMid, serial_id.serialLast);
        serial_id.unused = 0;
        ESP_LOGE(TAG, "serial_id: %llx", serial_id.value);

        ESP_LOGE(TAG, "year: %d", (uint8_t)((serial_id.value >> 28) & 0x0001f));
        ESP_LOGE(TAG, "day: %d", (uint8_t)((serial_id.value >> 33) & 0x0001f));
        ESP_LOGE(TAG, "month: %d", (uint8_t)((serial_id.value >> 38) & 0x0001f));

        hdc1080_serial2_t xxx;
        xxx.value = serial_id.value >> 7;

        ESP_LOGE(TAG, "year: %d", xxx.year);
        ESP_LOGE(TAG, "day: %d", xxx.day);
        ESP_LOGE(TAG, "month: %d", xxx.month);

        /* Check for chip id validity */
        if ((rslt == ESP_OK) && (device_id == HDC1080_DEVICE_ID))
        {
            dev->device_id = device_id;
            /* Reset the sensor */
            //rslt = bme280_soft_reset(dev);

            break;
        }

        /* Wait for 10 ms */
        vTaskDelay(10/portTICK_PERIOD_MS);
        --try_count;
    }

    /* Chip id check failed */
    if (!try_count)
    {
        rslt = ESP_ERR_NOT_FOUND;
    }

    return rslt;
}

// |================================================================================================ |
// |                                           Initialisation                                        |
// |================================================================================================ |

/**
 * @brief Init Stepper
 *
 * @param motor handle to stepper_driver_t type object
 */
esp_err_t hdc1080_init_sensor(sensor_driver_t *handle)
{
    esp_err_t ret = ESP_OK;
    sensor_driver_hdc1080_t *hdc1080 = __containerof(handle, sensor_driver_hdc1080_t, parent);

	sensor_driver_hdc1080_conf_t hdc1080_config = hdc1080->driver_config;

    hdc1080->hdc1080_device.i2c_addr = hdc1080->driver_config.i2c_addr;
 
    ret = hdc1080_init(&hdc1080->hdc1080_device);

    return ret;
}

/**
 * @brief Clear GSTAT register
 *
 * @param motor handle to stepper_driver_t type object
 */
esp_err_t hdc1080_read_values(sensor_driver_t *handle, sensor_data_t *values)
{
    esp_err_t ret = ESP_OK;


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


