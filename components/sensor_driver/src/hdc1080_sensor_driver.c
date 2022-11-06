#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "hdc1080_sensor_driver.h"


#define RAW2HUM(raw) (((float)raw) * 100/65536)
#define RAW2TEMP(raw) (((float)raw) * 165/65536 - 40)

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

uint16_t byteswap(uint16_t a)
{
  a = ((a & 0x00FF) << 8) | ((a & 0xFF00) >> 8);
  return a;
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



esp_err_t hdc1080_i2c_write_(uint8_t reg_addr, uint16_t data, uint8_t id)
{
    int ret;
    uint8_t write_buf[3] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_NUM_0, id, write_buf, sizeof(write_buf), 10);

    return ret;
}

esp_err_t hdc1080_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, uint8_t id)
{
	esp_err_t err;
 
 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_start %d", err);
        goto end;
    }

	err = i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_write_byte %d", err);
        goto end;
    }

	err = i2c_master_write_byte(cmd, reg_addr, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_write_byte %d", err);
        goto end;
    }

	err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_start %d", err);
        goto end;
    }

	err = i2c_master_write_byte(cmd, (id << 1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_write_byte %d", err);
        goto end;
    }

	err = i2c_master_read(cmd, reg_data, cnt, I2C_MASTER_LAST_NACK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_read %d", err);
        goto end;
    }

	err = i2c_master_stop(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_stop %d", err);
        goto end;
    }

	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_cmd_begin %d", err);
        goto end;
    }

end:	
	i2c_cmd_link_delete(cmd);

	return err;
}

esp_err_t hdc1080_i2c_read_(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, uint8_t id)
{
    esp_err_t err = i2c_master_write_read_device(I2C_NUM_0, id, &reg_addr, 1, reg_data, 2, 10);

    return err;
}

esp_err_t hdc1080_i2c_read_delay(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, uint8_t id)
{
    esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, id, &reg_addr, 1, 10);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    err = i2c_master_read_from_device(I2C_NUM_0, id, reg_data, cnt, 10);
    //swap_bytes(reg_data, cnt);
    return err;
}

esp_err_t hdc1080_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, uint8_t i2c_addr)
{
    esp_err_t rslt = ESP_OK;

    rslt = hdc1080_i2c_read_(reg_addr, reg_data, len, i2c_addr);
    swap_bytes(reg_data, len);

    return rslt;
}

esp_err_t hdc1080_write_reg(uint8_t reg_addr, uint16_t reg_data,  uint8_t i2c_addr)
{
    esp_err_t rslt = ESP_OK;

    swap_bytes(&reg_data, 2);
    rslt = hdc1080_i2c_write(reg_addr, (uint8_t *)&reg_data, 2, i2c_addr);

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


   vTaskDelay(DEVICE_STARTUP_TIME / portTICK_PERIOD_MS);


    uint16_t device_id = 0;
    uint16_t manufacturer_id = 0;
    hdc1080_serial_t serial_id;
    uint16_t serial_first = 0;
    uint16_t serial_mid = 0;
    uint16_t serial_last = 0;
    

    ret |= hdc1080_get_regs(REG_DEVICE_ID, (uint8_t*) &device_id, 2, hdc1080->driver_config.i2c_addr);
ESP_LOGE(TAG, "rslt: %d", ret);
    ret |= hdc1080_get_regs(REG_MANUFACTURER_ID, (uint8_t*) &manufacturer_id, 2, hdc1080->driver_config.i2c_addr);
ESP_LOGE(TAG, "rslt: %d", ret);
    
    ret |= hdc1080_get_regs(REG_SERIAL_ID_FIRST, (uint8_t*) &serial_first, 2, hdc1080->driver_config.i2c_addr);
    ESP_LOGE(TAG, "rslt: %d", ret);
    ret |= hdc1080_get_regs(REG_SERIAL_ID_MID,   (uint8_t*) &serial_mid,   2, hdc1080->driver_config.i2c_addr);
    ESP_LOGE(TAG, "rslt: %d", ret);
    ret |= hdc1080_get_regs(REG_SERIAL_ID_LAST,  (uint8_t*) &serial_last,  2, hdc1080->driver_config.i2c_addr);
    ESP_LOGE(TAG, "rslt: %d", ret);

    ESP_LOGE(TAG, "serial_id: fb %x fc %x fd %x", serial_first, serial_mid, serial_last);
    ESP_LOGE(TAG, "serial_id: fb %llx fc %llx fd %llx", (uint64_t)serial_first<<32, (uint64_t)serial_mid<<16, (uint64_t)serial_last);

    serial_id.value = (uint64_t)serial_first<<32 | (uint64_t)serial_mid<<16 | (uint64_t)serial_last;
    ESP_LOGE(TAG, "year: %d", serial_id.year);
    ESP_LOGE(TAG, "day: %d", serial_id.day);
    ESP_LOGE(TAG, "month: %d", serial_id.month);

    ESP_LOGE(TAG, "device_id   manufacturer_id  %d %x %x", ret, device_id, manufacturer_id);




    uint16_t conf = 0;
    //ret =  hdc1080_get_regs(REG_CONFIGURATION, (uint8_t*) &conf, 2, hdc1080->driver_config.i2c_addr);
    ret =  hdc1080_get_regs(REG_CONFIGURATION, (uint8_t*) &conf, 2, hdc1080->driver_config.i2c_addr);
    ESP_LOGE(TAG, "hdc1080_get_regs rslt: %d %x", ret, conf);

    hdc1080_configuration_register_t c = {
        .software_reset = 0,
        .heater = 0,
        .acquisition_mode = 1,
        .humidity_resolution = hdc1080->driver_config.humidity_resolution,
        .temperature_resolution = hdc1080->driver_config.temperature_resolution,
    };

    ret = hdc1080_write_reg(REG_CONFIGURATION, c.value, hdc1080->driver_config.i2c_addr);
    ESP_LOGE(TAG, "hdc1080_i2c_write rslt: %d", ret);

    ret =  hdc1080_get_regs(REG_CONFIGURATION, (uint8_t*) &conf, 2, hdc1080->driver_config.i2c_addr);
    ESP_LOGE(TAG, "hdc1080_get_regs rslt: %d %x %x", ret, c.value, conf);


 
    /* Check for chip id validity */
    if ((ret == ESP_OK) && (device_id == HDC1080_DEVICE_ID))
    {
        //dev->device_id = device_id;
        //dev->manufacturer_id= manufacturer_id;

        ESP_LOGE(TAG, "device_id   manufacturer_id  %d %x %x", ret, device_id, manufacturer_id);
        /* Reset the sensor */
        //rslt = bme280_soft_reset(dev);

    }

    uint8_t raw[4];
    uint16_t result;
    hdc1080_i2c_read_delay(REG_TEMPERATURE, (uint8_t*)&raw, 4, hdc1080->driver_config.i2c_addr);
    ESP_LOGE(TAG, "REG_TEMPERATURE  %x %x %x %x", raw[0], raw[1], raw[2], raw[3]);

    result = raw[0]<<8 | raw[1];
    float temperature = RAW2TEMP(result);
    result = raw[2]<<8 | raw[3];
    float humidity = RAW2HUM(result);

    ESP_LOGE(TAG, "temp  humidity %f %f", temperature, humidity);

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


