#pragma once


#include "driver/i2c.h"
#include "esp_err.h"

#include "sensor_driver.h"

#define HDC1080_I2C_ADDR 0x40

/**\name HDC1080 device id */
#define HDC1080_DEVICE_ID 0x1050

typedef enum {
	humidity_8bit = 0x02,
	humidity_11bit = 0x01,
	humidity_14bit = 0x00
} hdc1080_humidity_resolution_t;


typedef enum {
	temperature_11bit = 0x1,
	temperature_14bit = 0x0
} hdc1080_temperature_resolution_t;

/**
* @brief Driver register adresses
*
*/
enum uint8_t {
    REG_TEMPERATURE      = 0x00,
    REG_HUMIDITY         = 0x01,
    REG_CONFIGURATION    = 0x02,
    REG_MANUFACTURER_ID  = 0xFE,
    REG_DEVICE_ID        = 0xFF,
    REG_SERIAL_ID_FIRST  = 0xFB,
    REG_SERIAL_ID_MID    = 0xFC,
    REG_SERIAL_ID_LAST   = 0xFD,
    REG_LAST_ADDR    = REG_SERIAL_ID_LAST
} hdc1080_regaddr_t;

typedef union {
	uint16_t value;
	struct {
        uint16_t
		relative_humidity : 14,
        reserved : 2;
	};
} hdc1080_humidity_register_t;

typedef union {
	uint16_t value;
	struct {
        uint16_t
		relative_temperature : 14,
        reserved : 2;
	};
} hdc1080_temperature_register_t;

typedef union {
	uint16_t value;
	struct {
        uint16_t
		humidity_bitwidth : 2,
		temperature_bitwidth : 1,
		battery_status : 1,
		acquisition_mode : 1,
		heater : 1,
		reserved_again : 1,
		software_reset : 1,
        reserved : 8;
	};
} hdc1080_configuration_register_t;

typedef union {
	uint64_t value;
	struct {
		uint16_t serialLast;
		uint16_t serialMid;
        uint16_t serialFirst;		
        uint16_t unused;
	};
} hdc1080_serial_t;

typedef union {
	uint64_t value;
	struct {
        uint64_t
		reserved : 7,
        serial_1 : 21,
		year : 5,
		day : 5,
		month : 5,
		serial_2 : 5,
        unused : 16;
 	};
} hdc1080_serial2_t;

/**
* @brief Sensor Configuration Type
*
*/
typedef struct sensor_driver_hdc1080_conf_s {
    hdc1080_humidity_resolution_t humidity_resolution; 
    hdc1080_temperature_resolution_t temperature_resolution; 
	uint8_t i2c_addr;
} sensor_driver_hdc1080_conf_t;


struct hdc1080_dev
{
    /*< Device ID */
    uint16_t device_id;

    /*< Manufacturer ID */
    uint16_t manufacturer_id;

	/*< Serial ID */
    uint64_t serial_id;

    /*< I2C address */
    uint8_t i2c_addr;

   
};


typedef struct {
    sensor_driver_t parent;

    sensor_driver_hdc1080_conf_t driver_config;
    struct hdc1080_dev hdc1080_device;

} sensor_driver_hdc1080_t;


/**
* @brief Install a new HDC1080 driver 
*
* @param config: step motor configuration
* @return
*      step motor instance or NULL
*/
sensor_driver_t *sensor_driver_new_hdc1080(const sensor_driver_hdc1080_conf_t *config);














