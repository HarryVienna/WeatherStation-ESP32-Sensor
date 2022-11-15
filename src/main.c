#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "esp_log.h"

#include "wifi.h"
#include "mqtt_client.h"

#include "hdc1080_sensor_driver.h"
#include "bme280_sensor_driver.h"



#define NO_OF_SAMPLES   16 

#define SENSOR_NR_0 GPIO_NUM_25
#define SENSOR_NR_1 GPIO_NUM_26

#define VOLTAGE_ADC_CHANNEL ADC1_CHANNEL_7

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_FREQ_HZ 100000

#define MQTT_MESSAGE "{ \"battery\": %d,  \"temperature\": %f,  \"humidity\": %f,  \"pressure\": %f }"
#define MQTT_SUBJECT "/weatherstation/sensor/%02d"
#define MQTT_SERVER  "mqtt://192.168.0.200"
#define MQTT_CONNECTED_BIT BIT0
#define MQTT_PUBLISHED_BIT BIT1
#define MQTT_DISCONNECTED_BIT BIT2

#define SENSOR_SLEEPTIME 600

RTC_DATA_ATTR static int boot_count = 0;

static const char* TAG = "main";

static EventGroupHandle_t s_mqtt_event_group;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        xEventGroupSetBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        xEventGroupSetBits(s_mqtt_event_group, MQTT_PUBLISHED_BIT);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        xEventGroupSetBits(s_mqtt_event_group, MQTT_DISCONNECTED_BIT);
        break;    
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


esp_err_t blink() {
    gpio_pad_select_gpio(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NUM_2, 0);

    return ESP_OK;
}

esp_err_t get_sensor_number(uint32_t *nr) 
{
    gpio_set_direction(SENSOR_NR_0, GPIO_MODE_INPUT);   
    gpio_set_direction(SENSOR_NR_1, GPIO_MODE_INPUT);  

    gpio_set_pull_mode(SENSOR_NR_0, GPIO_PULLUP_PULLDOWN);
    gpio_set_pull_mode(SENSOR_NR_1, GPIO_PULLUP_PULLDOWN);

    int bit_0 = gpio_get_level(SENSOR_NR_0);
    int bit_1 = gpio_get_level(SENSOR_NR_1);
   
    *nr = bit_1 << 1 | bit_0;

    return ESP_OK;
}

esp_err_t get_voltage(uint32_t *voltage) 
{
    esp_adc_cal_characteristics_t adc1_chars;
    
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, ESP_ADC_CAL_VAL_EFUSE_VREF, &adc1_chars);
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(VOLTAGE_ADC_CHANNEL, ADC_ATTEN_DB_11);

    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw(VOLTAGE_ADC_CHANNEL);
    }
    adc_reading /= NO_OF_SAMPLES;
    ESP_LOGI(TAG, "raw  data: %d", adc_reading);
    *voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc1_chars) * 2; // Voltage divider is 1:1

    return ESP_OK;
}


void i2c_init(i2c_config_t *i2c_config)
{

	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, i2c_config));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_config->mode, 0, 0, 0));
}


void app_main(){

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    ++boot_count;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    esp_err_t ret;

    ret = blink();


    uint32_t sensor_nr = 0;    
    ret = get_sensor_number(&sensor_nr);
    ESP_LOGI(TAG, "sensor: %d", sensor_nr);


    uint32_t voltage = 0;    
    ret = get_voltage(&voltage);
    ESP_LOGI(TAG, "voltage: %d", voltage);



	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ
	};

    i2c_init(&i2c_config);


/*
    const sensor_driver_hdc1080_conf_t hdc1080_config = {
        .humidity_resolution = humidity_14bit,
        .temperature_resolution = temperature_11bit,
        .i2c_addr = HDC1080_I2C_ADDR
    };

    sensor_driver_t *hdc1080_driver = sensor_driver_new_hdc1080(&hdc1080_config);

    ret = sensor_driver_init_sensor(hdc1080_driver);

    sensor_data_t values;
    sensor_driver_read_values(hdc1080_driver, &values);

*/


    const sensor_driver_bme280_conf_t bme280_config = {
        .osr_p = BME280_OVERSAMPLING_1X,
        .osr_t = BME280_OVERSAMPLING_1X,
        .osr_h = BME280_OVERSAMPLING_1X,
        .filter = BME280_FILTER_COEFF_OFF,
        .dev_id = BME280_I2C_ADDR_PRIM
    };

    sensor_driver_t *bme280_driver = sensor_driver_new_bme280(&bme280_config);

    ret = sensor_driver_init_sensor(bme280_driver);

    sensor_data_t values;
    sensor_driver_read_values(bme280_driver, &values);

    ESP_LOGI(TAG, "%0.2f C / %.2f %%", values.temperature, values.humidity);




    char message[128];
    sprintf(message, MQTT_MESSAGE, voltage, values.temperature, values.humidity, values.pressure);
    ESP_LOGI(TAG, "message %s", message);  
    char subject[32];
    sprintf(subject, MQTT_SUBJECT, sensor_nr);
    ESP_LOGI(TAG, "subject %s", subject);  


    static wifi_conf_t wifi_conf = {
        .aes_key = "ESP32EXAMPLECODE",
        .hostname = "ESP32",
        .ntp_server = "pool.ntp.org",
    };
    wifi_t *smartconfig = wifi_new_smartconfig(&wifi_conf);

    if (smartconfig->init(smartconfig) == ESP_OK) {

        do {
            ret = smartconfig->connect(smartconfig);
        }
        while (ret != ESP_OK);
    }


    // Create a new event group.
    s_mqtt_event_group = xEventGroupCreate();
    if (s_mqtt_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
    }

    static esp_mqtt_client_config_t mqtt_cfg = {
        .uri  = MQTT_SERVER,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, client);
    esp_mqtt_client_start(client);

    EventBits_t bits;
    do {
        bits = xEventGroupWaitBits(s_mqtt_event_group,
                MQTT_CONNECTED_BIT | MQTT_PUBLISHED_BIT | MQTT_DISCONNECTED_BIT,
                pdTRUE,
                pdFALSE,
                portMAX_DELAY);    
        if (bits & MQTT_CONNECTED_BIT) {
            ESP_LOGI(TAG, "MQTT_CONNECTED_BIT");    
            int msg_id = esp_mqtt_client_publish(client, subject, message, 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        } else if (bits & MQTT_PUBLISHED_BIT) {
            ESP_LOGI(TAG, "MQTT_PUBLISHED_BIT");
            esp_mqtt_client_disconnect(client);
            ESP_LOGI(TAG, "disconnected");
        } else if (bits & MQTT_DISCONNECTED_BIT) {
            ESP_LOGI(TAG, "MQTT_DISCONNECTED_BIT");
            ESP_LOGI(TAG, "Entering deep sleep for %d seconds", SENSOR_SLEEPTIME);
            gpio_set_level(GPIO_NUM_2, 1);
            smartconfig->stop(smartconfig);
            esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
            esp_deep_sleep(1000000L * SENSOR_SLEEPTIME);
        } 
    } while (true);


}
