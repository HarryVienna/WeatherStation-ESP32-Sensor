#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "esp_log.h"

#include "wifi.h"
#include "mqtt_client.h"

#define NO_OF_SAMPLES   16 

#define MQTT_CONNECTED_BIT BIT0
#define MQTT_PUBLISHED_BIT BIT1
#define MQTT_DISCONNECTED_BIT BIT2

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

void app_main(){

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    ++boot_count;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    gpio_pad_select_gpio(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NUM_2, 0);

    esp_err_t ret;





    static esp_adc_cal_characteristics_t adc1_chars;
    
    ESP_ERROR_CHECK(esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, ESP_ADC_CAL_VAL_EFUSE_VREF, &adc1_chars));
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11));

    uint32_t voltage = 0;    
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw(ADC1_CHANNEL_7);
    }
    adc_reading /= NO_OF_SAMPLES;
    ESP_LOGI(TAG, "raw  data: %d", adc_reading);
    voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc1_chars) * 2; // Voltage divider is 1:1
    ESP_LOGI(TAG, "voltage: %d", voltage);





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

        smartconfig->init_sntp(smartconfig);

        smartconfig->init_timezone(smartconfig);
    }

     // Create a new event group.
    s_mqtt_event_group = xEventGroupCreate();
    if (s_mqtt_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
    }

    static esp_mqtt_client_config_t mqtt_cfg = {
        .uri  = "mqtt://192.168.0.200",
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
            char* json = "{  \"battery\": %d,  \"temperature\": %f,  \"humidity\": %f,  \"pressure\": %f}";
            char message[128];
            sprintf(message, json, voltage, 0.0, 1.0, 2.0);
            ESP_LOGI(TAG, "message %s", message);  
            int msg_id = esp_mqtt_client_publish(client, "/weatherstation/bedroom", message, 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        } else if (bits & MQTT_PUBLISHED_BIT) {
            ESP_LOGI(TAG, "MQTT_PUBLISHED_BIT");
            esp_mqtt_client_disconnect(client);
            ESP_LOGI(TAG, "disconnected");
        } else if (bits & MQTT_DISCONNECTED_BIT) {
            ESP_LOGI(TAG, "MQTT_DISCONNECTED_BIT");
            const int deep_sleep_sec = 60 * 10;
            ESP_LOGI(TAG, "Entering deep sleep for %d seconds", deep_sleep_sec);
            gpio_set_level(GPIO_NUM_2, 1);
            smartconfig->stop(smartconfig);
            esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
            esp_deep_sleep(1000000L * deep_sleep_sec);
        } 
    } while (true);


}
