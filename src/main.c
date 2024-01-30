#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/rtc_io.h"
#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "bme280_sensor_driver.h"

#undef SENSOR_DFROBOT // Set this for the old board with FireBeetle 2 ESP32 Microcontroller 


#define NVS_NAMESPACE "SENSOR"
#define MAC_VALUE "MAC"
#define CHANNEL_VALUE "CHANNEL"

#define ESPNOW_MAXDELAY 200
#define ESPNOW_QUEUE_SIZE 16
#define ESPNOW_SEND_SUCCESSFUL_BIT BIT0
#define ESPNOW_SEND_FAIL_BIT BIT1

#define MAX_CHANNEL 13 

#define NO_OF_SAMPLES 16 

#define LED GPIO_NUM_2


#ifdef SENSOR_DFROBOT
    #define SENSOR_NR_0 GPIO_NUM_25
    #define SENSOR_NR_1 GPIO_NUM_26
#else
    #define SENSOR_NR_2 GPIO_NUM_25
    #define SENSOR_NR_1 GPIO_NUM_33
    #define SENSOR_NR_0 GPIO_NUM_32
#endif

#define VOLTAGE_ADC_CHANNEL ADC1_CHANNEL_7

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ 100000

#define SENSOR_SLEEPTIME 600

enum MessageType {
    PAIRING_REQ, 
    PAIRING_RESP, 
    DATA,
};

RTC_DATA_ATTR static int boot_count = 0;

static const char* TAG = "main";

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static xQueueHandle s_espnow_queue;
static EventGroupHandle_t s_espnow_event_group;

typedef struct struct_data {
    uint8_t msg_type;
    uint8_t sensor_nr;
    uint32_t voltage;
    double pressure;
    double temperature;
    double humidity;
} struct_data;

typedef struct struct_pairing_response { 
    uint8_t msg_type;
    uint8_t sensor_nr;    
    uint8_t macAddr[ESP_NOW_ETH_ALEN];
    uint8_t channel;
} struct_pairing_response;

typedef struct struct_pairing_request { 
    uint8_t msg_type;
    uint8_t sensor_nr;    
} struct_pairing_request;


/**
* @brief Blink an LED connected to a GPIO pin
* 
* This function blinks an LED connected to the specified GPIO pin.
*
* @note This function assumes that an LED is connected to GPIO pin GPIO_NUM_2.
*/
static void blink() {
    gpio_pad_select_gpio(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(LED, 0);
}

/**
* @brief Get sensor number from GPIO pins
*
* This function reads the sensor number from the specified GPIO pins and stores the result in the provided
* pointer to a uint8_t variable.
* @param[in,out] nr Pointer to a uint8_t variable to store the sensor number
*
* @return esp_err_t Returns ESP_OK on success, otherwise an error code indicating the cause of failure
*
* @note This function assumes that the sensor number is encoded on three GPIO pins, SENSOR_NR_0, SENSOR_NR_1, and SENSOR_NR_2.
  The sensor number is encoded as a 3-bit binary value, with the least significant bit on SENSOR_NR_0 and the most
  significant bit on SENSOR_NR_2.
*/
static esp_err_t get_sensor_number(uint8_t *nr) 
{
    #ifdef SENSOR_DFROBOT
        gpio_set_direction(SENSOR_NR_0, GPIO_MODE_INPUT);   
        gpio_set_direction(SENSOR_NR_1, GPIO_MODE_INPUT);  

        gpio_set_pull_mode(SENSOR_NR_0, GPIO_PULLUP_PULLDOWN);
        gpio_set_pull_mode(SENSOR_NR_1, GPIO_PULLUP_PULLDOWN);

        int bit_0 = gpio_get_level(SENSOR_NR_0);
        int bit_1 = gpio_get_level(SENSOR_NR_1);
    
        *nr = bit_1 << 1 | bit_0;
    #else
        gpio_set_direction(SENSOR_NR_0, GPIO_MODE_INPUT);   
        gpio_set_direction(SENSOR_NR_1, GPIO_MODE_INPUT);  
        gpio_set_direction(SENSOR_NR_2, GPIO_MODE_INPUT);  

        gpio_set_pull_mode(SENSOR_NR_0, GPIO_PULLUP_PULLDOWN);
        gpio_set_pull_mode(SENSOR_NR_1, GPIO_PULLUP_PULLDOWN);
        gpio_set_pull_mode(SENSOR_NR_2, GPIO_PULLUP_PULLDOWN);

        int bit_0 = gpio_get_level(SENSOR_NR_0);
        int bit_1 = gpio_get_level(SENSOR_NR_1);
        int bit_2 = gpio_get_level(SENSOR_NR_2);
    
        *nr = bit_2 << 2 | bit_1 << 1 | bit_0;
    #endif

    return ESP_OK;
}

/**
*
* @brief Get voltage value from ADC reading
*
* This function reads the voltage from the specified ADC channel and calculates the actual voltage value
* using a voltage divider of 2.2:4.7. The voltage value is stored in the provided pointer to a uint32_t variable.
*
* @param[in,out] voltage Pointer to a uint32_t variable to store the calculated voltage value
*
* @return esp_err_t Returns ESP_OK on success, otherwise an error code indicating the cause of failure
*/
static esp_err_t get_voltage(uint32_t *voltage) 
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

    #ifdef SENSOR_DFROBOT
        *voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc1_chars) * 2; // Voltage divider is 1:1   // Altes Board
    #else
        *voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc1_chars) * ((4.7 + 2.2) / 4.7); // Voltage divider is 2.2 : 4.7
    #endif

    return ESP_OK;
}

/**
* @brief Initialize I2C communication using given configuration
*
* This function initializes the I2C communication by setting the I2C parameters and installing the I2C driver
* for the specified I2C peripheral number. The configuration is passed as a pointer to an i2c_config_t structure.
*
* @param[in] i2c_config Pointer to an i2c_config_t structure containing the I2C configuration
*
* @return None
*/
static void i2c_init(i2c_config_t *i2c_config)
{

	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, i2c_config));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_config->mode, 0, 0, 0));
}

/**
* @brief Initialize the Non-Volatile Storage (NVS) module
*
* This function initializes the ESP32 Non-Volatile Storage (NVS) module.
* If the NVS module has already been initialized and there are no free pages or a new version has been found,
* the function will erase the existing data and reinitialize the module.
*
* @note This function should be called before using the NVS module.
*
* @note If the NVS module has already been initialized and there are no issues, this function does nothing.
*/
static void nvs_init(void)
{
    //nvs_flash_erase(); // JUST FOR TEST

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
}

/**
* @brief Initialize the Wi-Fi module in station mode
*
* This function initializes the ESP32 Wi-Fi module in station mode, 
* which allows the device to connect to an existing Wi-Fi network.
*
* @note This function should be called before using any Wi-Fi related functions.
*/
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
}


/**
* @brief Callback function to handle ESP-NOW send events.
*
* This function is used as a callback function to handle ESP-NOW send events.
* It sets bits in the event group based on the status of the send operation.
*
* @param mac_addr The MAC address of the recipient device.
* @param status The status of the send operation.
*          ESP_NOW_SEND_SUCCESS if the send operation was successful.
*          ESP_NOW_SEND_FAIL if the send operation failed.
* @return void
*/
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        xEventGroupSetBits(s_espnow_event_group, ESPNOW_SEND_SUCCESSFUL_BIT);
    } else {
        xEventGroupSetBits(s_espnow_event_group, ESPNOW_SEND_FAIL_BIT);
    }
}

/**
* @brief ESP-NOW receive callback function.
*
* This function is called when ESP-NOW data is received by the ESP32.
* It handles the data received from the paired ESP32 device and forwards it to a queue.
*
* @param[in] mac_addr MAC address of the device that sent the data.
* @param[in] data The received data payload.
* @param[in] len Length of the received data payload.
* @return None
*/
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    ESP_LOGI(TAG, "esp-now receive callback");
    struct_pairing_response pairingResponse;

    uint8_t type = data[0];
    switch (type) {
    case PAIRING_RESP:    // pairing response from server
        memcpy(&pairingResponse, data, sizeof(struct_pairing_response));

        if (xQueueSend(s_espnow_queue, &pairingResponse, ESPNOW_MAXDELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Send send queue fail");
        }

        break;
  }
}


/**
* @brief Initialize ESP-NOW module
*
* This function initializes the ESP-NOW module by calling esp_now_init() and registering
* the send and receive callback functions.
* 
* @return None
*/
void espnow_init(void)
{
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
}

/**
* @brief Read peer device MAC address and channel from NVS storage
*
* This function reads the MAC address and channel number of the paired device from the
* non-volatile storage (NVS).
*
* @param[in] mac_addr Pointer to the buffer where the MAC address will be stored.
*
* @param[in] chan Pointer to the variable where the channel number will be stored.
* 
* @return ESP_OK on success, ESP_FAIL otherwise.
*/
esp_err_t read_peer_nvs(uint8_t *mac_addr, uint8_t *chan) 
{
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        return ESP_FAIL;
    }
    size_t required_size = ESP_NOW_ETH_ALEN;
    err = nvs_get_blob(my_handle, MAC_VALUE, mac_addr, &required_size);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        return ESP_FAIL;
    }
    err = nvs_get_u8(my_handle, CHANNEL_VALUE, chan);
    if (err != ESP_OK) {
        nvs_close(my_handle);
        return ESP_FAIL;
    }

    nvs_close(my_handle);

    return ESP_OK;
}

/**
* @brief Store peer device MAC address and channel in NVS storage
*
* This function stores the MAC address and channel number of the paired device in the
* non-volatile storage (NVS).
*
* @param[in] mac_addr Pointer to the buffer containing the MAC address.
*
* @param[in] chan Channel number to be stored.
* 
* @return None.
*/
void store_peer_nvs(const uint8_t * mac_addr, uint8_t chan) 
{
    nvs_handle_t my_handle;

    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_blob(my_handle, MAC_VALUE, mac_addr, ESP_NOW_ETH_ALEN));
    ESP_ERROR_CHECK(nvs_set_u8(my_handle, CHANNEL_VALUE, chan));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    
    nvs_close(my_handle);
}

/**
 * @brief Delete peer device MAC address and channel from NVS storage
 *
 * This function erases the MAC address and channel number of the paired device from
 * the non-volatile storage (NVS).
 *
 * @return None.
 */
void delete_peer_nvs() 
{
    nvs_handle_t my_handle;

    ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_erase_key(my_handle, MAC_VALUE));
    ESP_ERROR_CHECK(nvs_erase_key(my_handle, CHANNEL_VALUE));
    ESP_ERROR_CHECK(nvs_commit(my_handle));
    
    nvs_close(my_handle);
}

/**
 * @brief Add a new peer device to the ESP-NOW peer list
 *
 * This function adds a new peer device to the ESP-NOW peer list with the given MAC address
 * and channel number.
 *
 * @param[in] mac_addr Pointer to the buffer containing the MAC address of the peer device.
 * @param[in] chan Channel number to be used for communication with the peer device.
 *
 * @return None.
 */
void add_peer(const uint8_t * mac_addr, uint8_t chan){
  esp_now_peer_info_t peer;

  esp_now_del_peer(mac_addr);

  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);

  ESP_ERROR_CHECK( esp_now_add_peer(&peer) );
}


/**
 * @brief Start the pairing process by sending a broadcast message and waiting for a response
 *
 * This function sends a broadcast message to all channels and waits for a response from a sensor.
 * If a response is received, the function returns the MAC address and channel of the paired sensor.
 *
 * @param[in] mac_addr Pointer to a uint8_t array to store the MAC address of the paired sensor.
 * @param[in] chan Pointer to a uint8_t variable to store the channel of the paired sensor.
 * @param[in] sensor_nr Sensor number to match for pairing.
 *
 * @return - ESP_OK if pairing is successful and MAC address and channel are stored in mac_addr and chan, respectively.
 *         - ESP_FAIL if pairing is not successful.
 *
 * @note This function assumes that esp_now has been initialized and add_peer function has been called to add the broadcast MAC address.
 */
esp_err_t start_pairing(uint8_t *mac_addr, uint8_t *chan, uint8_t sensor_nr)
{
    struct_pairing_request pair_req;
    struct_pairing_response pair_resp;

    pair_req.msg_type = PAIRING_REQ; 
    pair_req.sensor_nr = sensor_nr;

    for (int channel = 1; channel <= MAX_CHANNEL; channel++) {

        ESP_LOGI(TAG, "Sending broadcast to channel: %d", channel);
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

        add_peer(s_broadcast_mac, channel);

        ESP_ERROR_CHECK(esp_now_send(s_broadcast_mac, (uint8_t *) &pair_req, sizeof(struct_pairing_request)));

        if (xQueueReceive(s_espnow_queue, &pair_resp, ESPNOW_MAXDELAY / portTICK_RATE_MS) == pdTRUE) {
            ESP_LOGI(TAG, "Get Message");
            ESP_LOGI(TAG, "Received Sensor Nr: %d", pair_resp.sensor_nr);
            ESP_LOGI(TAG, "Received Channel: %d", pair_resp.channel);
            ESP_LOGI(TAG, "Received MAC "MACSTR"", MAC2STR(pair_resp.macAddr));

            memcpy(mac_addr, pair_resp.macAddr, ESP_NOW_ETH_ALEN);
            *chan = pair_resp.channel;

            return ESP_OK;
        }
    }

    ESP_LOGI(TAG, "Pairing not successful");
    return ESP_FAIL;
}

/**
 * @brief Start the deep sleep mode for the specified duration
 *
 * This function configures the power domains to be turned off during deep sleep and puts the ESP32 into deep sleep for the specified duration.
 * Upon wake up, the program will restart.
 *
 * @return none
 *
 * @note This function assumes that ESP_LOGI and esp_sleep_pd_config functions are available.
 * @note SENSOR_SLEEPTIME is a macro defined in a header file and represents the duration of sleep time in seconds.
 */
void start_deep_sleep() 
{
    ESP_LOGI(TAG, "Start Deep Sleep");
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);

    // https://electronics.stackexchange.com/questions/530151/esp32-wroom32-consuming-77-%c2%b5a-much-too-high-in-deep-sleep
    #ifdef SENSOR_DFROBOT
        gpio_reset_pin(SENSOR_NR_0);
        gpio_reset_pin(SENSOR_NR_1);
    #else
        gpio_reset_pin(SENSOR_NR_0);
        gpio_reset_pin(SENSOR_NR_1);
        gpio_reset_pin(SENSOR_NR_2);
    #endif

    // https://www.esp32.com/viewtopic.php?t=3634
    // gpio_pad_select_gpio(GPIO_NUM_32);
    // gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(GPIO_NUM_32, GPIO_PULLUP_ONLY);

    // gpio_pad_select_gpio(GPIO_NUM_33);
    // gpio_set_direction(GPIO_NUM_33, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(GPIO_NUM_33, GPIO_PULLUP_ONLY);

    esp_deep_sleep(1000000L * SENSOR_SLEEPTIME);
}


void app_main(){

    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    ++boot_count;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    esp_err_t ret;

    // ------- Blink LED -------
    blink();

    // ------- Read DIP switch value -------
    uint8_t sensor_nr;    
    ret = get_sensor_number(&sensor_nr);
    ESP_LOGI(TAG, "sensor: %d", sensor_nr);

    // ------- Read voltage value -------
    uint32_t voltage = 0;    
    ret = get_voltage(&voltage);
    ESP_LOGI(TAG, "voltage: %d", voltage);


    // ------- Read sensors -------
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ
	};

    i2c_init(&i2c_config);
   
    sensor_data_t values_bme280;
    const sensor_driver_bme280_conf_t bme280_config = {
        .osr_p = BME280_OVERSAMPLING_1X,
        .osr_t = BME280_OVERSAMPLING_1X,
        .osr_h = BME280_OVERSAMPLING_1X,
        .filter = BME280_FILTER_COEFF_OFF,
        .dev_id = BME280_I2C_ADDR_PRIM
    };

    sensor_driver_t *bme280_driver = sensor_driver_new_bme280(&bme280_config);

    ret = sensor_driver_init_sensor(bme280_driver);

    if (ret == ESP_OK) {
        sensor_driver_read_values(bme280_driver, &values_bme280);

        ESP_LOGI(TAG, "bme280 %0.2f C / %.2f %% / %.2f hPa", values_bme280.temperature, values_bme280.humidity, values_bme280.pressure);
    }

    // ------- Transmit data -------
    uint8_t peer_mac[ESP_NOW_ETH_ALEN];
    uint8_t chan;

    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(struct_pairing_response));
    if (s_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
    }
    s_espnow_event_group = xEventGroupCreate();
      if (s_espnow_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
    }

    nvs_init();
    wifi_init();
    espnow_init();

    // Try to read channel and MAC from NVS
    if (read_peer_nvs(peer_mac, &chan) != ESP_OK) {
        // If the values are not available, start a pairing
        if (start_pairing(peer_mac, &chan, sensor_nr) == ESP_OK) {
            store_peer_nvs(peer_mac, chan);
        }
        else {
            start_deep_sleep(); // Try later again
        }
    }

    ESP_LOGI(TAG, "NVS Channel: %d", chan);
    ESP_LOGI(TAG, "NVS MAC "MACSTR"", MAC2STR(peer_mac));

    add_peer(peer_mac, chan);
    esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);

    // Create message to send
    struct_data msg;
    msg.msg_type = DATA;
    msg.sensor_nr = sensor_nr;
    msg.voltage = voltage;
    msg.temperature = values_bme280.temperature;
    msg.humidity = values_bme280.humidity;
    msg.pressure = values_bme280.pressure;

    ESP_ERROR_CHECK(esp_now_send(peer_mac, (uint8_t *) &msg, sizeof(struct_data)));

    EventBits_t bits = xEventGroupWaitBits(s_espnow_event_group,
            ESPNOW_SEND_SUCCESSFUL_BIT | ESPNOW_SEND_FAIL_BIT,
            pdTRUE,
            pdFALSE,
            ESPNOW_MAXDELAY);
    if (bits & ESPNOW_SEND_SUCCESSFUL_BIT) {
        ESP_LOGI(TAG, "Message sent successful");
    } else if (bits & ESPNOW_SEND_FAIL_BIT) {
        ESP_LOGI(TAG, "Message sent failed");
        delete_peer_nvs(); // There was a problem. WIFI down? Channel changed? Whatever, delete pairing info and start again at the next wake up from deep sleep
    }      

    start_deep_sleep();
   
}
