/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "owb.h"
#include "ds18b20.h"
#include "mqtt.h"

#define GPIO_LED             (GPIO_NUM_2)
#define GPIO_ONE_WIRE        (CONFIG_ONE_WIRE_GPIO)
#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_10_BIT)
#define SAMPLE_PERIOD        (4000)  // sensor sampling period in milliseconds
#define PUBLISH_QUEUE_DEPTH  (16)

#define TAG "poolmon"
//static const char * TAG = "poolmon";

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

// let's try a global variable for now
static mqtt_client * client = NULL;

typedef struct
{
    OneWireBus * owb;
    OneWireBus_ROMCode * rom_codes;
    DS18B20_Info ** ds18b20_infos;
    int num_ds18b20s;
} Sensors;

typedef struct
{
    Sensors * sensors;
    QueueHandle_t publish_queue;
} SensorTaskInputs;

typedef struct
{
    int sensor_id;
    float value;
} SensorReading;

/**
 * @brief Find all (or the first N) ROM codes for devices connected to the One Wire Bus.
 * @param[in] owb Pointer to initialised bus instance.
 * @param[in] devices A pre-existing array of ROM code structures, sufficient to hold max_devices entries.
 * @param[in] max_devices Maximum number of ROM codes to find.
 * @return Number of ROM codes found, or zero if no devices found.
 */
static int find_owb_rom_codes(const OneWireBus * owb, OneWireBus_ROMCode * rom_codes, int max_codes)
{
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = owb_search_first(owb, &search_state);

    ESP_LOGI(TAG, "Find devices:");
    while (found && num_devices <= max_codes)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGI(TAG, "  %d : %s", num_devices, rom_code_s);
        rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        found = owb_search_next(owb, &search_state);
    }
    return num_devices;
}

static void associate_ds18b20_devices(const OneWireBus * owb,
                                      const OneWireBus_ROMCode * rom_codes,
                                      DS18B20_Info ** device_infos,
                                      int num_devices)
{
    for (int i = 0; i < num_devices; ++i)
    {
        DS18B20_Info * ds18b20_info = ds18b20_malloc();
        device_infos[i] = ds18b20_info;

        if (num_devices == 1)
        {
            ESP_LOGD(TAG, "Single device optimisations enabled");
            ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
        }
        else
        {
            ds18b20_init(ds18b20_info, owb, rom_codes[i]); // associate with bus and device
        }
        ds18b20_use_crc(ds18b20_info, true);           // enable CRC check for temperature readings
        ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
    }
}

//TODO: LED task, to blink LED when required
// - count number of connected devices
// - indicate when sampling
// - indicate MQTT connection state
// - indicate MQTT activity (publish, receive)
void led_init(void)
{
    gpio_pad_select_gpio(GPIO_LED);
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
}

void led_on(void)
{
    gpio_set_level(GPIO_LED, 1);
}

void led_off(void)
{
    gpio_set_level(GPIO_LED, 0);
}

void led_indicate_num_devices(int num_devices)
{
    for (int i = 0; i < num_devices; ++i)
    {
        led_on();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_off();
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void read_temperatures(DS18B20_Info ** device_infos, float * readings, int num_devices)
{
    ds18b20_convert_all(device_infos[0]->bus);

    // in this application all devices use the same resolution,
    // so use the first device to determine the delay
    ds18b20_wait_for_conversion(device_infos[0]);

    // read the results immediately after conversion otherwise it may fail
    // (using printf before reading may take too long)
    for (int i = 0; i < num_devices; ++i)
    {
        readings[i] = ds18b20_read_temp(device_infos[i]);
    }
}

void sensor_task(void *pvParameter)
{
    SensorTaskInputs * inputs = (SensorTaskInputs *)pvParameter;
    int sample_count = 0;

    ESP_LOGW(TAG":sensor_task", "Core ID %d", xPortGetCoreID());

    int errors[MAX_DEVICES] = {0};
    if (inputs->sensors->num_ds18b20s > 0)
    {
        // wait a second before starting commencing sampling
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        while (1)
        {
            TickType_t start_ticks = xTaskGetTickCount();

            led_on();

            float readings[MAX_DEVICES] = { 0 };
            read_temperatures(inputs->sensors->ds18b20_infos, readings, inputs->sensors->num_ds18b20s);
            ++sample_count;

            led_off();

            // print results in a separate loop, after all have been read
            printf("\nTemperature readings (degrees C): sample %d\n", sample_count);
            for (int i = 0; i < inputs->sensors->num_ds18b20s; ++i)
            {
                // filter out invalid readings
                if (readings[i] != DS18B20_INVALID_READING)
                {
                    SensorReading sensor_reading = {
                        .sensor_id = i,
                        .value = readings[i],
                    };
                    BaseType_t status = xQueueSendToBack(inputs->publish_queue, &sensor_reading, 0);
                    if (status != pdPASS)
                    {
                        ESP_LOGE(TAG":sensor", "Could not send to queue");
                    }
                }
                else
                {
                    ++errors[i];
                }
                printf("  %d: %.1f    %d errors\n", i, readings[i], errors[i]);
            }

            // make up delay to approximate sample period
            vTaskDelay(SAMPLE_PERIOD / portTICK_PERIOD_MS - (xTaskGetTickCount() - start_ticks));
        }
    }
    else
    {
        ESP_LOGE(TAG, "No devices found");
    }

    vTaskDelete(NULL);
}

// publish sensor readings
void publish_task(void *pvParameter)
{
    QueueHandle_t publish_queue = (QueueHandle_t)pvParameter;
    ESP_LOGW(TAG":publish_task", "Core ID %d", xPortGetCoreID());

    while (1)
    {
        // Not necessarily true when using multiple cores:
        //if (uxQueueMessagesWaiting(publish_queue) != 0)
        //{
        //    ESP_LOGE(TAG":mqtt", "Queue should be empty!\n");
        //}

        SensorReading reading = {0};
        BaseType_t sensor_queue_status = xQueueReceive(publish_queue, &reading, portMAX_DELAY);
        if (sensor_queue_status == pdPASS)
        {
            // TODO: we need a more reliable way to know that the client is valid,
            // as attempting to publish when the client has died causes a crash.
            if (client != NULL)
            {
                ESP_LOGD(TAG":publish_task", "Received %d:%f", reading.sensor_id, reading.value);
                char topic[64] = {0};
                char value[8] = {0};
                snprintf(topic, 64-1, "poolmon/sensor/%d", reading.sensor_id);
                snprintf(value, 8-1, "%.3f", reading.value);
                ESP_LOGI(TAG":publish_task", "Publish %s %s", topic, value);
                mqtt_publish(client, topic, value, strlen(value), 0, 0);
            }
            else
            {
                ESP_LOGW(TAG":publish_task", "MQTT not ready - throw away");
            }
        }
        else
        {
            ESP_LOGE(TAG":publish_task", "Could not receive from queue");
        }
    }

    vTaskDelete(NULL);
}

void connected_cb(void *self, void *params)
{
    mqtt_client *client = (mqtt_client *)self;

    // send a device status update
    const char * value = "MQTT connected";
    mqtt_publish(client, "poolmon/device/status", value, strlen(value), 0, 0);

    // let's subscribe to poolmon/device/control
    mqtt_subscribe(client, "poolmon/device/control/#", 0);
}

void disconnected_cb(void *self, void *params)
{
}

void reconnect_cb(void *self, void *params)
{
    mqtt_client *client = (mqtt_client *)self;

    // send a device status update
    const char * value = "MQTT reconnected";
    mqtt_publish(client, "poolmon/device/status", value, strlen(value), 0, 0);
}

void subscribe_cb(void *self, void *params)
{
//    ESP_LOGI(TAG":mqtt", "[APP] Subscribe ok, test publish msg");
//    mqtt_client *client = (mqtt_client *)self;
//    mqtt_publish(client, "/espda", "abcde", 5, 0, 0);
}

void publish_cb(void *self, void *params)
{
    ESP_LOGI(TAG":mqtt", "[APP] Publish CB");
}

void data_cb(void *self, void *params)
{
    //mqtt_client *client = (mqtt_client *)self;
    mqtt_event_data_t *event_data = (mqtt_event_data_t *)params;

    if (event_data->data_offset == 0) {
        char *topic = malloc(event_data->topic_length + 1);
        memcpy(topic, event_data->topic, event_data->topic_length);
        topic[event_data->topic_length] = 0;
        ESP_LOGI(TAG":mqtt", "[APP] Publish topic: %s", topic);

        char *data = malloc(event_data->data_length + 1);
        memcpy(data, event_data->data, event_data->data_length);
        data[event_data->data_length] = 0;

        // data is null-terminated so can be treated like a string if required

        ESP_LOGI(TAG":mqtt", "[APP] Publish data[%d/%d bytes]",
                 event_data->data_length + event_data->data_offset,
                 event_data->data_total_length);
        esp_log_buffer_hex(TAG":mqtt", data, event_data->data_length + 1);

        // Reboot command:
        if (strcmp(topic, "poolmon/device/control/reboot") == 0)
        {
            int count = atoi(data);

            // TODO: to do this properly, start a "reboot" task
            // That way a reboot can be reissued or cancelled,
            // and communication with the device is not lost.

            while (count > 0)
            {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                ESP_LOGW(TAG, "[APP] Rebooting in %d seconds", count);
                --count;
            }

            const char * value = "Rebooting";
            mqtt_publish(client, "poolmon/device/status", value, strlen(value), 0, 0);

            // wait another second or so for the MQTT message to go
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            fflush(stdout);
            esp_restart();
        }

        // TODO: control values:
        //  - proper reboot scheduling
        //  - sensor polling period (for all sensors)
        //  - sensor resolution (per sensor)
        //  - MQTT host name
        //  - MQTT keep-alive duration
        //  - log level

        // TODO: status values:
        //  - uptime
        //  - number of sensors detected
        //  - sensor errors (for each)
        //  - sensor stats (min, max)
        //

        free(topic);
        free(data);
    }
    else
    {
        // TODO: how do we deal with this? When does it occur?
        ESP_LOGW(TAG":mqtt", "event_data->data_offset is not zero: %d", event_data->data_offset);
    }
}

mqtt_settings settings = {
    .host = "rpi3.fritz.box",
#if defined(CONFIG_MQTT_SECURITY_ON)
    .port = 8883, // encrypted
#else
    .port = 1883, // unencrypted
#endif
    .client_id = "mqtt_client_id",
    .username = "user",
    .password = "pass",
    .clean_session = 0,
    .keepalive = 120,
    .lwt_topic = "/lwt",
    .lwt_msg = "offline",
    .lwt_qos = 0,
    .lwt_retain = 0,
    .connected_cb = connected_cb,
    .disconnected_cb = disconnected_cb,
    //.reconnect_cb = reconnect_cb,
    .subscribe_cb = subscribe_cb,
    .publish_cb = publish_cb,
    .data_cb = data_cb
};



static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

            //init app here
            client = mqtt_start(&settings);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
               auto-reassociate. */
            esp_wifi_connect();
            mqtt_stop();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            client = NULL;
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_conn_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG":wifi", "start the WIFI SSID:[%s] password:[%s]", CONFIG_WIFI_SSID, "******");
    ESP_ERROR_CHECK(esp_wifi_start());
}

Sensors detect_sensors(void)
{
    // set up the One Wire Bus
    OneWireBus * owb = owb_malloc();
    owb_init(owb, GPIO_ONE_WIRE);
    owb_use_crc(owb, true);       // enable CRC check for ROM code

    // locate attached devices
    OneWireBus_ROMCode * device_rom_codes = calloc(MAX_DEVICES, sizeof(*device_rom_codes));

    int num_devices = find_owb_rom_codes(owb, device_rom_codes, MAX_DEVICES);

    // free up unused space
    device_rom_codes = realloc(device_rom_codes, num_devices * sizeof(*device_rom_codes));

    // associate devices on bus with DS18B20 device driver
    DS18B20_Info ** device_infos = calloc(num_devices, sizeof(*device_infos));
    associate_ds18b20_devices(owb, device_rom_codes, device_infos, num_devices);

    Sensors sensors = {
        .owb = owb,
        .rom_codes = device_rom_codes,
        .ds18b20_infos = device_infos,
        .num_ds18b20s = num_devices,
    };
    return sensors;
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_LOGI(TAG, "[APP] Startup..");
    led_init();

    // It works best to find all connected devices before starting WiFi, otherwise it can be unreliable.
    // Assume that new devices are not connected during operation.
    Sensors sensors = detect_sensors();

    // Blink the LED to indicate the number of devices detected:
    led_indicate_num_devices(sensors.num_ds18b20s);

    // Create a queue for the sensor task to publish sensor readings
    // (Priority of sending task should be lower than receiving task)
    QueueHandle_t publish_queue = xQueueCreate(PUBLISH_QUEUE_DEPTH, sizeof(SensorReading));

    SensorTaskInputs sensor_task_inputs = {
        .sensors = &sensors,
        .publish_queue = publish_queue,
    };
    xTaskCreate(&sensor_task, "sensor_task", 4096, &sensor_task_inputs, CONFIG_MQTT_PRIORITY - 1, NULL);
    xTaskCreate(&publish_task, "publish_task", 4096, publish_queue, CONFIG_MQTT_PRIORITY, NULL);

//    nvs_flash_init();
    wifi_conn_init();

    // Run forever...
    while(1)
        ;

    // clean up dynamically allocated data
    free(sensors.rom_codes);
    for (int i = 0; i < sensors.num_ds18b20s; ++i)
    {
        ds18b20_free(&(sensors.ds18b20_infos[i]));
    }
    free(sensors.ds18b20_infos);
    owb_free(&(sensors.owb));
}
