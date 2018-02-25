/*
 * MIT License
 *
 * Copyright (c) 2017-2018 David Antliff
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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "soc/rtc.h"

// memory monitoring
#include "esp_system.h"
#include "esp_heap_caps.h"

#include "constants.h"
#include "resources.h"
#include "utils.h"
#include "led.h"
#include "i2c_master.h"
#include "sensor_temp.h"
#include "sensor_flow.h"
#include "sensor_light.h"
#include "publish.h"
#include "wifi_support.h"
#include "mqtt.h"
#include "avr_support.h"
#include "display.h"
#include "power.h"
#include "datastore/datastore.h"

#define TAG "app_main"

//TODO: LED task, to blink LED when required
// - count number of connected devices
// - indicate when sampling
// - indicate MQTT connection state
// - indicate MQTT activity (publish, receive)

static void do_esp32_reset(const char * topic, bool value, void * context)
{
    if (value)
    {
        ESP_LOGW(TAG, "ESP32 restart requested");
        bool * running = (bool *)context;
        *running = false;
    }
}

static void do_avr_reset(const char * topic, bool value, void * context)
{
    if (value)
    {
        ESP_LOGW(TAG, "AVR restart requested");
        avr_support_reset();

    }
}

static void do_avr_cp(const char * topic, bool value, void * context)
{
    avr_support_set_cp_pump(value ? AVR_PUMP_STATE_ON : AVR_PUMP_STATE_OFF);
}

static void do_avr_pp(const char * topic, bool value, void * context)
{
    avr_support_set_pp_pump(value ? AVR_PUMP_STATE_ON : AVR_PUMP_STATE_OFF);
}

static void do_avr_alarm(const char * topic, bool value, void * context)
{
    avr_support_set_alarm(value ? AVR_ALARM_STATE_ON : AVR_ALARM_STATE_OFF);
}

static void do_datastore_dump(const char * topic, bool value, void * context)
{
    if (value)
    {
        if (context != NULL)
        {
            const datastore_t * datastore = (const datastore_t *)context;
            datastore_dump(datastore);
        }
    }
}

static void echo_bool(const char * topic, bool value, void * context)
{
    int ctxt_val = *(int *)context;
    ESP_LOGW(TAG, "echo_bool: context %d, topic %s, value %d", ctxt_val, topic, value);
}

static void echo_uint8(const char * topic, uint8_t value, void * context)
{
    int ctxt_val = *(int *)context;
    ESP_LOGW(TAG, "echo_uint8: context %d, topic %s, value %d", ctxt_val, topic, value);
}

static void echo_uint32(const char * topic, uint32_t value, void * context)
{
    int ctxt_val = *(int *)context;
    ESP_LOGW(TAG, "echo_uint32: context %d, topic %s, value %u", ctxt_val, topic, value);
}

static void echo_int8(const char * topic, int8_t value, void * context)
{
    int ctxt_val = *(int *)context;
    ESP_LOGW(TAG, "echo_int8: context %d, topic %s, value %d", ctxt_val, topic, value);
}

static void echo_int32(const char * topic, int32_t value, void * context)
{
    int ctxt_val = *(int *)context;
    ESP_LOGW(TAG, "echo_int32: context %d, topic %s, value %d", ctxt_val, topic, value);
}

static void echo_float(const char * topic, float value, void * context)
{
    int ctxt_val = *(int *)context;
    ESP_LOGW(TAG, "echo_float: context %d, topic %s, value %.7f", ctxt_val, topic, value);
}

static void echo_double(const char * topic, double value, void * context)
{
    int ctxt_val = *(int *)context;
    ESP_LOGW(TAG, "echo_double: context %d, topic %s, value %.17f", ctxt_val, topic, value);
}

static void echo_string(const char * topic, const char * value, void * context)
{
    int ctxt_val = *(int *)context;
    ESP_LOGW(TAG, "echo_string: context %d, topic %s, value \'%s\'", ctxt_val, topic, value);
    esp_log_buffer_hex(TAG, value, strlen(value) + 1);
}

// brief delay during startup sequence
static void _delay(void)
{
    vTaskDelay(100 / portTICK_RATE_MS);
}



static void avr_test_sequence(void)
{
    static int state = 0;

    // simple state machine for testing
    switch (state)
    {
    case 0:
        avr_support_set_cp_pump(AVR_PUMP_STATE_OFF);
        avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
        avr_support_set_alarm(AVR_ALARM_STATE_OFF);
        break;
    case 1:
        avr_support_set_alarm(AVR_ALARM_STATE_ON);
        break;
    case 2:
        avr_support_set_cp_pump(AVR_PUMP_STATE_ON);
        break;
    case 3:
        avr_support_set_cp_pump(AVR_PUMP_STATE_OFF);
        avr_support_set_pp_pump(AVR_PUMP_STATE_ON);
        break;
    case 4:
        avr_support_set_cp_pump(AVR_PUMP_STATE_ON);
        avr_support_set_alarm(AVR_ALARM_STATE_OFF);
        break;
    case 5:
        avr_support_set_cp_pump(AVR_PUMP_STATE_OFF);
        break;
    case 6:
        avr_support_set_cp_pump(AVR_PUMP_STATE_ON);
        avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
        break;
    default:
        state = 0;
        break;
    }
    state = (state + 1) % 7;
}

void init_publish_subscriptions(const datastore_t * datastore, QueueHandle_t publish_queue)
{
    if (datastore)
    {
        publish_context_t * publish_context = malloc(sizeof(*publish_context));
        memset(publish_context, 0, sizeof(*publish_context));
        publish_context->queue = publish_queue;
        // TODO: Who will free this later?

        resource_id_t resources[] = {
            RESOURCE_ID_TEMP_VALUE,
            RESOURCE_ID_LIGHT_FULL,
            RESOURCE_ID_LIGHT_VISIBLE,
            RESOURCE_ID_LIGHT_INFRARED,
            RESOURCE_ID_LIGHT_ILLUMINANCE,
            RESOURCE_ID_FLOW_FREQUENCY,
            RESOURCE_ID_FLOW_RATE,
            RESOURCE_ID_SWITCHES_CP_MODE_VALUE,
            RESOURCE_ID_SWITCHES_CP_MAN_VALUE,
            RESOURCE_ID_SWITCHES_PP_MODE_VALUE,
            RESOURCE_ID_SWITCHES_PP_MAN_VALUE,
            RESOURCE_ID_PUMPS_CP_STATE,
            RESOURCE_ID_PUMPS_PP_STATE,
        };

        for (size_t i = 0; i < sizeof(resources) / sizeof(resources[0]); ++i)
        {
            datastore_status_t status;
            if ((status = datastore_add_set_callback(datastore, resources[i], publish_callback, publish_context)) != DATASTORE_STATUS_OK)
            {
                ESP_LOGE(TAG, "datastore_add_set_callback for resource %d failed: %d", resources[i], status);
            }
        }
    }
}

void app_main()
{
    init_boot_time_reference();

    esp_log_level_set("*", ESP_LOG_WARN);
//    esp_log_level_set("*", ESP_LOG_INFO);
//    esp_log_level_set("*", ESP_LOG_DEBUG);
//   esp_log_level_set("display", ESP_LOG_DEBUG);
//    esp_log_level_set("avr_support", ESP_LOG_DEBUG);
    esp_log_level_set("wifi_support", ESP_LOG_INFO);
    esp_log_level_set("datastore", ESP_LOG_INFO);
    esp_log_level_set("mqtt", ESP_LOG_INFO);
    esp_log_level_set("publish", ESP_LOG_DEBUG);
//    esp_log_level_set("sensor_temp", ESP_LOG_INFO);
    esp_log_level_set("i2c-lcd1602", ESP_LOG_INFO);   // debug is too verbose

    // Priority of queue consumer should be higher than producers
    UBaseType_t publish_priority = CONFIG_ESP_MQTT_TASK_STACK_PRIORITY;
    UBaseType_t display_priority = publish_priority - 1;
    UBaseType_t sensor_priority = publish_priority - 1;
    UBaseType_t avr_priority = sensor_priority;
    UBaseType_t wifi_monitor_priority = sensor_priority;

    ESP_LOGI(TAG, "Start");

    // round to nearest MHz (stored value is only precise to MHz)
    uint32_t apb_freq = (rtc_clk_apb_freq_get() + 500000) / 1000000 * 1000000;
    ESP_LOGI(TAG, "APB CLK %u Hz", apb_freq);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    datastore_t * datastore = resources_init();
    resources_load(datastore);
    datastore_dump(datastore);

    // Onboard LED
    led_init(CONFIG_ONBOARD_LED_GPIO);


    // I2C bus
    ESP_LOGW(TAG, "about to run i2c_master_init");
    _delay();
    i2c_master_info_t * i2c_master_info = i2c_master_init(I2C_MASTER_NUM, CONFIG_I2C_MASTER_SDA_GPIO, CONFIG_I2C_MASTER_SCL_GPIO, I2C_MASTER_FREQ_HZ);

    ESP_LOGW(TAG, "about to run i2c_master_scan");
    _delay();
    int num_i2c_devices = i2c_master_scan(i2c_master_info);
    ESP_LOGI(TAG, "%d I2C devices detected", num_i2c_devices);

    // bring up the display ASAP in case of error
    _delay();
    display_init(i2c_master_info, display_priority, datastore);


    // It works best to find all connected devices before starting WiFi, otherwise it can be unreliable.

    // Temp sensors
    _delay();
    temp_sensors_t * temp_sensors = sensor_temp_init(CONFIG_ONE_WIRE_GPIO, sensor_priority, datastore);

    // I2C devices - AVR, Light Sensor, LCD
    _delay();
    avr_support_init(i2c_master_info, avr_priority, datastore);
    avr_support_reset();

    _delay();
    sensor_light_init(i2c_master_info, sensor_priority, datastore);


    // Flow Meter
    _delay();
    sensor_flow_init(CONFIG_FLOW_METER_PULSE_GPIO, FLOW_METER_PCNT_UNIT, FLOW_METER_PCNT_CHANNEL,
                     CONFIG_FLOW_METER_RMT_GPIO, FLOW_METER_RMT_CHANNEL, FLOW_METER_RMT_CLK_DIV,
                     FLOW_METER_SAMPLING_PERIOD, FLOW_METER_SAMPLING_WINDOW, FLOW_METER_FILTER_LENGTH, sensor_priority, datastore);

    bool running = true;

    mqtt_info_t * mqtt_info = mqtt_malloc();
    mqtt_error_t mqtt_error = MQTT_ERROR_UNKNOWN;
    if ((mqtt_error = mqtt_init(mqtt_info, datastore)) == MQTT_OK)
    {
        // careful, stack variables!
        int context_echo_bool = 1;
        int context_echo_uint8 = 2;
        int context_echo_uint32 = 3;
        int context_echo_int8 = 4;
        int context_echo_int32 = 5;
        int context_echo_float = 6;
        int context_echo_double = 7;
        int context_echo_string = 8;

        if ((mqtt_error = mqtt_register_topic_as_bool(mqtt_info, "poolmon/echo/bool", &echo_bool, &context_echo_bool)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_uint8(mqtt_info, "poolmon/echo/uint8", &echo_uint8, &context_echo_uint8)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_uint8 failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_uint32(mqtt_info, "poolmon/echo/uint32", &echo_uint32, &context_echo_uint32)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_uint32 failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_int8(mqtt_info, "poolmon/echo/int8", &echo_int8, &context_echo_int8)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_int8 failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_int32(mqtt_info, "poolmon/echo/int32", &echo_int32, &context_echo_int32)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_int32 failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_float(mqtt_info, "poolmon/echo/float", &echo_float, &context_echo_float)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_float failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_double(mqtt_info, "poolmon/echo/double", &echo_double, &context_echo_double)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_double failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_string(mqtt_info, "poolmon/echo/string", &echo_string, &context_echo_string)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_string failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_bool(mqtt_info, "poolmon/esp32/reset", &do_esp32_reset, &running)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_bool(mqtt_info, "poolmon/avr/reset", &do_avr_reset, NULL)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_bool(mqtt_info, "poolmon/avr/cp", &do_avr_cp, NULL)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_bool(mqtt_info, "poolmon/avr/pp", &do_avr_pp, NULL)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_bool(mqtt_info, "poolmon/avr/alarm", &do_avr_alarm, NULL)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
        }

        if ((mqtt_error = mqtt_register_topic_as_bool(mqtt_info, "poolmon/datastore/dump", &do_datastore_dump, (void *)datastore)) != MQTT_OK)
        {
            ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
        }
    }
    else
    {
        ESP_LOGE(TAG, "mqtt_init failed: %d", mqtt_error);
    }

    _delay();
    nvs_flash_init();
    wifi_support_init(wifi_monitor_priority, datastore);

    _delay();
    QueueHandle_t publish_queue = publish_init(mqtt_info, PUBLISH_QUEUE_DEPTH, publish_priority);
    init_publish_subscriptions(datastore, publish_queue);

    _delay();
    power_init(sensor_priority, datastore);

    _delay();
    datastore_dump(datastore);

    TickType_t last_wake_time = xTaskGetTickCount();

    while (running)
    {
        last_wake_time = xTaskGetTickCount();

        if (seconds_since_boot() % 10 == 0)
        {
            ESP_LOGI(TAG, "RAM left %d", esp_get_free_heap_size());  // byte-addressable heap memory
            ESP_LOGI(TAG, "32bit aligned RAM left %d", heap_caps_get_free_size(MALLOC_CAP_32BIT));  // IRAM 32-bit aligned heap
            ESP_LOGI(TAG, "uptime %d seconds", seconds_since_boot());
        }

        avr_test_sequence();

        vTaskDelayUntil(&last_wake_time, 1000 / portTICK_RATE_MS);
    }

    // TODO: signal to all tasks to close and wait for them to do so
    // TODO: before deallocating structures that they might be using.

//    sensor_temp_close(temp_sensors);
//    i2c_master_close(i2c_master_info);
    datastore_free(&datastore);

    ESP_LOGE(TAG, "Restarting...");
    vTaskDelay(1000 / portTICK_RATE_MS);
    esp_restart();
}
