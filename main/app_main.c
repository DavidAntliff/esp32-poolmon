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
#include "nvs_support.h"
#include "wifi_support.h"
#include "mqtt.h"
#include "avr_support.h"
#include "display.h"
#include "power.h"
#include "control.h"
#include "system_monitor.h"
#include "sntp_rtc.h"
#include "datastore/datastore.h"

#define TAG "app_main"

#define MARK_PERIOD (60 * 10)  // emit something in the log every 10 minutes

//TODO: LED task, to blink LED when required
// - count number of connected devices
// - indicate when sampling
// - indicate MQTT connection state
// - indicate MQTT activity (publish, receive)

static void do_esp32_reset(const char * topic, bool value, void * context)
{
    if (value)
    {
        ESP_LOGW(TAG, "ESP32 reset requested");
        bool * running = (bool *)context;
        *running = false;
    }
}

static void do_avr_reset(const char * topic, bool value, void * context)
{
    if (value)
    {
        ESP_LOGW(TAG, "AVR reset requested");
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
    if (value && context)
    {
        const datastore_t * datastore = (const datastore_t *)context;
        datastore_dump(datastore);
    }
}

static void do_datastore_save(const char * topic, bool value, void * context)
{
    if (value && context)
    {
        const datastore_t * datastore = (const datastore_t *)context;
        resources_save(datastore);
    }
}

static void do_datastore_load(const char * topic, bool value, void * context)
{
    if (value && context)
    {
        const datastore_t * datastore = (const datastore_t *)context;
        resources_load(datastore);
    }
}

static void do_nvs_erase_all(const char * topic, bool value, void * context)
{
    if (value)
    {
        ESP_LOGW(TAG, "Erasing all resources from NVS");
        nvs_support_erase_all(NVS_NAMESPACE_RESOURCES);
    }
}

static void do_nvs_erase(const char * topic, const char * description, void * context)
{
    if (description && context)
    {
        ESP_LOGW(TAG, "Erasing resource %s from NVS", description);
        const datastore_t * datastore = (const datastore_t *)context;
        resources_erase(datastore, description);
    }
}

static void do_sensors_temp_label(const char * topic, const char * value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    uint32_t instance = 0;
    sscanf(topic, ROOT_TOPIC"/sensors/temp/%u/label", &instance);
    ESP_LOGD(TAG, "instance %u, value %s", instance, value);
    if (instance > 0 && instance <= SENSOR_TEMP_INSTANCES)
    {
        datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, instance - 1, value);
    }
}

static void do_sensors_temp_assignment(const char * topic, const char * value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    uint32_t instance = 0;
    sscanf(topic, ROOT_TOPIC"/sensors/temp/%u/assignment", &instance);
    ESP_LOGD(TAG, "instance %u, value %s", instance, value);
    if (instance > 0 && instance <= SENSOR_TEMP_INSTANCES)
    {
        datastore_set_string(datastore, RESOURCE_ID_TEMP_ASSIGNMENT, instance - 1, value);
    }
}

static void do_sensors_temp_override(const char * topic, float value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    uint32_t instance = 0;
    sscanf(topic, ROOT_TOPIC"/sensors/temp/%u/override", &instance);
    ESP_LOGD(TAG, "instance %u, value %f", instance, value);
    if (instance > 0 && instance <= SENSOR_TEMP_INSTANCES)
    {
        datastore_set_float(datastore, RESOURCE_ID_TEMP_OVERRIDE, instance - 1, value);
    }
}

static void do_sensors_flow_override(const char * topic, float value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    uint32_t instance = 0;
    sscanf(topic, ROOT_TOPIC"/sensors/flow/%u/override", &instance);
    ESP_LOGD(TAG, "instance %u, value %f", instance, value);
    if (instance == 1)
    {
        datastore_set_float(datastore, RESOURCE_ID_FLOW_RATE_OVERRIDE, instance - 1, value);
    }
}

static void do_control_cp_delta_on(const char * topic, float value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_float(datastore, RESOURCE_ID_CONTROL_CP_ON_DELTA, 0, value);
}

static void do_control_cp_delta_off(const char * topic, float value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_float(datastore, RESOURCE_ID_CONTROL_CP_OFF_DELTA, 0, value);
}

static void do_control_flow_threshold(const char * topic, float value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_float(datastore, RESOURCE_ID_CONTROL_FLOW_THRESHOLD, 0, value);
}

static void do_control_pp_cycle_count(const char * topic, uint32_t value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_COUNT, 0, value);
}

static void do_control_pp_cycle_on_duration(const char * topic, uint32_t value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_ON_DURATION, 0, value);
}

static void do_control_pp_cycle_pause_duration(const char * topic, uint32_t value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_PAUSE_DURATION, 0, value);
}

static void do_control_pp_daily_hour(const char * topic, int32_t value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_int32(datastore, RESOURCE_ID_CONTROL_PP_DAILY_HOUR, 0, value);
}

static void do_control_pp_daily_minute(const char * topic, int32_t value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_int32(datastore, RESOURCE_ID_CONTROL_PP_DAILY_MINUTE, 0, value);
}

static void do_display_backlight_timeout(const char * topic, uint32_t value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_uint32(datastore, RESOURCE_ID_DISPLAY_BACKLIGHT_TIMEOUT, 0, value);
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

void init_publish_subscriptions(const datastore_t * datastore, publish_context_t * publish_context)
{
    if (datastore)
    {
        resource_id_t resources[] = {
            RESOURCE_ID_TEMP_VALUE,
            RESOURCE_ID_LIGHT_FULL,
            RESOURCE_ID_LIGHT_VISIBLE,
            RESOURCE_ID_LIGHT_INFRARED,
            RESOURCE_ID_LIGHT_ILLUMINANCE,
            RESOURCE_ID_FLOW_FREQUENCY,
            RESOURCE_ID_FLOW_RATE,
            RESOURCE_ID_POWER_VALUE,
            RESOURCE_ID_POWER_TEMP_DELTA,
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

typedef struct
{
    mqtt_info_t * mqtt_info;
    bool * running;
    datastore_t * datastore;
    publish_context_t * publish_context;
} globals_t;

void mqtt_status_callback(const datastore_t * datastore, datastore_resource_id_t id, datastore_instance_id_t instance, void * context)
{
    ESP_LOGD(TAG, "mqtt_status_callback");
    globals_t * globals = (globals_t *)context;
    mqtt_error_t mqtt_error = MQTT_ERROR_UNKNOWN;

    mqtt_status_t mqtt_status = 0;
    if (datastore_get_uint32(globals->datastore, RESOURCE_ID_MQTT_STATUS, 0, &mqtt_status) != DATASTORE_STATUS_OK)
    {
        ESP_LOGE(TAG, "datastore get error");
    }
    else
    {
        if (mqtt_status == MQTT_STATUS_CONNECTED)
        {
            // send some useful data
            publish_resource(globals->publish_context, globals->datastore, RESOURCE_ID_WIFI_ADDRESS, 0);
            for (size_t i = 0; i < SENSOR_TEMP_INSTANCES; ++i)
            {
                // store assignment of detected sensors to instances in NV
                publish_resource(globals->publish_context, globals->datastore, RESOURCE_ID_TEMP_ASSIGNMENT, i);
                publish_resource(globals->publish_context, globals->datastore, RESOURCE_ID_TEMP_DETECTED, i);
            }

            // subscribe to some topics
            if ((mqtt_error = mqtt_register_topic_as_bool(globals->mqtt_info, ROOT_TOPIC"/esp32/reset", &do_esp32_reset, (void *)globals->running)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_bool(globals->mqtt_info, ROOT_TOPIC"/avr/reset", &do_avr_reset, NULL)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_bool(globals->mqtt_info, ROOT_TOPIC"/avr/cp", &do_avr_cp, NULL)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_bool(globals->mqtt_info, ROOT_TOPIC"/avr/pp", &do_avr_pp, NULL)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_bool(globals->mqtt_info, ROOT_TOPIC"/avr/alarm", &do_avr_alarm, NULL)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_bool(globals->mqtt_info, ROOT_TOPIC"/datastore/dump", &do_datastore_dump, (void *)globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_bool(globals->mqtt_info, ROOT_TOPIC"/datastore/save", &do_datastore_save, (void *)globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_bool(globals->mqtt_info, ROOT_TOPIC"/datastore/load", &do_datastore_load, (void *)globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_string(globals->mqtt_info, ROOT_TOPIC"/datastore/erase", &do_nvs_erase, (void *)globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_string failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_bool(globals->mqtt_info, ROOT_TOPIC"/datastore/erase_all", &do_nvs_erase_all, NULL)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
            }

            // temp sensor labels and device assignments
            for (size_t i = 0; i < SENSOR_TEMP_INSTANCES; ++i)
            {
                char topic[64] = "";
                snprintf(topic, 64, ROOT_TOPIC"/sensors/temp/%d/label", i + 1);
                if ((mqtt_error = mqtt_register_topic_as_string(globals->mqtt_info, topic, &do_sensors_temp_label, globals->datastore)) != MQTT_OK)
                {
                    ESP_LOGE(TAG, "mqtt_register_topic_as_string failed: %d", mqtt_error);
                }

                snprintf(topic, 64, ROOT_TOPIC"/sensors/temp/%d/assignment", i + 1);
                if ((mqtt_error = mqtt_register_topic_as_string(globals->mqtt_info, topic, &do_sensors_temp_assignment, globals->datastore)) != MQTT_OK)
                {
                    ESP_LOGE(TAG, "mqtt_register_topic_as_string failed: %d", mqtt_error);
                }

                // debug override of temperature values - once set, only a reset will clear
                snprintf(topic, 64, ROOT_TOPIC"/sensors/temp/%d/override", i + 1);
                if ((mqtt_error = mqtt_register_topic_as_float(globals->mqtt_info, topic, &do_sensors_temp_override, globals->datastore)) != MQTT_OK)
                {
                    ESP_LOGE(TAG, "mqtt_register_topic_as_float failed: %d", mqtt_error);
                }
            }

            // debug override of flow value - once set, only a reset will clear
            char topic[64] = "";
            snprintf(topic, 64, ROOT_TOPIC"/sensors/flow/1/override");
            if ((mqtt_error = mqtt_register_topic_as_float(globals->mqtt_info, topic, &do_sensors_flow_override, globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_float failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_float(globals->mqtt_info, ROOT_TOPIC"/control/cp/delta_on", &do_control_cp_delta_on, globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_float failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_float(globals->mqtt_info, ROOT_TOPIC"/control/cp/delta_off", &do_control_cp_delta_off, globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_float failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_float(globals->mqtt_info, ROOT_TOPIC"/control/flow/threshold", &do_control_flow_threshold, globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_float failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_uint32(globals->mqtt_info, ROOT_TOPIC"/control/pp/cycle/count", &do_control_pp_cycle_count, globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_uint32 failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_uint32(globals->mqtt_info, ROOT_TOPIC"/control/pp/cycle/on_duration", &do_control_pp_cycle_on_duration, globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_uint32 failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_uint32(globals->mqtt_info, ROOT_TOPIC"/control/pp/cycle/pause_duration", &do_control_pp_cycle_pause_duration, globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_uint32 failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_int32(globals->mqtt_info, ROOT_TOPIC"/control/pp/daily/hour", &do_control_pp_daily_hour, globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_int32 failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_int32(globals->mqtt_info, ROOT_TOPIC"/control/pp/daily/minute", &do_control_pp_daily_minute, globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_int32 failed: %d", mqtt_error);
            }

            if ((mqtt_error = mqtt_register_topic_as_uint32(globals->mqtt_info, ROOT_TOPIC"/display/backlight/timeout", &do_display_backlight_timeout, globals->datastore)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_uint32 failed: %d", mqtt_error);
            }
        }
    }
}


void app_main()
{
    esp_log_level_set("*", ESP_LOG_WARN);
    //esp_log_level_set("*", ESP_LOG_INFO);
    //esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set("nvs_support", ESP_LOG_INFO);
    esp_log_level_set("display", ESP_LOG_WARN);
    esp_log_level_set("resources", ESP_LOG_INFO);
    esp_log_level_set("i2c", ESP_LOG_INFO);
    esp_log_level_set("avr_support", ESP_LOG_INFO);
    esp_log_level_set("wifi_support", ESP_LOG_INFO);
    esp_log_level_set("datastore", ESP_LOG_INFO);
    esp_log_level_set("mqtt", ESP_LOG_INFO);
    esp_log_level_set("publish", ESP_LOG_INFO);
    esp_log_level_set("sensor_temp", ESP_LOG_WARN);
    esp_log_level_set("sensor_light", ESP_LOG_WARN);
    esp_log_level_set("i2c-lcd1602", ESP_LOG_INFO);   // debug is too verbose
    esp_log_level_set("control", ESP_LOG_INFO);
    esp_log_level_set("sntp_rtc", ESP_LOG_INFO);
    esp_log_level_set("power", ESP_LOG_WARN);
    esp_log_level_set("app_main", ESP_LOG_INFO);

    // Ensure RMT peripheral is reset properly, in case of prior crash
    periph_module_disable(PERIPH_RMT_MODULE);
    periph_module_enable(PERIPH_RMT_MODULE);

    // Priority of queue consumer should be higher than producers
    UBaseType_t publish_priority = CONFIG_ESP_MQTT_TASK_STACK_PRIORITY;
    UBaseType_t display_priority = publish_priority - 1;
    UBaseType_t sensor_priority = publish_priority - 1;
    UBaseType_t avr_priority = sensor_priority;
    UBaseType_t wifi_monitor_priority = sensor_priority;
    UBaseType_t control_priority = sensor_priority;
    UBaseType_t system_priority = publish_priority;

    ESP_LOGI(TAG, "Start");

    // round to nearest MHz (stored value is only precise to MHz)
    uint32_t apb_freq = (rtc_clk_apb_freq_get() + 500000) / 1000000 * 1000000;
    ESP_LOGI(TAG, "APB CLK %u Hz", apb_freq);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    // initialise NVS
    ESP_ERROR_CHECK(nvs_support_init(NVS_NAMESPACE_RESOURCES));

    datastore_t * datastore = resources_init();
    resources_load(datastore);

    // Onboard LED
    led_init(CONFIG_ONBOARD_LED_GPIO);

    // I2C bus
    _delay();
    i2c_master_info_t * i2c_master_info = i2c_master_init(I2C_MASTER_NUM, CONFIG_I2C_MASTER_SDA_GPIO, CONFIG_I2C_MASTER_SCL_GPIO, I2C_MASTER_FREQ_HZ);

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
    if ((mqtt_error = mqtt_init(mqtt_info, datastore)) != MQTT_OK)
    {
        ESP_LOGE(TAG, "mqtt_init failed: %d", mqtt_error);
    }

    _delay();
    wifi_support_init(wifi_monitor_priority, datastore);   // requires NVS to be initialised

    _delay();
    publish_context_t * publish_context = publish_init(mqtt_info, PUBLISH_QUEUE_DEPTH, publish_priority, ROOT_TOPIC);
    init_publish_subscriptions(datastore, publish_context);

    _delay();
    power_init(sensor_priority, datastore);

    _delay();
    datastore_dump(datastore);

    TickType_t last_wake_time = xTaskGetTickCount();

    // be careful of the scope on this
    globals_t globals = {
        .mqtt_info = mqtt_info,
        .running = &running,
        .datastore = datastore,
        .publish_context = publish_context,
    };

    datastore_status_t status = datastore_add_set_callback(datastore, RESOURCE_ID_MQTT_STATUS, mqtt_status_callback, &globals);
    if (status != DATASTORE_STATUS_OK)
    {
        ESP_LOGE(TAG, "datastore_add_set_callback for resource %d failed: %d", RESOURCE_ID_MQTT_STATUS, status);
    }

    sntp_rtc_init(wifi_monitor_priority, datastore);
    _delay();

    control_init(control_priority, datastore);
    _delay();

    system_monitor_init(system_priority, datastore, publish_context);
    _delay();

    uint32_t last_mark_time = 0;

    while (running)
    {
        last_wake_time = xTaskGetTickCount();

        //avr_test_sequence();

        // period mark in log
        uint32_t now = seconds_since_boot();
        if (now >= last_mark_time + MARK_PERIOD)
        {
            ESP_LOGI(TAG, "-- mark --");
            last_mark_time = now;
        }

        // network connection state machine
        wifi_status_t wifi_status = WIFI_STATUS_DISCONNECTED;
        mqtt_status_t mqtt_status = MQTT_STATUS_DISCONNECTED;
        datastore_get_uint32(datastore, RESOURCE_ID_WIFI_STATUS, 0, &wifi_status);
        datastore_get_uint32(datastore, RESOURCE_ID_MQTT_STATUS, 0, &mqtt_status);
        ESP_LOGD(TAG, "wifi_status %d, mqtt_status %d", wifi_status, mqtt_status);

        if (wifi_status == WIFI_STATUS_DISCONNECTED)
        {
            if (mqtt_status != MQTT_STATUS_DISCONNECTED)
            {
                ESP_LOGI(TAG, "MQTT stop");
                esp_mqtt_stop();
            }
            ESP_LOGI(TAG, "WiFi connect");
            esp_wifi_connect();
        }
        else if (wifi_status == WIFI_STATUS_GOT_ADDRESS)
        {
            if (mqtt_status == MQTT_STATUS_DISCONNECTED)
            {
                ESP_LOGI(TAG, "MQTT start");
                if ((mqtt_error = mqtt_start(mqtt_info, datastore)) != MQTT_OK)
                {
                    ESP_LOGE(TAG, "mqtt_start failed: %d", mqtt_error);
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, 1000 / portTICK_RATE_MS);
    }

    // TODO: signal to all tasks to close and wait for them to do so
    // TODO: before deallocating structures that they might be using.

//    sensor_temp_close(temp_sensors);
//    i2c_master_close(i2c_master_info);
    datastore_free(&datastore);
    publish_free(&publish_context);

    ESP_LOGE(TAG, "Restarting...");
    vTaskDelay(1000 / portTICK_RATE_MS);
    esp_restart();
}
