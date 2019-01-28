/*
 * MIT License
 *
 * Copyright (c) 2018 David Antliff
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

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "subscriptions.h"
#include "constants.h"
#include "sensor_temp.h"
#include "avr_support.h"
#include "resources.h"
#include "nvs_support.h"

#define TAG "subscriptions"

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
        ESP_LOGW(TAG, "Saved resources to NVS");
    }
}

static void do_datastore_load(const char * topic, bool value, void * context)
{
    if (value && context)
    {
        const datastore_t * datastore = (const datastore_t *)context;
        resources_load(datastore);
        ESP_LOGW(TAG, "Loaded resources from NVS");
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
        ESP_LOGI(TAG, "Temp sensor %u assigned to device %s", instance, value);
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
        ESP_LOGW(TAG, "Temp override: instance %u set to %f", instance, value);
        datastore_set_float(datastore, RESOURCE_ID_TEMP_OVERRIDE, instance - 1, value);
    }
}

static void do_temp_period(const char * topic, uint32_t value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_uint32(datastore, RESOURCE_ID_TEMP_PERIOD, 0, value);
}

static void do_sensors_flow_override(const char * topic, float value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    uint32_t instance = 0;
    sscanf(topic, ROOT_TOPIC"/sensors/flow/%u/override", &instance);
    ESP_LOGD(TAG, "instance %u, value %f", instance, value);
    if (instance == 1)
    {
        ESP_LOGW(TAG, "Flow override: instance %u set to %f", instance, value);
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

static void do_control_safe_temp_high(const char * topic, float value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_float(datastore, RESOURCE_ID_CONTROL_SAFE_TEMP_HIGH, 0, value);
    ESP_LOGW(TAG, "Safe temperature high limit set to %f", value);
}

static void do_control_safe_temp_low(const char * topic, float value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_float(datastore, RESOURCE_ID_CONTROL_SAFE_TEMP_LOW, 0, value);
    ESP_LOGW(TAG, "Safe temperature low limit set to %f", value);
}

static void do_display_backlight_timeout(const char * topic, uint32_t value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    datastore_set_uint32(datastore, RESOURCE_ID_DISPLAY_BACKLIGHT_TIMEOUT, 0, value);
}

static void do_ota_url(const char * topic, const char * value, void * context)
{
    datastore_t * datastore = (datastore_t *)context;
    ESP_LOGI(TAG, "OTA URL: %s", value);
    datastore_set_string(datastore, RESOURCE_ID_OTA_URL, 0, value);
}

typedef struct
{
    const char * topic;
    mqtt_type_t type;
    mqtt_receive_callback_generic handler;
} subscribe_item_t;

static subscribe_item_t SUBSCRIPTIONS[] = {
//    { ROOT_TOPIC"/esp32/reset",                      MQTT_TYPE_BOOL,   (mqtt_receive_callback_generic)&do_esp32_reset },  -- doesn't take datastore as context
    { ROOT_TOPIC"/avr/reset",                        MQTT_TYPE_BOOL,   (mqtt_receive_callback_generic)&do_avr_reset },
    { ROOT_TOPIC"/avr/cp",                           MQTT_TYPE_BOOL,   (mqtt_receive_callback_generic)&do_avr_cp },
    { ROOT_TOPIC"/avr/pp",                           MQTT_TYPE_BOOL,   (mqtt_receive_callback_generic)&do_avr_pp },
    { ROOT_TOPIC"/avr/alarm",                        MQTT_TYPE_BOOL,   (mqtt_receive_callback_generic)&do_avr_alarm },
    { ROOT_TOPIC"/datastore/dump",                   MQTT_TYPE_BOOL,   (mqtt_receive_callback_generic)&do_datastore_dump },
    { ROOT_TOPIC"/datastore/save",                   MQTT_TYPE_BOOL,   (mqtt_receive_callback_generic)&do_datastore_save },
    { ROOT_TOPIC"/datastore/load",                   MQTT_TYPE_BOOL,   (mqtt_receive_callback_generic)&do_datastore_load },
    { ROOT_TOPIC"/datastore/erase",                  MQTT_TYPE_STRING, (mqtt_receive_callback_generic)&do_nvs_erase },
    { ROOT_TOPIC"/datastore/erase_all",              MQTT_TYPE_BOOL,   (mqtt_receive_callback_generic)&do_nvs_erase_all },
    { ROOT_TOPIC"/temp/period",                      MQTT_TYPE_UINT32, (mqtt_receive_callback_generic)&do_temp_period },
    { ROOT_TOPIC"/sensors/flow/1/override",          MQTT_TYPE_FLOAT,  (mqtt_receive_callback_generic)&do_sensors_flow_override },
    { ROOT_TOPIC"/control/cp/delta_on",              MQTT_TYPE_FLOAT,  (mqtt_receive_callback_generic)&do_control_cp_delta_on },
    { ROOT_TOPIC"/control/cp/delta_off",             MQTT_TYPE_FLOAT,  (mqtt_receive_callback_generic)&do_control_cp_delta_off },
    { ROOT_TOPIC"/control/flow/threshold",           MQTT_TYPE_FLOAT,  (mqtt_receive_callback_generic)&do_control_flow_threshold },
    { ROOT_TOPIC"/control/pp/cycle/count",           MQTT_TYPE_UINT32, (mqtt_receive_callback_generic)&do_control_pp_cycle_count },
    { ROOT_TOPIC"/control/pp/cycle/on_duration",     MQTT_TYPE_UINT32, (mqtt_receive_callback_generic)&do_control_pp_cycle_on_duration },
    { ROOT_TOPIC"/control/pp/cycle/pause_duration",  MQTT_TYPE_UINT32, (mqtt_receive_callback_generic)&do_control_pp_cycle_pause_duration },
    { ROOT_TOPIC"/control/pp/daily/hour",            MQTT_TYPE_INT32,  (mqtt_receive_callback_generic)&do_control_pp_daily_hour },
    { ROOT_TOPIC"/control/pp/daily/minute",          MQTT_TYPE_INT32,  (mqtt_receive_callback_generic)&do_control_pp_daily_minute },
    { ROOT_TOPIC"/control/safe/high",                MQTT_TYPE_FLOAT,  (mqtt_receive_callback_generic)&do_control_safe_temp_high },
    { ROOT_TOPIC"/control/safe/low",                 MQTT_TYPE_FLOAT,  (mqtt_receive_callback_generic)&do_control_safe_temp_low },
    { ROOT_TOPIC"/display/backlight/timeout",        MQTT_TYPE_UINT32, (mqtt_receive_callback_generic)&do_display_backlight_timeout },
    { ROOT_TOPIC"/ota/url",                          MQTT_TYPE_STRING, (mqtt_receive_callback_generic)&do_ota_url },
};

void subscriptions_init(const datastore_t * datastore, datastore_resource_id_t id, datastore_instance_id_t instance, void * context)
{
    ESP_LOGD(TAG, "mqtt_status_callback");
    subscriptions_context_t * globals = (subscriptions_context_t *)context;
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
                publish_resource(globals->publish_context, globals->datastore, RESOURCE_ID_TEMP_DETECTED, i);
            }

            if ((mqtt_error = mqtt_register_topic_as_bool(globals->mqtt_info, ROOT_TOPIC"/esp32/reset", &do_esp32_reset, globals->running)) != MQTT_OK)
            {
                ESP_LOGE(TAG, "mqtt_register_topic_as_bool failed: %d", mqtt_error);
            }

            // subscribe to some topics that accept global->datastore as context:
            for (size_t i = 0; i < sizeof(SUBSCRIPTIONS) / sizeof(SUBSCRIPTIONS[0]); ++i)
            {
                if ((mqtt_error = mqtt_register_topic(globals->mqtt_info, SUBSCRIPTIONS[i].topic, SUBSCRIPTIONS[i].handler, globals->datastore, SUBSCRIPTIONS[i].type)) != MQTT_OK)
                {
                    ESP_LOGE(TAG, "mqtt_register_topic failed: %d", mqtt_error);
                }
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
        }
    }
}
