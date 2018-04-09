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

#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "resources.h"
#include "constants.h"
#include "wifi_support.h"
#include "mqtt.h"
#include "sensor_temp.h"
#include "nvs_support.h"

#define TAG "resources"

#define ERROR_CHECK(x) do {                                                       \
        esp_err_t rc = (x);                                                       \
        if (rc != DATASTORE_STATUS_OK) {                                          \
            ESP_LOGW(TAG, "Datastore error %d at %s:%d", rc, __FILE__, __LINE__); \
        }                                                                         \
    } while(0);

static void _add_resource(const datastore_t * datastore, datastore_resource_id_t id, const char * name, datastore_resource_t resource)
{
    ERROR_CHECK(datastore_add_resource(datastore, id, resource));
    ERROR_CHECK(datastore_set_name(datastore, id, name));
}

datastore_t * resources_init(void)
{
    datastore_t * datastore = datastore_create();

    if (datastore)
    {
        _add_resource(datastore, RESOURCE_ID_SYSTEM_VERSION,         "SYSTEM_VERSION",         datastore_create_string_resource(SYSTEM_LEN_VERSION, 1));
        _add_resource(datastore, RESOURCE_ID_SYSTEM_BUILD_DATE_TIME, "SYSTEM_BUILD_DATE_TIME", datastore_create_string_resource(SYSTEM_LEN_BUILD_DATE_TIME, 1));
        _add_resource(datastore, RESOURCE_ID_SYSTEM_TIME_SET,        "SYSTEM_TIME_SET",        datastore_create_resource(DATASTORE_TYPE_BOOL, 1));

        _add_resource(datastore, RESOURCE_ID_WIFI_SSID,              "WIFI_SSID",              datastore_create_string_resource(WIFI_LEN_SSID, 1));
        _add_resource(datastore, RESOURCE_ID_WIFI_PASSWORD,          "WIFI_PASSWORD",          datastore_create_string_resource(WIFI_LEN_PASSWORD, 1));
        _add_resource(datastore, RESOURCE_ID_WIFI_STATUS,            "WIFI_STATUS",            datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_WIFI_TIMESTAMP,         "WIFI_TIMESTAMP",         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_WIFI_RSSI,              "WIFI_RSSI",              datastore_create_resource(DATASTORE_TYPE_INT8,   1));
        _add_resource(datastore, RESOURCE_ID_WIFI_ADDRESS,           "WIFI_ADDRESS",           datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_WIFI_CONNECTION_COUNT,  "WIFI_CONNECTION_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_MQTT_STATUS,            "MQTT_STATUS",            datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_TIMESTAMP,         "MQTT_TIMESTAMP",         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_BROKER_ADDRESS,    "MQTT_BROKER_ADDRESS",    datastore_create_string_resource(MQTT_LEN_BROKER_ADDRESS, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_BROKER_PORT,       "MQTT_BROKER_PORT",       datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_CONNECTION_COUNT,  "MQTT_CONNECTION_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_MESSAGE_TX_COUNT,  "MQTT_MESSAGE_TX_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_MESSAGE_RX_COUNT,  "MQTT_MESSAGE_RX_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_TEMP_VALUE,             "TEMP_VALUE",             datastore_create_resource(DATASTORE_TYPE_FLOAT,              SENSOR_TEMP_INSTANCES));
        _add_resource(datastore, RESOURCE_ID_TEMP_LABEL,             "TEMP_LABEL",             datastore_create_string_resource(SENSOR_TEMP_LEN_LABEL,      SENSOR_TEMP_INSTANCES));
        _add_resource(datastore, RESOURCE_ID_TEMP_DETECTED,          "TEMP_DETECTED",          datastore_create_string_resource(SENSOR_TEMP_LEN_ROM_CODE,   SENSOR_TEMP_INSTANCES));
        _add_resource(datastore, RESOURCE_ID_TEMP_ASSIGNMENT,        "TEMP_ASSIGNMENT",        datastore_create_string_resource(SENSOR_TEMP_LEN_ROM_CODE,   SENSOR_TEMP_INSTANCES));

        _add_resource(datastore, RESOURCE_ID_LIGHT_I2C_ADDRESS,      "LIGHT_I2C_ADDRESS",      datastore_create_resource(DATASTORE_TYPE_UINT8,  1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_DETECTED,         "LIGHT_DETECTED",         datastore_create_resource(DATASTORE_TYPE_BOOL,   1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_FULL,             "LIGHT_FULL",             datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_VISIBLE,          "LIGHT_VISIBLE",          datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_INFRARED,         "LIGHT_INFRARED",         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_ILLUMINANCE,      "LIGHT_ILLUMINANCE",      datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_FLOW_FREQUENCY,         "FLOW_FREQUENCY",         datastore_create_resource(DATASTORE_TYPE_FLOAT,  1));
        _add_resource(datastore, RESOURCE_ID_FLOW_RATE,              "FLOW_RATE",              datastore_create_resource(DATASTORE_TYPE_FLOAT,  1));

        _add_resource(datastore, RESOURCE_ID_POWER_TEMP_DELTA,       "POWER_TEMP_DELTA",       datastore_create_resource(DATASTORE_TYPE_FLOAT,  1));
        _add_resource(datastore, RESOURCE_ID_POWER_VALUE,            "POWER_VALUE",            datastore_create_resource(DATASTORE_TYPE_FLOAT,  1));

        _add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MODE_VALUE, "SWITCHES_CP_MODE_VALUE", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //_add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MODE_COUNT, "SWITCHES_CP_MODE_COUNT", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MAN_VALUE,  "SWITCHES_CP_MAN_VALUE",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //_add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MAN_COUNT, "SWITCHES_CP_MAN_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MODE_VALUE, "SWITCHES_PP_MODE_VALUE", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //_add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MODE_COUNT, "SWITCHES_PP_MODE_COUNT", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MAN_VALUE,  "SWITCHES_PP_MAN_VALUE",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //_add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MAN_COUNT, "SWITCHES_PP_MAN_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_PUMPS_CP_STATE,         "PUMPS_CP_STATE",         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_PUMPS_PP_STATE,         "PUMPS_PP_STATE",         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_CONTROL_CP_ON_DELTA,    "CONTROL_CP_ON_DELTA",    datastore_create_resource(DATASTORE_TYPE_FLOAT, 1));
        _add_resource(datastore, RESOURCE_ID_CONTROL_CP_OFF_DELTA,   "CONTROL_CP_OFF_DELTA",   datastore_create_resource(DATASTORE_TYPE_FLOAT, 1));
        _add_resource(datastore, RESOURCE_ID_CONTROL_FLOW_THRESHOLD, "CONTROL_FLOW_THRESHOLD", datastore_create_resource(DATASTORE_TYPE_FLOAT, 1));

        _add_resource(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_COUNT,          "CONTROL_PP_CYCLE_COUNT",          datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_ON_DURATION,    "CONTROL_PP_CYCLE_ON_DURATION",    datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_PAUSE_DURATION, "CONTROL_PP_CYCLE_PAUSE_DURATION", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_CONTROL_PP_DAILY_HOUR,           "CONTROL_PP_DAILY_HOUR",           datastore_create_resource(DATASTORE_TYPE_INT32, 1));
        _add_resource(datastore, RESOURCE_ID_CONTROL_PP_DAILY_MINUTE,         "CONTROL_PP_DAILY_MINUTE",         datastore_create_resource(DATASTORE_TYPE_INT32, 1));

        _add_resource(datastore, RESOURCE_ID_AVR_VERSION,       "AVR_VERSION",       datastore_create_resource(DATASTORE_TYPE_UINT8, 1));
        _add_resource(datastore, RESOURCE_ID_AVR_COUNT_RESET,   "AVR_COUNT_RESET",   datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_AVR_COUNT_CP,      "AVR_COUNT_CP",      datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_AVR_COUNT_PP,      "AVR_COUNT_PP",      datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_AVR_COUNT_CP_MODE, "AVR_COUNT_CP_MODE", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_AVR_COUNT_CP_MAN,  "AVR_COUNT_CP_MAN",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_AVR_COUNT_PP_MODE, "AVR_COUNT_PP_MODE", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_AVR_COUNT_PP_MAN,  "AVR_COUNT_PP_MAN",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_AVR_COUNT_BUZZER,  "AVR_COUNT_BUZZER",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
    }

    // defaults
    ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_SYSTEM_VERSION, 0, VERSION));
    ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_SYSTEM_BUILD_DATE_TIME, 0, BUILD_TIMESTAMP));
    ERROR_CHECK(datastore_set_bool(datastore, RESOURCE_ID_SYSTEM_TIME_SET, 0, false));

    ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_WIFI_SSID, 0, CONFIG_WIFI_SSID));
    ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_WIFI_PASSWORD, 0, CONFIG_WIFI_PASSWORD));

    ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_MQTT_BROKER_ADDRESS, 0, CONFIG_MQTT_BROKER_IP_ADDRESS));
    ERROR_CHECK(datastore_set_uint32(datastore, RESOURCE_ID_MQTT_BROKER_PORT, 0, CONFIG_MQTT_BROKER_TCP_PORT));

    ERROR_CHECK(datastore_set_uint8(datastore, RESOURCE_ID_LIGHT_I2C_ADDRESS, 0, CONFIG_LIGHT_SENSOR_I2C_ADDRESS));

    return datastore;
}

static const char * _make_key(const char * name, datastore_instance_id_t instance_id, char * key, size_t key_length)
{
    // key length maximum is 15 characters, allow two for instance ID
    //snprintf(key, key_length, "%.13s%2d", name, instance_id);
    // hash the name to avoid prefix collisions due to limited key length
    // sdbm hash:
    uint32_t hash_value = 0;
    for (uint32_t counter = 0; name[counter] != '\0'; ++name)
    {
        hash_value = name[counter] + (hash_value << 6) + (hash_value << 16) - hash_value;
    }
    snprintf(key, key_length, "%x:%d", hash_value, instance_id);
    return key;
}

static datastore_status_t _load_from_nvs(nvs_handle nh, const datastore_t * datastore, datastore_resource_id_t resource_id, datastore_instance_id_t instance_id, const char * default_value)
{
    datastore_status_t err = DATASTORE_STATUS_UNKNOWN;
    const char * name = datastore_get_name(datastore, resource_id);
    char key[NVS_MAX_KEY_LEN] = "";
    _make_key(name, instance_id, key, NVS_MAX_KEY_LEN);
    ESP_LOGD(TAG, "%s:%d has key %s", name, instance_id, key);

    char value[NVS_MAX_STRING_LEN] = "";
    size_t size = sizeof(value);
    esp_err_t esp_err = nvs_get_str(nh, key, value, &size);
    if (esp_err == ESP_OK)
    {
        err = datastore_set_as_string(datastore, resource_id, instance_id, value);
        ESP_LOGD(TAG, "Loaded %s from NVS: %s", key, value);
    }
    else
    {
        if (esp_err != ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Error 0x%x reading %s:%d from NVS", esp_err, name, instance_id);
        }
        ESP_LOGI(TAG, "Set default for %s:%d: %s", name, instance_id, default_value);
        err = datastore_set_as_string(datastore, resource_id, instance_id, default_value);
    }
    return err;
}

void resources_load(const datastore_t * datastore)
{
    if (datastore)
    {
        ESP_LOGI(TAG, "Loading resources from NVS");

        // load from NVS, or use default
        nvs_handle nh;
        ESP_ERROR_CHECK(nvs_open(NVS_NAMESPACE_RESOURCES, NVS_READONLY, &nh));

        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_TEMP_LABEL, 0, "Temp 1"));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_TEMP_LABEL, 1, "Temp 2"));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_TEMP_LABEL, 2, "Temp 3"));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_TEMP_LABEL, 3, "Temp 4"));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_TEMP_LABEL, 4, "Temp 5"));

        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_TEMP_ASSIGNMENT, 0, ""));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_TEMP_ASSIGNMENT, 1, ""));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_TEMP_ASSIGNMENT, 2, ""));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_TEMP_ASSIGNMENT, 3, ""));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_TEMP_ASSIGNMENT, 4, ""));

        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_CONTROL_CP_ON_DELTA, 0, "4.0"));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_CONTROL_CP_OFF_DELTA, 0, "2.0"));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_CONTROL_FLOW_THRESHOLD, 0, "5.0"));

        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_CONTROL_PP_CYCLE_COUNT, 0, "5"));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_CONTROL_PP_CYCLE_ON_DURATION, 0, "60"));
        ERROR_CHECK(_load_from_nvs(nh, datastore, RESOURCE_ID_CONTROL_PP_CYCLE_PAUSE_DURATION, 0, "30"));
    }
}

static esp_err_t _save_to_nvs(nvs_handle nh, const datastore_t * datastore, datastore_resource_id_t resource_id, datastore_instance_id_t instance_id)
{
    const char * name = datastore_get_name(datastore, resource_id);
    char key[NVS_MAX_KEY_LEN] = "";
    _make_key(name, instance_id, key, NVS_MAX_KEY_LEN);

    char value[NVS_MAX_STRING_LEN] = "";
    ERROR_CHECK(datastore_get_as_string(datastore, resource_id, instance_id, value, sizeof(value)));

    esp_err_t err = nvs_set_str(nh, key, value);
    if (err == ESP_OK)
    {
        ESP_LOGD(TAG, "Saved %s:%d to NVS: %s", name, instance_id, value);
    }
    else
    {
        ESP_LOGE(TAG, "Error 0x%x writing %s:%d to NVS", err, name, instance_id);
    }
    return err;
}

void resources_save(const datastore_t * datastore)
{
    if (datastore)
    {
        ESP_LOGI(TAG, "Saving resources to NVS");

        nvs_handle nh;
        esp_err_t err = nvs_open(NVS_NAMESPACE_RESOURCES, NVS_READWRITE, &nh);
        if (err == ESP_OK)
        {
            for (size_t i = 0; i < SENSOR_TEMP_INSTANCES; ++i)
            {
                ESP_ERROR_CHECK(_save_to_nvs(nh, datastore, RESOURCE_ID_TEMP_LABEL, i));
                ESP_ERROR_CHECK(_save_to_nvs(nh, datastore, RESOURCE_ID_TEMP_ASSIGNMENT, i));
            }

            ESP_ERROR_CHECK(_save_to_nvs(nh, datastore, RESOURCE_ID_CONTROL_CP_ON_DELTA, 0));
            ESP_ERROR_CHECK(_save_to_nvs(nh, datastore, RESOURCE_ID_CONTROL_CP_OFF_DELTA, 0));
            ESP_ERROR_CHECK(_save_to_nvs(nh, datastore, RESOURCE_ID_CONTROL_FLOW_THRESHOLD, 0));

            ESP_ERROR_CHECK(_save_to_nvs(nh, datastore, RESOURCE_ID_CONTROL_PP_CYCLE_COUNT, 0));
            ESP_ERROR_CHECK(_save_to_nvs(nh, datastore, RESOURCE_ID_CONTROL_PP_CYCLE_ON_DURATION, 0));
            ESP_ERROR_CHECK(_save_to_nvs(nh, datastore, RESOURCE_ID_CONTROL_PP_CYCLE_PAUSE_DURATION, 0));

            nvs_commit(nh);
            nvs_close(nh);
        }
        else
        {
            ESP_LOGE(TAG, "Error 0x%x opening NVS for writing", err);
        }
    }
}

void resources_erase(const datastore_t * datastore, const char * description)
{
    char * tmp = strdup(description);

    // find the ':'
    size_t len = strlen(tmp);
    size_t i = 0;
    while(i < len && tmp[i] != ':')
        ++i;

    // assume instance 0 if ':' not found, or at end of string with no numeral following
    datastore_instance_id_t instance_id = 0;
    if (i < len)
    {
        tmp[i] = '\0';  // remove the ':' character and terminate the name
        if (i < len - 1)  // is there a numeric instance after the ':'?
        {
            instance_id = atoi(&tmp[i + 1]);
        }
        ESP_LOGD(TAG, "name %s, instance %d", tmp, instance_id);
    }

    char key[NVS_MAX_KEY_LEN] = "";
    _make_key(tmp, instance_id, key, NVS_MAX_KEY_LEN);

    nvs_handle nh;
    esp_err_t err = nvs_open(NVS_NAMESPACE_RESOURCES, NVS_READWRITE, &nh);
    if (err == ESP_OK)
    {
        err = nvs_erase_key(nh, key);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Erased %s:%d", tmp, instance_id);
        }
        else
        {
            if (err == ESP_ERR_NVS_NOT_FOUND)
            {
                ESP_LOGE(TAG, "Cannot erase %s:%d - not found", tmp, instance_id);
            }
            else
            {
                ESP_LOGE(TAG, "Error 0x%x while attempting to erase %s:%d", err, tmp, instance_id);
            }
        }
        nvs_commit(nh);
        nvs_close(nh);
    }

    free(tmp);
}

