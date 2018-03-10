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

#include "esp_log.h"

#include "resources.h"
#include "constants.h"
#include "wifi_support.h"
#include "mqtt.h"
#include "sensor_temp.h"

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
        _add_resource(datastore, RESOURCE_ID_SYSTEM_UPTIME,          "SYSTEM_UPTIME",          datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_WIFI_SSID,              "WIFI_SSID",              datastore_create_string_resource(WIFI_LEN_SSID, 1));
        _add_resource(datastore, RESOURCE_ID_WIFI_PASSWORD,          "WIFI_PASSWORD",          datastore_create_string_resource(WIFI_LEN_PASSWORD, 1));
        _add_resource(datastore, RESOURCE_ID_WIFI_STATUS,            "WIFI_STATUS",            datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_WIFI_TIMESTAMP,         "WIFI_TIMESTAMP",         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_WIFI_RSSI,              "WIFI_RSSI",              datastore_create_resource(DATASTORE_TYPE_INT8,   1));
        _add_resource(datastore, RESOURCE_ID_WIFI_ADDRESS,           "WIFI_ADDRESS",           datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_MQTT_STATUS,            "MQTT_STATUS",            datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_TIMESTAMP,         "MQTT_TIMESTAMP",         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_BROKER_ADDRESS,    "MQTT_BROKER_ADDRESS",    datastore_create_string_resource(MQTT_LEN_BROKER_ADDRESS, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_BROKER_PORT,       "MQTT_BROKER_PORT",       datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_CONNECTION_COUNT,  "MQTT_CONNECTION_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_MESSAGE_TX_COUNT,  "MQTT_MESSAGE_TX_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_MQTT_MESSAGE_RX_COUNT,  "MQTT_MESSAGE_RX_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_TEMP_VALUE,             "TEMP_VALUE",             datastore_create_resource(DATASTORE_TYPE_FLOAT,              SENSOR_TEMP_INSTANCES));
        _add_resource(datastore, RESOURCE_ID_TEMP_TIMESTAMP,         "TEMP_TIMESTAMP",         datastore_create_resource(DATASTORE_TYPE_UINT32,             SENSOR_TEMP_INSTANCES));
        _add_resource(datastore, RESOURCE_ID_TEMP_LABEL,             "TEMP_LABEL",             datastore_create_string_resource(SENSOR_TEMP_LEN_LABEL,      SENSOR_TEMP_INSTANCES));
        _add_resource(datastore, RESOURCE_ID_TEMP_DETECTED,          "TEMP_DETECTED",          datastore_create_string_resource(SENSOR_TEMP_LEN_ROM_CODE,   SENSOR_TEMP_INSTANCES));
        _add_resource(datastore, RESOURCE_ID_TEMP_ASSIGNMENT,        "TEMP_ASSIGNMENT",        datastore_create_string_resource(SENSOR_TEMP_LEN_ROM_CODE,   SENSOR_TEMP_INSTANCES));

        _add_resource(datastore, RESOURCE_ID_LIGHT_I2C_ADDRESS,      "LIGHT_I2C_ADDRESS",      datastore_create_resource(DATASTORE_TYPE_UINT8,  1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_DETECTED,         "LIGHT_DETECTED",         datastore_create_resource(DATASTORE_TYPE_BOOL,   1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_FULL,             "LIGHT_FULL",             datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_VISIBLE,          "LIGHT_VISIBLE",          datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_INFRARED,         "LIGHT_INFRARED",         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_ILLUMINANCE,      "LIGHT_ILLUMINANCE",      datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_LIGHT_TIMESTAMP,        "LIGHT_TIMESTAMP",        datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_FLOW_FREQUENCY,         "FLOW_FREQUENCY",         datastore_create_resource(DATASTORE_TYPE_FLOAT,  1));
        _add_resource(datastore, RESOURCE_ID_FLOW_RATE,              "FLOW_RATE",              datastore_create_resource(DATASTORE_TYPE_FLOAT,  1));

        _add_resource(datastore, RESOURCE_ID_POWER_VALUE,            "POWER_VALUE",            datastore_create_resource(DATASTORE_TYPE_FLOAT,  1));
        _add_resource(datastore, RESOURCE_ID_POWER_TIMESTAMP,        "POWER_TIMESTAMP",        datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MODE_VALUE, "SWITCHES_CP_MODE_VALUE", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //_add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MODE_COUNT, "SWITCHES_CP_MODE_COUNT", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MAN_VALUE,  "SWITCHES_CP_MAN_VALUE",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //_add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MAN_COUNT, "SWITCHES_CP_MAN_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MODE_VALUE, "SWITCHES_PP_MODE_VALUE", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //_add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MODE_COUNT, "SWITCHES_PP_MODE_COUNT", datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MAN_VALUE,  "SWITCHES_PP_MAN_VALUE",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //_add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MAN_COUNT, "SWITCHES_PP_MAN_COUNT",  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_SWITCHES_TIMESTAMP,     "SWITCHES_TIMESTAMP",     datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_PUMPS_CP_STATE,         "PUMPS_CP_STATE",         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        _add_resource(datastore, RESOURCE_ID_PUMPS_PP_STATE,         "PUMPS_PP_STATE",         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        _add_resource(datastore, RESOURCE_ID_CONTROL_CP_ON_DELTA,    "CONTROL_CP_ON_DELTA",    datastore_create_resource(DATASTORE_TYPE_FLOAT, 1));
        _add_resource(datastore, RESOURCE_ID_CONTROL_CP_OFF_DELTA,   "CONTROL_CP_OFF_DELTA",   datastore_create_resource(DATASTORE_TYPE_FLOAT, 1));
    }

    return datastore;
}

void resources_load(const datastore_t * datastore)
{
    if (datastore)
    {
        ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_SYSTEM_VERSION, 0, VERSION));
        ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_SYSTEM_BUILD_DATE_TIME, 0, BUILD_TIMESTAMP));

        // TODO: load from NV
        ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_WIFI_SSID, 0, CONFIG_WIFI_SSID));
        ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_WIFI_PASSWORD, 0, CONFIG_WIFI_PASSWORD));

        ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_MQTT_BROKER_ADDRESS, 0, CONFIG_MQTT_BROKER_IP_ADDRESS));
        ERROR_CHECK(datastore_set_uint32(datastore, RESOURCE_ID_MQTT_BROKER_PORT, 0, CONFIG_MQTT_BROKER_TCP_PORT));

        ERROR_CHECK(datastore_set_uint8(datastore, RESOURCE_ID_LIGHT_I2C_ADDRESS, 0, CONFIG_LIGHT_SENSOR_I2C_ADDRESS));

        // TODO
        ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, 0, "Temp 1"));
        ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, 1, "Temp 2"));
        ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, 2, "Temp 3"));
        ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, 3, "Temp 4"));
        ERROR_CHECK(datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, 4, "Temp 5"));

        ERROR_CHECK(datastore_set_float(datastore, RESOURCE_ID_CONTROL_CP_ON_DELTA, 0, 4.0f));
        ERROR_CHECK(datastore_set_float(datastore, RESOURCE_ID_CONTROL_CP_OFF_DELTA, 0, 2.0f));
    }
}
