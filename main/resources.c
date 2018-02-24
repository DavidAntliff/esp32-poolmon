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

#include "resources.h"
#include "constants.h"
#include "wifi_support.h"
#include "mqtt.h"
#include "sensor_temp.h"

datastore_t * g_datastore = NULL;

datastore_t * resources_init(void)
{
    datastore_t * datastore = datastore_create();

    if (datastore)
    {
        datastore_add_resource(datastore, RESOURCE_ID_SYSTEM_VERSION,         datastore_create_string_resource(SYSTEM_LEN_VERSION, 1));
        datastore_add_resource(datastore, RESOURCE_ID_SYSTEM_BUILD_DATE_TIME, datastore_create_string_resource(SYSTEM_LEN_BUILD_DATE_TIME, 1));
        datastore_add_resource(datastore, RESOURCE_ID_SYSTEM_UPTIME,          datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        datastore_add_resource(datastore, RESOURCE_ID_WIFI_SSID,              datastore_create_string_resource(WIFI_LEN_SSID, 1));
        datastore_add_resource(datastore, RESOURCE_ID_WIFI_PASSWORD,          datastore_create_string_resource(WIFI_LEN_PASSWORD, 1));
        datastore_add_resource(datastore, RESOURCE_ID_WIFI_STATUS,            datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_WIFI_TIMESTAMP,         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_WIFI_RSSI,              datastore_create_resource(DATASTORE_TYPE_INT8,   1));
        datastore_add_resource(datastore, RESOURCE_ID_WIFI_ADDRESS,           datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        datastore_add_resource(datastore, RESOURCE_ID_MQTT_STATUS,            datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_MQTT_TIMESTAMP,         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_MQTT_BROKER_ADDRESS,    datastore_create_string_resource(MQTT_LEN_BROKER_ADDRESS, 1));
        datastore_add_resource(datastore, RESOURCE_ID_MQTT_BROKER_PORT,       datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_MQTT_CONNECTION_COUNT,  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_MQTT_MESSAGE_TX_COUNT,  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_MQTT_MESSAGE_RX_COUNT,  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        datastore_add_resource(datastore, RESOURCE_ID_TEMP_VALUE,             datastore_create_resource(DATASTORE_TYPE_FLOAT,  SENSOR_TEMP_INSTANCES));
        datastore_add_resource(datastore, RESOURCE_ID_TEMP_TIMESTAMP,         datastore_create_resource(DATASTORE_TYPE_UINT32, SENSOR_TEMP_INSTANCES));
        datastore_add_resource(datastore, RESOURCE_ID_TEMP_LABEL,             datastore_create_string_resource(SENSOR_TEMP_LEN_LABEL, SENSOR_TEMP_INSTANCES));
        datastore_add_resource(datastore, RESOURCE_ID_TEMP_ASSIGNMENT,        datastore_create_resource(DATASTORE_TYPE_UINT8,  SENSOR_TEMP_INSTANCES));

        datastore_add_resource(datastore, RESOURCE_ID_LIGHT_I2C_ADDRESS,      datastore_create_resource(DATASTORE_TYPE_UINT8,  1));
        datastore_add_resource(datastore, RESOURCE_ID_LIGHT_DETECTED,         datastore_create_resource(DATASTORE_TYPE_BOOL,   1));
        datastore_add_resource(datastore, RESOURCE_ID_LIGHT_FULL,             datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_LIGHT_VISIBLE,          datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_LIGHT_INFRARED,         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_LIGHT_ILLUMINANCE,      datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_LIGHT_TIMESTAMP,        datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_FLOW_FREQUENCY,         datastore_create_resource(DATASTORE_TYPE_FLOAT,  1));
        datastore_add_resource(datastore, RESOURCE_ID_FLOW_RATE,              datastore_create_resource(DATASTORE_TYPE_FLOAT,  1));

        datastore_add_resource(datastore, RESOURCE_ID_POWER_VALUE,            datastore_create_resource(DATASTORE_TYPE_FLOAT,  1));
        datastore_add_resource(datastore, RESOURCE_ID_POWER_TIMESTAMP,        datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        datastore_add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MODE_VALUE, datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //datastore_add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MODE_COUNT, datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MAN_VALUE,  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //datastore_add_resource(datastore, RESOURCE_ID_SWITCHES_CP_MAN_COUNT,  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MODE_VALUE, datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //datastore_add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MODE_COUNT, datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MAN_VALUE,  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        //datastore_add_resource(datastore, RESOURCE_ID_SWITCHES_PP_MAN_COUNT,  datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_SWITCHES_TIMESTAMP,     datastore_create_resource(DATASTORE_TYPE_UINT32, 1));

        datastore_add_resource(datastore, RESOURCE_ID_PUMPS_CP_STATE,         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
        datastore_add_resource(datastore, RESOURCE_ID_PUMPS_PP_STATE,         datastore_create_resource(DATASTORE_TYPE_UINT32, 1));
    }

    return datastore;
}

void resources_load(const datastore_t * datastore)
{
    if (datastore)
    {
        datastore_set_string(datastore, RESOURCE_ID_SYSTEM_VERSION, 0, VERSION);
        datastore_set_string(datastore, RESOURCE_ID_SYSTEM_BUILD_DATE_TIME, 0, BUILD_TIMESTAMP);

        // TODO: load from NV
        datastore_set_string(datastore, RESOURCE_ID_WIFI_SSID, 0, CONFIG_WIFI_SSID);
        datastore_set_string(datastore, RESOURCE_ID_WIFI_PASSWORD, 0, CONFIG_WIFI_PASSWORD);

        datastore_set_string(datastore, RESOURCE_ID_MQTT_BROKER_ADDRESS, 0, CONFIG_MQTT_BROKER_IP_ADDRESS);
        datastore_set_uint32(datastore, RESOURCE_ID_MQTT_BROKER_PORT, 0, CONFIG_MQTT_BROKER_TCP_PORT);

        datastore_set_uint8(datastore, RESOURCE_ID_LIGHT_I2C_ADDRESS, 0, CONFIG_LIGHT_SENSOR_I2C_ADDRESS);

        // TODO
        datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, 0, "LABEL1");
        datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, 1, "LABEL2");
        datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, 2, "LABEL3");
        datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, 3, "LABEL4");
        datastore_set_string(datastore, RESOURCE_ID_TEMP_LABEL, 4, "LABEL5");
    }
}
