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

#ifndef RESOURCES_H
#define RESOURCES_H

#include "datastore/datastore.h"

typedef enum
{
    RESOURCE_ID_SYSTEM_VERSION = 0,
    RESOURCE_ID_SYSTEM_BUILD_DATE_TIME,
    RESOURCE_ID_SYSTEM_UPTIME,

//    RESOURCE_ID_I2C_MASTER_DEVICE_COUNT,
//    RESOURCE_ID_I2C_ERROR_COUNT,
//    RESOURCE_ID_I2C_ERROR_TIMESTAMP,
//
    RESOURCE_ID_WIFI_SSID,
    RESOURCE_ID_WIFI_PASSWORD,
    RESOURCE_ID_WIFI_STATUS,
    RESOURCE_ID_WIFI_TIMESTAMP,
    RESOURCE_ID_WIFI_RSSI,
    RESOURCE_ID_WIFI_ADDRESS,

    RESOURCE_ID_MQTT_STATUS,
    RESOURCE_ID_MQTT_TIMESTAMP,
    RESOURCE_ID_MQTT_BROKER_ADDRESS,
    RESOURCE_ID_MQTT_BROKER_PORT,
    RESOURCE_ID_MQTT_CONNECTION_COUNT,
    RESOURCE_ID_MQTT_MESSAGE_TX_COUNT,
    RESOURCE_ID_MQTT_MESSAGE_RX_COUNT,

    RESOURCE_ID_TEMP_VALUE,
    RESOURCE_ID_TEMP_TIMESTAMP,
    RESOURCE_ID_TEMP_LABEL,
    RESOURCE_ID_TEMP_ASSIGNMENT,

    RESOURCE_ID_LIGHT_I2C_ADDRESS,
    RESOURCE_ID_LIGHT_DETECTED,
    RESOURCE_ID_LIGHT_FULL,
    RESOURCE_ID_LIGHT_VISIBLE,
    RESOURCE_ID_LIGHT_INFRARED,
    RESOURCE_ID_LIGHT_ILLUMINANCE,
    RESOURCE_ID_LIGHT_TIMESTAMP,

    RESOURCE_ID_FLOW_FREQUENCY,
    RESOURCE_ID_FLOW_RATE,

    RESOURCE_ID_POWER_VALUE,
    RESOURCE_ID_POWER_TIMESTAMP,

    RESOURCE_ID_SWITCHES_CP_MODE_VALUE,
    //RESOURCE_ID_SWITCHES_CP_MODE_COUNT,
    RESOURCE_ID_SWITCHES_CP_MAN_VALUE,
    //RESOURCE_ID_SWITCHES_CP_MAN_COUNT,
    RESOURCE_ID_SWITCHES_PP_MODE_VALUE,
    //RESOURCE_ID_SWITCHES_PP_MODE_COUNT,
    RESOURCE_ID_SWITCHES_PP_MAN_VALUE,
    //RESOURCE_ID_SWITCHES_PP_MAN_COUNT,
    RESOURCE_ID_SWITCHES_TIMESTAMP,

    RESOURCE_ID_PUMPS_CP_STATE,
    RESOURCE_ID_PUMPS_PP_STATE,

    RESOURCE_ID_LAST,
} resource_id_t;

datastore_t * resources_init(void);
void resources_load(const datastore_t * datastore);

#endif // RESOURCES_H