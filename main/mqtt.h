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

#ifndef MQTT_H
#define MQTT_H

#include "esp_mqtt.h"
#include "datastore/datastore.h"

typedef enum
{
    MQTT_ERROR_UNKNOWN = -1,
    MQTT_OK = 0,
    MQTT_ERROR_NULL_POINTER,
    MQTT_ERROR_NOT_INITIALISED,
    MQTT_ERROR_INVALID_TYPE,
    MQTT_ERROR_LAST,
} mqtt_error_t;

typedef enum
{
    MQTT_STATUS_DISCONNECTED = 0,
    MQTT_STATUS_CONNECTING,
    MQTT_STATUS_CONNECTED,
} mqtt_status_t;

#define MQTT_LEN_BROKER_ADDRESS 64

typedef struct
{
    void * private;
} mqtt_info_t;

mqtt_info_t * mqtt_malloc(void);
void mqtt_free(mqtt_info_t ** mqtt_info);
mqtt_error_t mqtt_init(mqtt_info_t * mqtt_info, const datastore_t * datastore);
mqtt_error_t mqtt_start(mqtt_info_t * mqtt_info);

bool mqtt_publish(const char * topic, const uint8_t * payload, size_t len, int qos, bool retained);

typedef void (*mqtt_receive_callback_bool)(const char * topic, bool value, void * context);
typedef void (*mqtt_receive_callback_uint8)(const char * topic, uint8_t value, void * context);
typedef void (*mqtt_receive_callback_uint32)(const char * topic, uint32_t value, void * context);
typedef void (*mqtt_receive_callback_int8)(const char * topic, int8_t value, void * context);
typedef void (*mqtt_receive_callback_int32)(const char * topic, int32_t value, void * context);
typedef void (*mqtt_receive_callback_float)(const char * topic, float value, void * context);
typedef void (*mqtt_receive_callback_double)(const char * topic, double value, void * context);
typedef void (*mqtt_receive_callback_string)(const char * topic, const char * value, void * context);

mqtt_error_t mqtt_register_topic_as_bool(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_bool rcb, void * context);
mqtt_error_t mqtt_register_topic_as_uint8(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_uint8 rcb, void * context);
mqtt_error_t mqtt_register_topic_as_uint32(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_uint32 rcb, void * context);
mqtt_error_t mqtt_register_topic_as_int8(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_int8 rcb, void * context);
mqtt_error_t mqtt_register_topic_as_int32(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_int32 rcb, void * context);
mqtt_error_t mqtt_register_topic_as_float(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_float rcb, void * context);
mqtt_error_t mqtt_register_topic_as_double(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_double rcb, void * context);
mqtt_error_t mqtt_register_topic_as_string(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_string rcb, void * context);

#endif // MQTT_H
