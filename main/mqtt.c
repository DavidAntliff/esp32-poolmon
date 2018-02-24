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

/* Notes
 * The 256dpi/esp-mqtt component is better suited than tuanpmt/espmqtt because:
 *   1. it's better supported and utilised
 *   2. it's more reliable
 * The tuanpmt/espmqtt component suffers from a design flaw where MQTT messages grouped
 * together in the TCP stream are dropped (in both directions).
 *
 * One identified issue with the 256dpi/esp-mqtt component is that large messages
 * near the declared MQTT buffer size will cause a disconnect.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_mqtt.h"

#include "mqtt.h"
#include "resources.h"
#include "trie.h"
#include "convert_string.h"
#include "utils.h"
#include "datastore/datastore.h"

#define TAG "mqtt"

typedef struct
{
    trie * trie;
} private_t;

// TODO: Singleton for now
static trie * g_trie = NULL;

typedef enum
{
    MQTT_TYPE_INVALID = 0,
    MQTT_TYPE_BOOL,
    MQTT_TYPE_UINT8,
    MQTT_TYPE_UINT32,
    MQTT_TYPE_INT8,
    MQTT_TYPE_INT32,
    MQTT_TYPE_FLOAT,
    MQTT_TYPE_DOUBLE,
    MQTT_TYPE_STRING,
    MQTT_TYPE_LAST,
} mqtt_type_t;

typedef void (*generic_callback)(void);

typedef struct
{
    //const char * topic;
    mqtt_type_t type;
    generic_callback rcb;
    void * context;
} topic_info_t;

static void _status_callback(esp_mqtt_status_t status)
{
    ESP_LOGD(TAG, "_status_callback: %d", status);

    switch (status)
    {
        case ESP_MQTT_STATUS_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            datastore_set_uint32(g_datastore, RESOURCE_ID_MQTT_STATUS, 0, MQTT_STATUS_CONNECTED);
            datastore_set_uint32(g_datastore, RESOURCE_ID_MQTT_TIMESTAMP, 0, seconds_since_boot());
            datastore_increment(g_datastore, RESOURCE_ID_MQTT_CONNECTION_COUNT, 0);

            // send a device status update
            const char * value = "MQTT connected";
            mqtt_publish("poolmon/device/status", (uint8_t*)value, strlen(value), 0, false);
            esp_mqtt_subscribe("poolmon/#", 0);
            break;

        case ESP_MQTT_STATUS_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT disconnected");
            datastore_set_uint32(g_datastore, RESOURCE_ID_MQTT_STATUS, 0, MQTT_STATUS_DISCONNECTED);
            break;
        default:
            break;
    }
}

static void _message_callback(const char * topic, uint8_t * payload, size_t len)
{
    ESP_LOGD(TAG, "_message_callback: topic '%s', len %d", topic, len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, payload, len, ESP_LOG_DEBUG);

    const char * data = (const char *)payload;

    datastore_increment(g_datastore, RESOURCE_ID_MQTT_MESSAGE_RX_COUNT, 0);

    // TODO: use g_trie until we add a context pointer to the message callback
    topic_info_t * topic_info = trie_search(g_trie, topic);
    if (topic_info)
    {
        uint8_t value = atoi(data);
        ESP_LOGD(TAG, "%s %d", topic, value);
        if (topic_info->rcb)
        {
            // dispatch based on type
            switch (topic_info->type)
            {
            case MQTT_TYPE_BOOL:
            {
                // For true, accept any case of "T", "TRUE", non-zero
                // For false, accept any case of "F", "FALSE", zero
                bool value = false;
                uint32_t numeric = 0;
                if (strncasecmp("true", data, len) == 0)
                {
                    value = true;
                }
                else if (strncasecmp("false", data, len) == 0)
                {
                    value = false;
                }
                else if (string_to_uint32(data, &numeric))
                {
                    value = numeric;
                }
                else
                {
                    ESP_LOGE(TAG, "invalid value \'%s\' for bool", data);
                    goto skip;
                }
                ((mqtt_receive_callback_bool)(topic_info->rcb))(topic, value, topic_info->context);
            skip: ;
                break;
            }
            case MQTT_TYPE_UINT8:
            {
                uint8_t value = 0;
                if (string_to_uint8(data, &value))
                {
                    ((mqtt_receive_callback_uint8)(topic_info->rcb))(topic, value, topic_info->context);
                }
                else
                {
                    ESP_LOGE(TAG, "invalid value \'%s\' for uint8", data);
                }
                break;
            }
            case MQTT_TYPE_UINT32:
            {
                uint32_t value = 0;
                if (string_to_uint32(data, &value))
                {
                    ((mqtt_receive_callback_uint32)(topic_info->rcb))(topic, value, topic_info->context);
                }
                else
                {
                    ESP_LOGE(TAG, "invalid value \'%s\' for uint32", data);
                }
                break;
            }
            case MQTT_TYPE_INT8:
            {
                int8_t value = 0;
                if (string_to_int8(data, &value))
                {
                    ((mqtt_receive_callback_int8)(topic_info->rcb))(topic, value, topic_info->context);
                }
                else
                {
                    ESP_LOGE(TAG, "invalid value \'%s\' for int8", data);
                }
                break;
            }
            case MQTT_TYPE_INT32:
            {
                int32_t value = 0;
                if (string_to_int32(data, &value))
                {
                    ((mqtt_receive_callback_int32)(topic_info->rcb))(topic, value, topic_info->context);
                }
                else
                {
                    ESP_LOGE(TAG, "invalid value \'%s\' for int32", data);
                }
                break;
            }
            case MQTT_TYPE_FLOAT:
            {
                float value = 0;
                if (string_to_float(data, &value))
                {
                    ((mqtt_receive_callback_float)(topic_info->rcb))(topic, value, topic_info->context);
                }
                else
                {
                    ESP_LOGE(TAG, "invalid value \'%s\' for float", data);
                }
                break;
            }
            case MQTT_TYPE_DOUBLE:
            {
                double value = 0;
                if (string_to_double(data, &value))
                {
                    ((mqtt_receive_callback_double)(topic_info->rcb))(topic, value, topic_info->context);
                }
                else
                {
                    ESP_LOGE(TAG, "invalid value \'%s\' for double", data);
                }
                break;
            }
            case MQTT_TYPE_STRING:
            {
                ((mqtt_receive_callback_string)(topic_info->rcb))(topic, data, topic_info->context);
                break;
            }
            default:
                ESP_LOGE(TAG, "unhandled type %d", topic_info->type);
                break;
            }
        }
    }
    else
    {
        ESP_LOGW(TAG, "topic %s not handled", topic);
    }
}

mqtt_info_t * mqtt_malloc(void)
{
    mqtt_info_t * mqtt_info = NULL;
    private_t * private = malloc(sizeof(*private));
    if (private != NULL)
    {
        memset(private, 0, sizeof(*private));
        ESP_LOGD(TAG, "malloc private %p", private);

        mqtt_info = malloc(sizeof(*mqtt_info));
        if (mqtt_info)
        {
            ESP_LOGD(TAG, "malloc mqtt_info %p", mqtt_info);
            memset(mqtt_info, 0, sizeof(*mqtt_info));
            mqtt_info->private = private;
        }
        else
        {
            ESP_LOGE(TAG, "malloc failed");
        }
    }
    else
    {
        ESP_LOGE(TAG, "malloc failed");
    }

    return mqtt_info;
}

void mqtt_free(mqtt_info_t ** mqtt_info)
{
    if (mqtt_info != NULL && (*mqtt_info != NULL))
    {
        ESP_LOGD(TAG, "free private %p", (*mqtt_info)->private);

        private_t * private = (private_t *)(*mqtt_info)->private;
        if (private && private->trie)
        {
            // TODO: free all topic_info structs in trie

            ESP_LOGD(TAG, "free trie %p", private->trie);
            trie_free(private->trie);
            private->trie = 0;
        }
        free((*mqtt_info)->private);
        ESP_LOGD(TAG, "free mqtt_info %p", *mqtt_info);
        free(*mqtt_info);
        *mqtt_info = NULL;
    }
}

static mqtt_error_t _is_init(const mqtt_info_t * mqtt_info)
{
    mqtt_error_t err = MQTT_ERROR_NOT_INITIALISED;
    if (mqtt_info != NULL)
    {
        if (mqtt_info->private)
        {
            // OK
            err = MQTT_OK;
        }
        else
        {
            ESP_LOGE(TAG, "mqtt is not initialised");
        }
    }
    else
    {
        ESP_LOGE(TAG, "mqtt_info is NULL");
        err = MQTT_ERROR_NULL_POINTER;
    }
    return err;
}

mqtt_error_t mqtt_init(mqtt_info_t * mqtt_info)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    mqtt_error_t err = MQTT_ERROR_UNKNOWN;
    if (mqtt_info != NULL)
    {
        private_t * private = (private_t *)mqtt_info->private;
        if (private != NULL)
        {
            // sadly the esp_mqtt component only supports a single instance
            esp_mqtt_init(_status_callback, _message_callback, 256 /*buffer size*/, 2000 /*timeout*/);
            datastore_set_uint32(g_datastore, RESOURCE_ID_MQTT_STATUS, 0, MQTT_STATUS_CONNECTING);
            private->trie = trie_create();

            if (private->trie)
            {
                ESP_LOGD(TAG, "malloc trie %p", private->trie);
                // TODO:
                g_trie = private->trie;
                err = MQTT_OK;
            }
            else
            {
                ESP_LOGE(TAG, "unable to create trie");
                err = MQTT_ERROR_NULL_POINTER;
            }
        }
        else
        {
            ESP_LOGE(TAG, "mqtt_info->private is NULL");
            err = MQTT_ERROR_NULL_POINTER;
        }
    }
    else
    {
        ESP_LOGE(TAG, "mqtt_info is NULL");
        err = MQTT_ERROR_NULL_POINTER;
    }
    return err;
}

bool mqtt_publish(const char * topic, uint8_t * payload, size_t len, int qos, bool retained)
{
    bool result = false;
    if ((result = esp_mqtt_publish(topic, payload, len, qos, retained)) != false)
    {
        datastore_increment(g_datastore, RESOURCE_ID_MQTT_MESSAGE_TX_COUNT, 0);
    }
    return result;
}

static mqtt_error_t _register_topic(mqtt_info_t * mqtt_info, const char * topic, generic_callback rcb, void * context, mqtt_type_t type)
{
    mqtt_error_t err = MQTT_ERROR_UNKNOWN;
    if ((err = _is_init(mqtt_info)) == MQTT_OK)
    {
        if (topic)
        {
            const private_t * private = (const private_t *)mqtt_info->private;
            topic_info_t * topic_info = malloc(sizeof(*topic_info));
            if (topic_info)
            {
                memset(topic_info, 0, sizeof(*topic_info));
                //topic_info->topic = strdup(topic);
                topic_info->type = type;
                topic_info->rcb = rcb;
                topic_info->context = context;
                trie_insert(private->trie, topic, topic_info);
            }
            else
            {
                ESP_LOGE(TAG, "malloc failed");
                err = MQTT_ERROR_NULL_POINTER;
            }
        }
        else
        {
            ESP_LOGE(TAG, "topic is NULL");
            err = MQTT_ERROR_NULL_POINTER;
        }
    }
    return err;
}

mqtt_error_t mqtt_register_topic_as_bool(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_bool rcb, void * context)
{
    return _register_topic(mqtt_info, topic, (generic_callback)rcb, context, MQTT_TYPE_BOOL);
}

mqtt_error_t mqtt_register_topic_as_uint8(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_uint8 rcb, void * context)
{
    return _register_topic(mqtt_info, topic, (generic_callback)rcb, context, MQTT_TYPE_UINT8);
}

mqtt_error_t mqtt_register_topic_as_uint32(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_uint32 rcb, void * context)
{
    return _register_topic(mqtt_info, topic, (generic_callback)rcb, context, MQTT_TYPE_UINT32);
}

mqtt_error_t mqtt_register_topic_as_int8(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_int8 rcb, void * context)
{
    return _register_topic(mqtt_info, topic, (generic_callback)rcb, context, MQTT_TYPE_INT8);
}

mqtt_error_t mqtt_register_topic_as_int32(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_int32 rcb, void * context)
{
    return _register_topic(mqtt_info, topic, (generic_callback)rcb, context, MQTT_TYPE_INT32);
}

mqtt_error_t mqtt_register_topic_as_float(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_float rcb, void * context)
{
    return _register_topic(mqtt_info, topic, (generic_callback)rcb, context, MQTT_TYPE_FLOAT);
}

mqtt_error_t mqtt_register_topic_as_double(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_double rcb, void * context)
{
    return _register_topic(mqtt_info, topic, (generic_callback)rcb, context, MQTT_TYPE_DOUBLE);
}

mqtt_error_t mqtt_register_topic_as_string(mqtt_info_t * mqtt_info, const char * topic, mqtt_receive_callback_string rcb, void * context)
{
    return _register_topic(mqtt_info, topic, (generic_callback)rcb, context, MQTT_TYPE_STRING);
}

void mqtt_dump(const mqtt_info_t * mqtt_info)
{
    mqtt_error_t err = MQTT_ERROR_UNKNOWN;
    if ((err = _is_init(mqtt_info)) == MQTT_OK)
    {
        const private_t * private = (const private_t *)mqtt_info->private;
        ESP_LOGW(TAG, "trie: count %d, size %d bytes", trie_count(private->trie, ""), trie_size(private->trie));
    }
}

#if 0
// let's try a global variable for now
mqtt_client * g_client = NULL;

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
            mqtt_publish(g_client, "poolmon/device/status", value, strlen(value), 0, 0);

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

mqtt_settings g_settings = {
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
#endif
