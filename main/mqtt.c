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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_mqtt.h"

#include "mqtt.h"
#include "trie.h"

#define TAG "mqtt"

typedef struct
{
    trie * trie;
} private_t;

// TODO: Singleton for now
static trie * g_trie = NULL;

static void _status_callback(esp_mqtt_status_t status)
{
    ESP_LOGI(TAG, "_status_callback: %d", status);

    switch (status)
    {
        case ESP_MQTT_STATUS_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");

            // send a device status update
            const char * value = "MQTT connected";
            esp_mqtt_publish("poolmon/device/status", (uint8_t*)value, strlen(value), 0, false);
            esp_mqtt_subscribe("poolmon/#", 0);
            break;
        case ESP_MQTT_STATUS_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        default:
            break;
    }
}

static void _message_callback(const char * topic, uint8_t * payload, size_t len)
{
    ESP_LOGI(TAG, "_message_callback: topic '%s', len %d", topic, len);
    esp_log_buffer_hex(TAG, payload, len);
    const char * data = (const char *)payload;

    // TODO: handle values other than uint8_ts
    // TODO: use g_trie for the moment
    uint8_t * result = trie_search(g_trie, topic);
    if (result)
    {
        *result = atoi(data);
        ESP_LOGI(TAG, "%s %d", topic, *result);
    }
    else
    {
        ESP_LOGI(TAG, "%s not handled", topic);
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
        free((*mqtt_info)->private);
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
    mqtt_error_t err = MQTT_ERROR_UNKNOWN;
    if (mqtt_info != NULL)
    {
        private_t * private = (private_t *)mqtt_info->private;
        if (private != NULL)
        {
            // sadly the esp_mqtt component only supports a single instance
            esp_mqtt_init(_status_callback, _message_callback, 256 /*buffer size*/, 2000 /*timeout*/);
            private->trie = trie_create();

            if (private->trie)
            {
                // TODO:
                g_trie = private->trie;
                err = MQTT_OK;
            }
            else
            {
                ESP_LOGE(TAG, "unable to create trie");
                err = MQTT_ERROR_CANNOT_CREATE;
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
