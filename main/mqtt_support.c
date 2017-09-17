/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "mqtt_support.h"
#include "mqtt.h"

#define TAG "mqtt_support"

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
