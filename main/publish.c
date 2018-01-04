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
#include "esp_log.h"

#include "publish.h"
#include "mqtt.h"

#define TAG "publish"

extern mqtt_client * g_client;

#define ROOT_TOPIC "poolmon"

typedef struct
{
    const char * topic;
} value_info_t;

// value publish info indexed by value ID
static const value_info_t values_info[PUBLISH_VALUE_LAST] =
{
    { "sensors/temp/1", },
    { "sensors/temp/2", },
    { "sensors/temp/3", },
    { "sensors/temp/4", },
    { "sensors/temp/5", },

    { "sensors/light/1/full_spectrum", },
    { "sensors/light/1/visible", },
    { "sensors/light/1/infrared", },
    { "sensors/light/1/lux", },

    { "sensors/flow/1/freq", },
    { "sensors/flow/1/rate", },

    { "switches/1", },
    { "switches/2", },
    { "switches/3", },
    { "switches/4", },

    { "ssr/1", },
    { "ssr/2", },

    { "alarm/1", },
};


// publish sensor readings
static void publish_task(void * pvParameter)
{
    QueueHandle_t publish_queue = (QueueHandle_t)pvParameter;
    ESP_LOGW(TAG, "Core ID %d", xPortGetCoreID());

    while (1)
    {
        // Not necessarily true when using multiple cores:
        //if (uxQueueMessagesWaiting(publish_queue) != 0)
        //{
        //    ESP_LOGE(TAG":mqtt", "Queue should be empty!\n");
        //}

        published_value_t published_value = {0};
        BaseType_t sensor_queue_status = xQueueReceive(publish_queue, &published_value, portMAX_DELAY);
        if (sensor_queue_status == pdPASS)
        {
            // TODO: we need a more reliable way to know that the client is valid,
            // as attempting to publish when the client has died causes a crash.
            if (g_client != NULL)
            {
                ESP_LOGD(TAG, "Received %d:%f", published_value.id, published_value.value);
                if (published_value.id >= 0 && published_value.id < PUBLISH_VALUE_LAST)
                {
                    char topic[64] = {0};
                    char value[8] = {0};
                    snprintf(topic, 64-1, "%s/%s", ROOT_TOPIC, values_info[published_value.id].topic);
                    snprintf(value, 8-1, "%.3f", published_value.value);
                    ESP_LOGI(TAG, "Publish %s %s", topic, value);
                    mqtt_publish(g_client, topic, value, strlen(value), 0, 0);
                }
                else
                {
                    ESP_LOGE(TAG, "Invalid value ID %d", published_value.id);
                }
            }
            else
            {
                ESP_LOGW(TAG":publish_task", "MQTT not ready - throw away");
            }
        }
        else
        {
            ESP_LOGE(TAG":publish_task", "Could not receive from queue");
        }
    }

    vTaskDelete(NULL);
}

void publish_value(publish_value_id_t value_id, float value, QueueHandle_t publish_queue)
{
    if (value_id >= 0 && value_id < PUBLISH_VALUE_LAST)
    {
        published_value_t published_value = {
            .id = value_id,
            .value = value,
        };
        BaseType_t status = xQueueSendToBack(publish_queue, &published_value, 0);
        if (status != pdPASS)
        {
            ESP_LOGE(TAG, "Could not send to queue");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Invalid value ID %d", value_id);
    }
}

QueueHandle_t publish_init(unsigned int queue_depth, UBaseType_t priority)
{
    assert(sizeof(values_info) / sizeof(values_info[0]) == PUBLISH_VALUE_LAST);

    // Create a queue for the sensor task to publish sensor readings
    // (Priority of sending task should be lower than receiving task)
    QueueHandle_t publish_queue = xQueueCreate(queue_depth, sizeof(published_value_t));
    xTaskCreate(&publish_task, "publish_task", 4096, publish_queue, priority, NULL);
    return publish_queue;
}


