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

// publish sensor readings
static void publish_task(void * pvParameter)
{
    QueueHandle_t publish_queue = (QueueHandle_t)pvParameter;
    ESP_LOGW(TAG":publish_task", "Core ID %d", xPortGetCoreID());

    while (1)
    {
        // Not necessarily true when using multiple cores:
        //if (uxQueueMessagesWaiting(publish_queue) != 0)
        //{
        //    ESP_LOGE(TAG":mqtt", "Queue should be empty!\n");
        //}

        SensorReading reading = {0};
        BaseType_t sensor_queue_status = xQueueReceive(publish_queue, &reading, portMAX_DELAY);
        if (sensor_queue_status == pdPASS)
        {
            // TODO: we need a more reliable way to know that the client is valid,
            // as attempting to publish when the client has died causes a crash.
            if (g_client != NULL)
            {
                ESP_LOGD(TAG":publish_task", "Received %d:%f", reading.sensor_id, reading.value);
                char topic[64] = {0};
                char value[8] = {0};
                snprintf(topic, 64-1, "poolmon/sensor/%d", reading.sensor_id);
                snprintf(value, 8-1, "%.3f", reading.value);
                ESP_LOGI(TAG":publish_task", "Publish %s %s", topic, value);
                mqtt_publish(g_client, topic, value, strlen(value), 0, 0);
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

QueueHandle_t publish_init(unsigned int queue_depth, UBaseType_t priority)
{
    // Create a queue for the sensor task to publish sensor readings
    // (Priority of sending task should be lower than receiving task)
    QueueHandle_t publish_queue = xQueueCreate(queue_depth, sizeof(SensorReading));
    xTaskCreate(&publish_task, "publish_task", 4096, publish_queue, priority, NULL);
    return publish_queue;
}


