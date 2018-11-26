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
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "publish.h"
#include "resources.h"

#define TAG "publish"

typedef struct
{
    mqtt_info_t * mqtt_info;
    QueueHandle_t publish_queue;
    const char * root_topic;
} task_inputs_t;

typedef void (*value_renderer)(const datastore_t * datastore, datastore_resource_id_t resource_id, datastore_instance_id_t instance_id, char * buffer, size_t buffer_size);

typedef struct
{
    datastore_resource_id_t resource_id;
    datastore_instance_id_t instance_id;
    const char * topic;
    value_renderer renderer;
} value_info_t;

static void _as_string(const datastore_t * datastore, datastore_resource_id_t resource_id, datastore_instance_id_t instance_id, char * buffer, size_t buffer_size)
{
    datastore_get_as_string(datastore, resource_id, instance_id, buffer, buffer_size);
}

static void _as_ipv4_address(const datastore_t * datastore, datastore_resource_id_t resource_id, datastore_instance_id_t instance_id, char * buffer, size_t buffer_size)
{
    uint32_t ipv4_address = 0;
    datastore_get_uint32(datastore, resource_id, instance_id, &ipv4_address);
    snprintf(buffer, buffer_size, "%d.%d.%d.%d",
             (ipv4_address & 0xff),
             (ipv4_address & 0xff00) >> 8,
             (ipv4_address & 0xff0000) >> 16,
             (ipv4_address & 0xff000000) >> 24);
}

static const value_info_t values_info[] =
{
    { RESOURCE_ID_TEMP_VALUE, 0, "sensors/temp/1/value", _as_string },
    { RESOURCE_ID_TEMP_VALUE, 1, "sensors/temp/2/value", _as_string },
    { RESOURCE_ID_TEMP_VALUE, 2, "sensors/temp/3/value", _as_string },
    { RESOURCE_ID_TEMP_VALUE, 3, "sensors/temp/4/value", _as_string },
    { RESOURCE_ID_TEMP_VALUE, 4, "sensors/temp/5/value", _as_string },

    { RESOURCE_ID_TEMP_DETECTED, 0, "sensors/temp/1/detected", _as_string },
    { RESOURCE_ID_TEMP_DETECTED, 1, "sensors/temp/2/detected", _as_string },
    { RESOURCE_ID_TEMP_DETECTED, 2, "sensors/temp/3/detected", _as_string },
    { RESOURCE_ID_TEMP_DETECTED, 3, "sensors/temp/4/detected", _as_string },
    { RESOURCE_ID_TEMP_DETECTED, 4, "sensors/temp/5/detected", _as_string },

    { RESOURCE_ID_TEMP_ASSIGNMENT, 0, "sensors/temp/1/assignment", _as_string },
    { RESOURCE_ID_TEMP_ASSIGNMENT, 1, "sensors/temp/2/assignment", _as_string },
    { RESOURCE_ID_TEMP_ASSIGNMENT, 2, "sensors/temp/3/assignment", _as_string },
    { RESOURCE_ID_TEMP_ASSIGNMENT, 3, "sensors/temp/4/assignment", _as_string },
    { RESOURCE_ID_TEMP_ASSIGNMENT, 4, "sensors/temp/5/assignment", _as_string },

    { RESOURCE_ID_LIGHT_FULL,        0, "sensors/light/1/full_spectrum", _as_string },
    { RESOURCE_ID_LIGHT_VISIBLE,     0, "sensors/light/1/visible",       _as_string },
    { RESOURCE_ID_LIGHT_INFRARED,    0, "sensors/light/1/infrared",      _as_string },
    { RESOURCE_ID_LIGHT_ILLUMINANCE, 0, "sensors/light/1/lux",           _as_string },

    { RESOURCE_ID_FLOW_FREQUENCY, 0, "sensors/flow/1/freq", _as_string },
    { RESOURCE_ID_FLOW_RATE,      0, "sensors/flow/1/rate", _as_string },

    { RESOURCE_ID_POWER_VALUE,      0, "power/value", _as_string },
    { RESOURCE_ID_POWER_TEMP_DELTA, 0, "power/delta", _as_string },

    { RESOURCE_ID_SWITCHES_CP_MODE_VALUE, 0, "switches/cp/mode",   _as_string },
    { RESOURCE_ID_SWITCHES_CP_MAN_VALUE,  0, "switches/cp/manual", _as_string },
    { RESOURCE_ID_SWITCHES_PP_MODE_VALUE, 0, "switches/pp/mode",   _as_string },
    { RESOURCE_ID_SWITCHES_PP_MAN_VALUE,  0, "switches/pp/manual", _as_string },

    { RESOURCE_ID_PUMPS_CP_STATE, 0, "pumps/cp/state", _as_string },
    { RESOURCE_ID_PUMPS_PP_STATE, 0, "pumps/pp/state", _as_string },

    { RESOURCE_ID_WIFI_ADDRESS, 0, "wifi/address", _as_ipv4_address },
    { RESOURCE_ID_SYSTEM_LOG, 0, "device/log", _as_string },

//    { RESOURCE_ID_ALARM_STATE, 0, "alarms/1/state", },
};

void publish_topics_init(const datastore_t * datastore, publish_context_t * publish_context)
{
    for (size_t i = 0; i < sizeof(values_info) / sizeof(values_info[0]); ++i)
    {
        datastore_status_t status;
        if ((status = datastore_add_set_callback(datastore, values_info[i].resource_id, publish_callback, publish_context)) != DATASTORE_STATUS_OK)
        {
            ESP_LOGE(TAG, "datastore_add_set_callback for resource %d failed: %d", values_info[i].resource_id, status);
        }
    }
}

typedef struct
{
    const datastore_t * datastore;
    datastore_resource_id_t resource_id;
    datastore_instance_id_t instance_id;
} publish_request_t;

static void process_request(const publish_request_t * request, const char * root_topic)
{
    ESP_LOGD(TAG, "Received request: id %d, name %s, instance %d", request->resource_id, datastore_get_name(request->datastore, request->resource_id), request->instance_id);

    // check if MQTT is ready before attempting to send
    mqtt_status_t mqtt_status = MQTT_STATUS_DISCONNECTED;
    if (datastore_get_uint32(request->datastore, RESOURCE_ID_MQTT_STATUS, 0, &mqtt_status) == DATASTORE_STATUS_OK)
    {
        if (mqtt_status == MQTT_STATUS_CONNECTED)
        {
            // find the topic
            bool found = false;
            for (size_t i = 0; found == false && i < sizeof(values_info) / sizeof(values_info[0]); ++i)
            {
                if (values_info[i].resource_id == request->resource_id
                    && values_info[i].instance_id == request->instance_id)
                {
                    // retrieve value as string
                    found = true;
                    char topic[64] = "";
                    char value_string[256] = "";
                    snprintf(topic, sizeof(topic) - 1, "%s/%s", root_topic, values_info[i].topic);
                    if (values_info[i].renderer != NULL)
                    {
                        values_info[i].renderer(request->datastore, request->resource_id, request->instance_id, value_string, sizeof(value_string));
                        //datastore_get_as_string(request->datastore, request->resource_id, request->instance_id, value_string, sizeof(value_string));
                        size_t value_size = strlen(value_string);
                        ESP_LOGD(TAG, "Topic %s, value \"%s\" [%d bytes]", topic, value_string, value_size);
                        mqtt_publish(topic, (uint8_t *)value_string, strlen(value_string) + 1, 0, false);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "No value renderer for ID %d\n", request->resource_id);
                    }
                }
            }

            if (found == false)
            {
                ESP_LOGW(TAG, "Request id %d, name %s, instance %d not published", request->resource_id, datastore_get_name(request->datastore, request->resource_id), request->instance_id);
            }
        }
        else
        {
            ESP_LOGW(TAG, "MQTT not connected - discarding request id %d, name %s, instance %d", request->resource_id, datastore_get_name(request->datastore, request->resource_id), request->instance_id);
        }
    }
}

// publish sensor readings
static void publish_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());
    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    QueueHandle_t publish_queue = task_inputs->publish_queue;
    //mqtt_info_t * mqtt_info = task_inputs->mqtt_info;
    const char * root_topic = task_inputs->root_topic;

    while (1)
    {
        publish_request_t request = {0};
        BaseType_t sensor_queue_status = xQueueReceive(publish_queue, &request, portMAX_DELAY);
        if (sensor_queue_status == pdPASS)
        {
            process_request(&request, root_topic);
        }
        else
        {
            ESP_LOGE(TAG":publish_task", "Could not receive from queue");
        }
    }

    vTaskDelete(NULL);
}

void publish_resource(const publish_context_t * publish_context, const datastore_t * datastore, datastore_resource_id_t id, datastore_instance_id_t instance)
{
    // push the datastore ID and instance to the queue so that the task can send an MQTT update
    ESP_LOGD(TAG, "publish_resource: context %p, datastore %p, resource id %d, instance id %d", publish_context, datastore, id, instance);
    if (publish_context != NULL)
    {
        QueueHandle_t publish_queue = publish_context->queue;
        if (datastore != NULL)
        {
            publish_request_t request = {
                .datastore = datastore,
                .resource_id = id,
                .instance_id = instance,
            };
            BaseType_t status = xQueueSendToBack(publish_queue, &request, 0);
            if (status != pdPASS)
            {
                ESP_LOGE(TAG, "Could not send resource id %d, instance %d to queue: status %d", id, instance, status);
            }
        }
    }
    else
    {
        ESP_LOGE(TAG, "publish_context is NULL");
    }
}

void publish_direct(const publish_context_t * publish_context, const char * topic, const uint8_t * data, size_t length)
{
    if (publish_context != NULL)
    {
        mqtt_publish(topic, data, length, 0, false);
    }
    else
    {
        ESP_LOGE(TAG, "publish_context is NULL");
    }
}

void publish_callback(const datastore_t * datastore, datastore_resource_id_t id, datastore_instance_id_t instance, void * context)
{
    ESP_LOGD(TAG, "publish_callback: datastore %p, resource id %d, instance id %d, context %p", datastore, id, instance, context);
    publish_context_t * publish_context = (publish_context_t *)context;
    publish_resource(publish_context, datastore, id, instance);
}

publish_context_t * publish_init(mqtt_info_t * mqtt_info, unsigned int queue_depth, UBaseType_t priority, const char * root_topic)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // Create a queue for the sensor task to publish sensor readings
    // (Priority of sending task should be lower than receiving task)
    QueueHandle_t publish_queue = xQueueCreate(queue_depth, sizeof(publish_request_t));

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->mqtt_info = mqtt_info;
        task_inputs->publish_queue = publish_queue;
        task_inputs->root_topic = root_topic;
        xTaskCreate(&publish_task, "publish_task", 4096, task_inputs, priority, NULL);
    }

    publish_context_t * publish_context = malloc(sizeof(*publish_context));
    if (publish_context != NULL)
    {
        memset(publish_context, 0, sizeof(*publish_context));
        publish_context->queue = publish_queue;
    }

    return publish_context;
}

void publish_free(publish_context_t ** publish_context)
{
    if (publish_context != NULL && (*publish_context != NULL))
    {
        free(*publish_context);
        *publish_context = NULL;
    }
}

