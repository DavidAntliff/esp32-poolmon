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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "wifi_support.h"
#include "resources.h"
#include "utils.h"
#include "datastore/datastore.h"

#define TAG "wifi_support"
#define CHECK_PERIOD (1000) // milliseconds

typedef struct
{
    const datastore_t * datastore;
} task_inputs_t;

static TaskHandle_t _task_handle = NULL;

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    const datastore_t * datastore = (const datastore_t *)ctx;

    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            datastore_set_uint32(datastore, RESOURCE_ID_WIFI_STATUS, 0, WIFI_STATUS_DISCONNECTED);
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected");
            datastore_set_uint32(datastore, RESOURCE_ID_WIFI_STATUS, 0, WIFI_STATUS_CONNECTED);
            datastore_increment(datastore, RESOURCE_ID_WIFI_CONNECTION_COUNT, 0);
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "WiFi got IP");
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            datastore_set_uint32(datastore, RESOURCE_ID_WIFI_STATUS, 0, WIFI_STATUS_GOT_ADDRESS);
            datastore_set_uint32(datastore, RESOURCE_ID_WIFI_TIMESTAMP, 0, seconds_since_boot());
            datastore_set_uint32(datastore, RESOURCE_ID_WIFI_ADDRESS, 0, event->event_info.got_ip.ip_info.ip.addr);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            // This is a workaround as ESP32 WiFi libs don't currently auto-reassociate.
            ESP_LOGI(TAG, "WiFi disconnected");
            datastore_set_uint32(datastore, RESOURCE_ID_WIFI_STATUS, 0, WIFI_STATUS_DISCONNECTED);
            datastore_set_uint32(datastore, RESOURCE_ID_WIFI_ADDRESS, 0, 0);
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_conn_init(const datastore_t * datastore)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, (void *)datastore));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = { 0 };
    datastore_get_string(datastore, RESOURCE_ID_WIFI_SSID, 0, (char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid));
    datastore_get_string(datastore, RESOURCE_ID_WIFI_PASSWORD, 0, (char *)wifi_config.sta.password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG":wifi", "start the WIFI SSID:[%s] password:[%s]", wifi_config.sta.ssid, "******");
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_monitor_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());
    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    const datastore_t * datastore = task_inputs->datastore;

    bool new_info = true;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        last_wake_time = xTaskGetTickCount();

        wifi_ap_record_t ap_info = { 0 };
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
        {
            if (new_info)
            {
                ESP_LOGI(TAG, "AP SSID %s", ap_info.ssid);
                ESP_LOGI(TAG, "AP primary channel %d", ap_info.primary);
                ESP_LOGI(TAG, "AP secondary channel %d", ap_info.second);
                ESP_LOGI(TAG, "802.11%s%s%s", ap_info.phy_11b ? "b" : "", ap_info.phy_11g ? "g" : "", ap_info.phy_11n ? "n" : "");
                ESP_LOGI(TAG, "RSSI %d", ap_info.rssi);
                new_info = false;

            }
            else
            {
                ESP_LOGD(TAG, "RSSI %d", ap_info.rssi);
            }

            datastore_set_int8(datastore, RESOURCE_ID_WIFI_RSSI, 0, ap_info.rssi);
        }
        else
        {
            new_info = true;
            datastore_set_int8(datastore, RESOURCE_ID_WIFI_RSSI, 0, 0);
        }

        vTaskDelayUntil(&last_wake_time, CHECK_PERIOD / portTICK_PERIOD_MS);
    }

    free(task_inputs);
    _task_handle = NULL;
    vTaskDelete(NULL);
}

void wifi_support_init(UBaseType_t priority, const datastore_t * datastore)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    wifi_conn_init(datastore);

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->datastore = datastore;
        xTaskCreate(&wifi_monitor_task, "wifi_monitor_task", 4096, task_inputs, priority, &_task_handle);
    }
}

void wifi_support_delete(void)
{
    // disable the event handler
    esp_event_loop_set_cb(NULL, NULL);

    if (_task_handle)
        vTaskDelete(_task_handle);
}
