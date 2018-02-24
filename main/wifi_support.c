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
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "wifi_support.h"
#include "resources.h"
#include "esp_mqtt.h"
#include "datastore/datastore.h"

#define TAG "wifi_support"

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi connected");
            datastore_set_uint32(g_datastore, RESOURCE_ID_WIFI_STATUS, 0, WIFI_STATUS_CONNECTED);
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "WiFi got IP");
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            datastore_set_uint32(g_datastore, RESOURCE_ID_WIFI_STATUS, 0, WIFI_STATUS_GOT_ADDRESS);
            datastore_set_uint32(g_datastore, RESOURCE_ID_WIFI_ADDRESS, 0, event->event_info.got_ip.ip_info.ip.addr);
            esp_mqtt_start(CONFIG_MQTT_BROKER_IP_ADDRESS, CONFIG_MQTT_BROKER_TCP_PORT, "esp-mqtt", "username", "password");
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            // This is a workaround as ESP32 WiFi libs don't currently auto-reassociate.
            ESP_LOGI(TAG, "WiFi disconnected");
            datastore_set_uint32(g_datastore, RESOURCE_ID_WIFI_STATUS, 0, WIFI_STATUS_DISCONNECTED);
            datastore_set_uint32(g_datastore, RESOURCE_ID_WIFI_ADDRESS, 0, 0);
            esp_mqtt_stop();
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_conn_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = { 0 };
    datastore_get_string(g_datastore, RESOURCE_ID_WIFI_SSID, 0, (char *)wifi_config.sta.ssid);
    datastore_get_string(g_datastore, RESOURCE_ID_WIFI_PASSWORD, 0, (char *)wifi_config.sta.password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG":wifi", "start the WIFI SSID:[%s] password:[%s]", wifi_config.sta.ssid, "******");
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_monitor_task(void * pvParameter)
{
    //assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    bool new_info = true;

    while (1)
    {
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

            datastore_set_int8(g_datastore, RESOURCE_ID_WIFI_RSSI, 0, ap_info.rssi);
        }
        else
        {
            new_info = true;
            datastore_set_int8(g_datastore, RESOURCE_ID_WIFI_RSSI, 0, 0);
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void wifi_support_init(UBaseType_t priority)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    wifi_conn_init();

    xTaskCreate(&wifi_monitor_task, "wifi_monitor_task", 4096, NULL, priority, NULL);
}
