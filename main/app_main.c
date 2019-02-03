/*
 * MIT License
 *
 * Copyright (c) 2017-2018 David Antliff
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
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "soc/rtc.h"

// memory monitoring
#include "esp_system.h"
#include "esp_heap_caps.h"

#include "constants.h"
#include "resources.h"
#include "utils.h"
#include "led.h"
#include "i2c_master.h"
#include "sensor_temp.h"
#include "sensor_flow.h"
#include "sensor_light.h"
#include "publish.h"
#include "nvs_support.h"
#include "wifi_support.h"
#include "mqtt.h"
#include "subscriptions.h"
#include "avr_support.h"
#include "display.h"
#include "power.h"
#include "control.h"
#include "system_monitor.h"
#include "sntp_rtc.h"
#include "datastore/datastore.h"
#include "ota.h"

#define TAG "app_main"

#define MARK_PERIOD (60 * 10)  // emit something in the log every 10 minutes

#ifdef __GNUC__
#  define SYMBOL_IS_NOT_USED __attribute__ ((unused))
#else
#  define SYMBOL_IS_NOT_USED
#endif


// brief delay during startup sequence
static void _delay(void)
{
    vTaskDelay(100 / portTICK_RATE_MS);
}

static void SYMBOL_IS_NOT_USED avr_test_sequence(void)
{
    static int state = 0;

    // simple state machine for testing
    switch (state)
    {
    case 0:
        avr_support_set_cp_pump(AVR_PUMP_STATE_OFF);
        avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
        avr_support_set_alarm(AVR_ALARM_STATE_OFF);
        break;
    case 1:
        avr_support_set_alarm(AVR_ALARM_STATE_ON);
        break;
    case 2:
        avr_support_set_cp_pump(AVR_PUMP_STATE_ON);
        break;
    case 3:
        avr_support_set_cp_pump(AVR_PUMP_STATE_OFF);
        avr_support_set_pp_pump(AVR_PUMP_STATE_ON);
        break;
    case 4:
        avr_support_set_cp_pump(AVR_PUMP_STATE_ON);
        avr_support_set_alarm(AVR_ALARM_STATE_OFF);
        break;
    case 5:
        avr_support_set_cp_pump(AVR_PUMP_STATE_OFF);
        break;
    case 6:
        avr_support_set_cp_pump(AVR_PUMP_STATE_ON);
        avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
        break;
    default:
        state = 0;
        break;
    }
    state = (state + 1) % 7;
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_WARN);
    //esp_log_level_set("*", ESP_LOG_INFO);
    //esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set("nvs_support", ESP_LOG_INFO);
    esp_log_level_set("display", ESP_LOG_WARN);
    esp_log_level_set("resources", ESP_LOG_INFO);
    esp_log_level_set("i2c", ESP_LOG_INFO);
    esp_log_level_set("avr_support", ESP_LOG_INFO);
    esp_log_level_set("wifi_support", ESP_LOG_INFO);
    esp_log_level_set("datastore", ESP_LOG_INFO);
    esp_log_level_set("mqtt", ESP_LOG_INFO);
    esp_log_level_set("publish", ESP_LOG_INFO);
    esp_log_level_set("sensor_temp", ESP_LOG_WARN);
    esp_log_level_set("sensor_light", ESP_LOG_WARN);
    esp_log_level_set("i2c-lcd1602", ESP_LOG_INFO);   // debug is too verbose
    esp_log_level_set("control", ESP_LOG_INFO);
    esp_log_level_set("sntp_rtc", ESP_LOG_INFO);
    esp_log_level_set("power", ESP_LOG_WARN);
    esp_log_level_set("app_main", ESP_LOG_INFO);
    esp_log_level_set("subscriptions", ESP_LOG_INFO);
    esp_log_level_set("ota", ESP_LOG_INFO);

    // Ensure RMT peripheral is reset properly, in case of prior crash
    periph_module_disable(PERIPH_RMT_MODULE);
    periph_module_enable(PERIPH_RMT_MODULE);

    // Priority of queue consumer should be higher than producers
    UBaseType_t publish_priority = CONFIG_ESP_MQTT_TASK_STACK_PRIORITY;
    UBaseType_t display_priority = publish_priority - 1;
    UBaseType_t sensor_priority = publish_priority - 1;
    UBaseType_t avr_priority = sensor_priority;
    UBaseType_t wifi_monitor_priority = sensor_priority;
    UBaseType_t control_priority = sensor_priority;
    UBaseType_t system_priority = publish_priority;
    UBaseType_t ota_priority = publish_priority + 1;

    // round to nearest MHz (stored value is only precise to MHz)
    uint32_t apb_freq = (rtc_clk_apb_freq_get() + 500000) / 1000000 * 1000000;
    ESP_LOGI(TAG, "APB CLK %u Hz", apb_freq);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    // initialise NVS
    ESP_ERROR_CHECK(nvs_support_init(NVS_NAMESPACE_RESOURCES));

    datastore_t * datastore = resources_init();
    resources_load(datastore);

    // Onboard LED
    led_init(CONFIG_ONBOARD_LED_GPIO);

    // I2C bus
    _delay();
    i2c_master_info_t * i2c_master_info = i2c_master_init(I2C_MASTER_NUM, CONFIG_I2C_MASTER_SDA_GPIO, CONFIG_I2C_MASTER_SCL_GPIO, I2C_MASTER_FREQ_HZ);

    _delay();
    int num_i2c_devices = i2c_master_scan(i2c_master_info);
    ESP_LOGI(TAG, "%d I2C devices detected", num_i2c_devices);

    // bring up the display ASAP in case of error
    _delay();
    display_init(i2c_master_info, display_priority, datastore);

    // It works best to find all connected devices before starting WiFi, otherwise it can be unreliable.

    // Temp sensors
    _delay();
    temp_sensors_t * SYMBOL_IS_NOT_USED temp_sensors = sensor_temp_init(CONFIG_ONE_WIRE_GPIO, sensor_priority, datastore);

    // I2C devices - AVR, Light Sensor, LCD
    _delay();
    avr_support_init(i2c_master_info, avr_priority, datastore);
    avr_support_reset();

    // short beep to prove speaker works, and alert in case of a reset
    _delay();
    avr_support_set_alarm(AVR_ALARM_STATE_ON);
    vTaskDelay(500 / portTICK_RATE_MS);
    avr_support_set_alarm(AVR_ALARM_STATE_OFF);

    _delay();
    sensor_light_init(i2c_master_info, sensor_priority, datastore);

    // Flow Meter
    _delay();
    sensor_flow_init(CONFIG_FLOW_METER_PULSE_GPIO, FLOW_METER_PCNT_UNIT, FLOW_METER_PCNT_CHANNEL,
                     CONFIG_FLOW_METER_RMT_GPIO, FLOW_METER_RMT_CHANNEL, FLOW_METER_RMT_CLK_DIV,
                     FLOW_METER_SAMPLING_PERIOD, FLOW_METER_SAMPLING_WINDOW, FLOW_METER_FILTER_LENGTH, sensor_priority, datastore);

    bool running = true;

    mqtt_info_t * mqtt_info = mqtt_malloc();

    mqtt_error_t mqtt_error = MQTT_ERROR_UNKNOWN;
    if ((mqtt_error = mqtt_init(mqtt_info, datastore)) != MQTT_OK)
    {
        ESP_LOGE(TAG, "mqtt_init failed: %d", mqtt_error);
    }

    _delay();
    wifi_support_init(wifi_monitor_priority, datastore);   // requires NVS to be initialised

    _delay();
    publish_context_t * publish_context = publish_init(mqtt_info, PUBLISH_QUEUE_DEPTH, publish_priority, ROOT_TOPIC);
    publish_topics_init(datastore, publish_context);

    _delay();
    power_init(sensor_priority, datastore);

    _delay();
    datastore_dump(datastore);

    TickType_t last_wake_time = xTaskGetTickCount();

    // be careful of the scope on this - it needs to persist for the duration of the program:
    subscriptions_context_t globals = {
        .mqtt_info = mqtt_info,
        .running = &running,
        .datastore = datastore,
        .publish_context = publish_context,
    };

    datastore_status_t status = datastore_add_set_callback(datastore, RESOURCE_ID_MQTT_STATUS, 0, subscriptions_init, &globals);
    if (status != DATASTORE_STATUS_OK)
    {
        ESP_LOGE(TAG, "datastore_add_set_callback for resource %d failed: %d", RESOURCE_ID_MQTT_STATUS, status);
    }

    sntp_rtc_init(wifi_monitor_priority, datastore);
    _delay();

    control_init(control_priority, datastore);
    _delay();

    system_monitor_init(system_priority, datastore, publish_context);
    _delay();

    ota_init(ota_priority, datastore);
    _delay();

    uint32_t last_mark_time = 0;

    while (running)
    {
        last_wake_time = xTaskGetTickCount();

        //avr_test_sequence();

        // period mark in log
        uint32_t now = seconds_since_boot();
        if (now >= last_mark_time + MARK_PERIOD)
        {
            ESP_LOGI(TAG, "-- mark --");
            last_mark_time = now;
        }

        // network connection state machine
        wifi_status_t wifi_status = WIFI_STATUS_DISCONNECTED;
        mqtt_status_t mqtt_status = MQTT_STATUS_DISCONNECTED;
        datastore_get_uint32(datastore, RESOURCE_ID_WIFI_STATUS, 0, &wifi_status);
        datastore_get_uint32(datastore, RESOURCE_ID_MQTT_STATUS, 0, &mqtt_status);
        ESP_LOGD(TAG, "wifi_status %d, mqtt_status %d", wifi_status, mqtt_status);

        if (wifi_status == WIFI_STATUS_DISCONNECTED)
        {
            if (mqtt_status != MQTT_STATUS_DISCONNECTED)
            {
                ESP_LOGI(TAG, "MQTT stop");
                esp_mqtt_stop();
            }
            ESP_LOGI(TAG, "WiFi connect");
            esp_wifi_connect();
        }
        else if (wifi_status == WIFI_STATUS_GOT_ADDRESS)
        {
            if (mqtt_status == MQTT_STATUS_DISCONNECTED)
            {
                ESP_LOGI(TAG, "MQTT start");
                if ((mqtt_error = mqtt_start(mqtt_info, datastore)) != MQTT_OK)
                {
                    ESP_LOGE(TAG, "mqtt_start failed: %d", mqtt_error);
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, 1000 / portTICK_RATE_MS);
    }

    datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Restarting");

    // Delete all tasks before deallocating structures that they might be using.
    ESP_LOGD(TAG, "Deleting tasks");

    // "delete" all tasks
    system_monitor_delete();
    control_delete();
    sntp_rtc_delete();
    power_delete();
    publish_delete();
    wifi_support_delete();
    sensor_flow_delete();
    sensor_light_delete();
    avr_support_delete();
    sensor_temp_delete();
    display_delete();

    sensor_temp_close(temp_sensors);
    i2c_master_close(i2c_master_info);
    datastore_free(&datastore);
    publish_free(&publish_context);

    ESP_LOGW(TAG, "Restarting...");
    vTaskDelay(1000 / portTICK_RATE_MS);
    esp_restart();
}
