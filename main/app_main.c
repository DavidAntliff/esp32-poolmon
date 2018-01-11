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

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
//#include "nvs_flash.h"
#include "soc/rtc.h"

// memory monitoring
#include "esp_system.h"
#include "esp_heap_alloc_caps.h"
#include "freertos/heap_regions.h"

#include "constants.h"
#include "datastore.h"
#include "led.h"
#include "mqtt.h"
#include "i2c_master.h"
#include "sensor_temp.h"
#include "sensor_flow.h"
#include "sensor_light.h"
#include "publish.h"
#include "wifi_support.h"
#include "avr_support.h"
#include "display.h"

#define TAG "poolmon"

//TODO: LED task, to blink LED when required
// - count number of connected devices
// - indicate when sampling
// - indicate MQTT connection state
// - indicate MQTT activity (publish, receive)

// TODO: make this non-global!
datastore_t * datastore = NULL;

void app_main()
{
//    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set("display", ESP_LOG_INFO);
    esp_log_level_set("datastore", ESP_LOG_DEBUG);
    //esp_log_level_set("sensor_temp", ESP_LOG_INFO);

    // Priority of queue consumer should be higher than producers
    UBaseType_t publish_priority = CONFIG_MQTT_PRIORITY;
    UBaseType_t display_priority = publish_priority - 1;
    UBaseType_t sensor_priority = publish_priority - 1;
    UBaseType_t avr_priority = sensor_priority;

    ESP_LOGI(TAG, "[APP] Startup..");

    datastore = datastore_malloc();
    datastore_init(datastore);
    datastore_dump(datastore);

    // Onboard LED
    led_init(CONFIG_ONBOARD_LED_GPIO);

    // I2C bus
    i2c_master_info_t * i2c_master_info = i2c_master_init(I2C_MASTER_NUM, CONFIG_I2C_MASTER_SDA_GPIO, CONFIG_I2C_MASTER_SCL_GPIO, I2C_MASTER_FREQ_HZ);
    int num_i2c_devices = i2c_scan(i2c_master_info);
    ESP_LOGI(TAG, "%d I2C devices detected", num_i2c_devices);

    // bring up the display ASAP in case of error
    display_init(i2c_master_info, display_priority);

    // round to nearest MHz (stored value is only precise to MHz)
    uint32_t apb_freq = (rtc_clk_apb_freq_get() + 500000) / 1000000 * 1000000;
    ESP_LOGI(TAG, "APB CLK %u Hz", apb_freq);

    QueueHandle_t publish_queue = publish_init(PUBLISH_QUEUE_DEPTH, publish_priority);

    // It works best to find all connected devices before starting WiFi, otherwise it can be unreliable.

    // Temp sensors
    temp_sensors_t * temp_sensors = sensor_temp_init(CONFIG_ONE_WIRE_GPIO, sensor_priority, publish_queue);

    // I2C devices - AVR, Light Sensor, LCD
    avr_support_init(i2c_master_info, avr_priority, publish_queue);
    sensor_light_init(i2c_master_info, sensor_priority, publish_queue);

    // Flow Meter
    sensor_flow_init(CONFIG_FLOW_METER_PULSE_GPIO, FLOW_METER_PCNT_UNIT, FLOW_METER_PCNT_CHANNEL,
                     CONFIG_FLOW_METER_RMT_GPIO, FLOW_METER_RMT_CHANNEL, FLOW_METER_RMT_CLK_DIV,
                     FLOW_METER_SAMPLING_PERIOD, FLOW_METER_SAMPLING_WINDOW, FLOW_METER_FILTER_LENGTH, sensor_priority, publish_queue);

    //nvs_flash_init();
    //wifi_support_init();

    // Run forever...
    while (1)
    {
        ESP_LOGI(TAG, "RAM left %d", esp_get_free_heap_size());  // byte-addressable heap memory
        ESP_LOGI(TAG, "32bit aligned RAM left %d", xPortGetFreeHeapSizeTagged(MALLOC_CAP_32BIT));  // IRAM 32-bit aligned heap

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    sensor_temp_close(temp_sensors);
    i2c_master_close(i2c_master_info);
    datastore_free(&datastore);
}
