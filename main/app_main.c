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

#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_log.h"
//#include "nvs_flash.h"
#include "sdkconfig.h"

#include "led.h"
#include "mqtt.h"
#include "sensor_temp.h"
#include "sensor_flow.h"
#include "publish.h"
#include "wifi_support.h"
#include "hardware.h"

#include "c_timeutils.h"

#define PUBLISH_QUEUE_DEPTH  (16)

#define TAG "poolmon"

#define ESP_INTR_FLAG_DEFAULT 0

SemaphoreHandle_t xSemaphore = NULL;

// interrupt service routine
void IRAM_ATTR isr_handler(void * arg)
{
    // notify
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

extern mqtt_client * g_client;

static void button_task(void * pvParameter)
{
    struct timeval last_press = {0};
    gettimeofday(&last_press, NULL);

    for (;;) {
        // wait for notification from ISR
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            struct timeval now;
            gettimeofday(&now, NULL);
            ESP_LOGI(TAG, "ISR fired!");

            if (timeval_durationBeforeNow(&last_press) > 100)
            {
                if (g_client != NULL)
                {
                    mqtt_publish(g_client, "nodered/button", "1", 1, 0, 0);
                }
                last_press = now;
            }
            else
            {
                ESP_LOGI(TAG, "Debounce");
            }
        }
    }
}


void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_LOGI(TAG, "[APP] Startup..");
    led_init(GPIO_LED);
    gpio_set_level(GPIO_LED, 1);

    // green LED init
    gpio_pad_select_gpio(GPIO_GREEN_LED);
    gpio_set_direction(GPIO_GREEN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_GREEN_LED, 1);

    // blue LED init
    gpio_pad_select_gpio(GPIO_BLUE_LED);
    gpio_set_direction(GPIO_BLUE_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_BLUE_LED, 1);

    // Priority of queue consumer should be higher than producers
    UBaseType_t publish_priority = CONFIG_MQTT_PRIORITY;
    UBaseType_t sensor_priority = publish_priority - 1;

    QueueHandle_t publish_queue = publish_init(PUBLISH_QUEUE_DEPTH, publish_priority);

    // It works best to find all connected devices before starting WiFi, otherwise it can be unreliable.
    TempSensors * temp_sensors = sensor_temp_init(GPIO_ONE_WIRE, sensor_priority, publish_queue);
    //sensor_flow_init();

    // button config
    gpio_pad_select_gpio(GPIO_BUTTON);
    gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);

    // enable interrupt on falling edge for pin
    gpio_set_intr_type(GPIO_BUTTON, GPIO_INTR_NEGEDGE);

    xSemaphore = xSemaphoreCreateBinary();
    xTaskCreate(&button_task, "button_task", 4096, NULL, 5, NULL);

    // install ISR service with default configuration
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // attach the interrupt service routine
    gpio_isr_handler_add(GPIO_BUTTON, isr_handler, NULL);

    //    nvs_flash_init();
    wifi_support_init();

    // Run forever...
    while(1)
        ;

    sensor_temp_close(temp_sensors);
    //sensor_flow_close();
}
