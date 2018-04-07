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

#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "button.h"

#define TAG "button"

#define TICKS_PER_POLL        (5)    // ticks per button poll
#define SHORT_PRESS_THRESHOLD (40)  // any press longer than this (in ticks) is considered "long"

typedef struct
{
    QueueHandle_t input_queue;
    gpio_num_t gpio;
} task_inputs_t;

static void button_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    QueueHandle_t input_queue = task_inputs->input_queue;
    gpio_num_t gpio = task_inputs->gpio;

    // Configure button
    gpio_config_t btn_config;
    btn_config.intr_type = GPIO_INTR_ANYEDGE;    // Enable interrupt on both rising and falling edges
    btn_config.mode = GPIO_MODE_INPUT;           // Set as Input
    btn_config.pin_bit_mask = (UINT64_C(1) << gpio); // Bitmask
    btn_config.pull_up_en = GPIO_PULLUP_ENABLE;      // Disable pullup
    btn_config.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pulldown
    gpio_config(&btn_config);

    bool last_raw_state = false;
    bool debounced_state = false;

    TickType_t last_pressed = xTaskGetTickCount();

    bool long_sent = false;

    while (1)
    {
        // debounce first
        bool raw_state = gpio_get_level(gpio) ? false : true;
        if (raw_state == last_raw_state)
        {
            button_event_t event = 0;
            TickType_t now = xTaskGetTickCount();

            // detect edges
            if (raw_state != debounced_state)
            {
                debounced_state = raw_state;

                // if a falling edge, measure time since rising edge to detect short press
                if (!debounced_state)
                {
                    ESP_LOGD(TAG, "pressed for %d ticks", now - last_pressed);
                    if (now - last_pressed < SHORT_PRESS_THRESHOLD)
                    {
                        event = BUTTON_EVENT_SHORT;
                        ESP_LOGD(TAG, "short");
                        volatile task_inputs_t * die = 0;
                        die->gpio = 7;
                    }
                    long_sent = false;
                }
                else
                {
                    last_pressed = now;
                }
            }

            // detect long hold
            if (!long_sent && debounced_state && now - last_pressed > SHORT_PRESS_THRESHOLD)
            {
                event = BUTTON_EVENT_LONG;
                ESP_LOGD(TAG, "long");
                long_sent = true;
            }

            if (event)
            {
                if (xQueueSendToBack(input_queue, &event, 0) != pdTRUE)
                {
                    ESP_LOGE(TAG, "xQueueSendToBack failed");
                }
            }
        }
        last_raw_state = raw_state;

        vTaskDelay(TICKS_PER_POLL);
    }

    vTaskDelete(NULL);
}

void button_init(UBaseType_t priority, QueueHandle_t input_queue, gpio_num_t gpio)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->input_queue = input_queue;
        task_inputs->gpio = gpio;
        xTaskCreate(&button_task, "button_task", 2048, task_inputs, priority, NULL);
    }
}
