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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/pcnt.h"

#include "sensor_flow.h"
#include "constants.h"
#include "resources.h"
#include "publish.h"
#include "datastore/datastore.h"

#define TAG "sensor_flow"

typedef struct
{
    uint8_t pcnt_gpio;         // count events on this GPIO
    pcnt_unit_t pcnt_unit;     // PCNT unit to use for counting
    pcnt_channel_t pcnt_channel;  // PCNT channel to use for counting
    uint8_t rmt_gpio;          // used by RMT to define a sampling window
    rmt_channel_t rmt_channel; // The RMT channel to use
    uint8_t rmt_clk_div;       // RMT pulse length, as a divider of the APB clock
    float sampling_period;     // time (in seconds) between subsequent samples
    float sampling_window;     // sample window length (in seconds)
    uint16_t filter_length;    // counter filter length in APB cycles
    QueueHandle_t publish_queue;
} task_inputs_t;

static void init_rmt(uint8_t tx_gpio, rmt_channel_t channel, uint8_t clk_div)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    rmt_config_t rmt_tx = {
        .rmt_mode = RMT_MODE_TX,
        .channel = channel,
        .gpio_num = tx_gpio,
        .mem_block_num = 1,  // single block
        .clk_div = clk_div,
        .tx_config.loop_en = false,
        .tx_config.carrier_en = false,
        .tx_config.idle_level = RMT_IDLE_LEVEL_LOW,
        .tx_config.idle_output_en = true,
    };
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

static int create_rmt_window(rmt_item32_t * items, double sampling_window, double rmt_period)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    int num_items = 0;

    // enable counter for exactly x seconds:
    int32_t total_duration = (uint32_t)(sampling_window / rmt_period);
    ESP_LOGI(TAG, "total_duration %d periods", total_duration);

    // max duration per item is 2^15-1 = 32767
    while (total_duration > 0)
    {
        uint32_t duration = total_duration > 32767 ? 32767 : total_duration;
        items[num_items].level0 = 1;
        items[num_items].duration0 = duration;
        total_duration -= duration;
        //ESP_LOGI(TAG, "duration %d", duration);

        if (total_duration > 0)
        {
            uint32_t duration = total_duration > 32767 ? 32767 : total_duration;
            items[num_items].level1 = 1;
            items[num_items].duration1 = duration;
            total_duration -= duration;
        }
        else
        {
            items[num_items].level1 = 0;
            items[num_items].duration1 = 0;
        }
        //ESP_LOGI(TAG, "[%d].level0 %d", num_items, items[num_items].level0);
        //ESP_LOGI(TAG, "[%d].duration0 %d", num_items, items[num_items].duration0);
        //ESP_LOGI(TAG, "[%d].level1 %d", num_items, items[num_items].level1);
        //ESP_LOGI(TAG, "[%d].duration1 %d", num_items, items[num_items].duration1);

        ++num_items;
    }
    //ESP_LOGI(TAG, "num_items %d", num_items);
    return num_items;
}

static void init_pcnt(uint8_t pulse_gpio, uint8_t ctrl_gpio, pcnt_unit_t unit, pcnt_channel_t channel, uint16_t filter_length)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // set up counter
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pulse_gpio,
        .ctrl_gpio_num = ctrl_gpio,
        .lctrl_mode = PCNT_MODE_DISABLE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,  // count both rising and falling edges
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = 0,
        .counter_l_lim = 0,
        .unit = unit,
        .channel = channel,
    };

    pcnt_unit_config(&pcnt_config);

    // set the GPIO back to high-impedance, as pcnt_unit_config sets it as pull-up
    gpio_set_pull_mode(pulse_gpio, GPIO_FLOATING);

    // enable counter filter - at 80MHz APB CLK, 1000 pulses is max 80,000 Hz, so ignore pulses less than 12.5 us.
    pcnt_set_filter_value(unit, filter_length);
    pcnt_filter_enable(unit);
}

// Hz => LPM
#define FLOW_RATE_LPM(X) (FLOW_METER_MODEL_A * (X) + FLOW_METER_MODEL_B)

static double calc_flow_rate_lpm(double hz)
{
    double rate_lpm = 0.0;

    if (hz >= 0.0)
    {
        if (hz < FLOW_METER_MODEL_CUTOFF_HZ)
        {
            // interpolate between cutoff LPM and zero
            rate_lpm = hz / FLOW_METER_MODEL_CUTOFF_HZ * FLOW_RATE_LPM(FLOW_METER_MODEL_CUTOFF_HZ);
        }
        else
        {
            rate_lpm = FLOW_RATE_LPM(hz);
        }
    }
    else
    {
        ESP_LOGE(TAG, "invalid flow frequency %f", hz);
    }
    return rate_lpm;
}

static void sensor_flow_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;

    init_rmt(task_inputs->rmt_gpio, task_inputs->rmt_channel, task_inputs->rmt_clk_div);
    init_pcnt(task_inputs->pcnt_gpio, task_inputs->rmt_gpio, task_inputs->pcnt_unit, task_inputs->pcnt_channel, task_inputs->filter_length);

    // assuming 80MHz APB clock
    const double rmt_period = (double)(task_inputs->rmt_clk_div) / 80000000.0;

    rmt_item32_t rmt_items[RMT_MEM_ITEM_NUM] = { 0 };
    int num_rmt_items = create_rmt_window(rmt_items, task_inputs->sampling_window, rmt_period);
    assert(num_rmt_items < RMT_MEM_ITEM_NUM);

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        last_wake_time = xTaskGetTickCount();

        // clear counter
        pcnt_counter_clear(task_inputs->pcnt_unit);

        // start sampling window
        rmt_write_items(task_inputs->rmt_channel, rmt_items, num_rmt_items, false);

        // wait for window to finish
        rmt_wait_tx_done(task_inputs->rmt_channel, portMAX_DELAY);

        // read counter
        int16_t count = 0;
        pcnt_get_counter_value(task_inputs->pcnt_unit, &count);

        // TODO: check for overflow?

        double frequency_hz = count / 2.0 / task_inputs->sampling_window;
        double rate_lpm = calc_flow_rate_lpm(frequency_hz);

        publish_value(PUBLISH_VALUE_FLOW_FREQ, frequency_hz, task_inputs->publish_queue);
        publish_value(PUBLISH_VALUE_FLOW_RATE, rate_lpm, task_inputs->publish_queue);

        datastore_set_float(g_datastore, RESOURCE_ID_FLOW_FREQUENCY, 0, frequency_hz);
        datastore_set_float(g_datastore, RESOURCE_ID_FLOW_RATE, 0, rate_lpm);

        ESP_LOGI(TAG, "counter %d, frequency %f Hz, rate %f LPM", count, frequency_hz, rate_lpm);

        vTaskDelayUntil(&last_wake_time, task_inputs->sampling_period * 1000 / portTICK_PERIOD_MS);
    }

    free(task_inputs);
    vTaskDelete(NULL);
}

void sensor_flow_init(uint8_t pcnt_gpio, pcnt_unit_t pcnt_unit, pcnt_channel_t pcnt_channel,
                      uint8_t rmt_gpio, rmt_channel_t rmt_channel, uint8_t rmt_clk_div,
                      float sampling_period, float sampling_window, uint16_t filter_length, UBaseType_t priority, QueueHandle_t publish_queue)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->pcnt_gpio = pcnt_gpio;
        task_inputs->pcnt_unit = pcnt_unit;
        task_inputs->pcnt_channel = pcnt_channel;
        task_inputs->rmt_gpio = rmt_gpio;
        task_inputs->rmt_channel = rmt_channel;
        task_inputs->rmt_clk_div = rmt_clk_div;
        task_inputs->sampling_period = sampling_period;
        task_inputs->sampling_window = sampling_window;
        task_inputs->filter_length = filter_length;
        task_inputs->publish_queue = publish_queue;
        xTaskCreate(&sensor_flow_task, "sensor_flow_task", 4096, task_inputs, priority, NULL);
    }
}

