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
#include "driver/gpio.h"

#include "sensor_flow.h"
#include "constants.h"
#include "resources.h"
#include "publish.h"
#include "utils.h"
#include "datastore/datastore.h"
#include "display.h"

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
    const datastore_t * datastore;
} task_inputs_t;

static TaskHandle_t _task_handle = NULL;

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
#define FLOW_RATE_LPM(X, A, B) ((A) * (X) + (B))

// LPM = a * Hz + b
// For Hz values less than cutoff, interpolate to zero
static double calc_flow_rate_lpm(double hz, double a, double b)
{
    double rate_lpm = 0.0;

    if (hz >= 0.0)
    {
        if (hz < FLOW_METER_MODEL_CUTOFF_HZ)
        {
            // interpolate between cutoff LPM and zero
            rate_lpm = hz / FLOW_METER_MODEL_CUTOFF_HZ * FLOW_RATE_LPM(FLOW_METER_MODEL_CUTOFF_HZ, a, b);
        }
        else
        {
            rate_lpm = FLOW_RATE_LPM(hz, a, b);
        }
    }
    else
    {
        ESP_LOGE(TAG, "invalid flow frequency %f", hz);
    }
    return rate_lpm;
}

static void _display_page_changed(const datastore_t * datastore, datastore_resource_id_t id, datastore_instance_id_t instance, void * context)
{
    display_page_id_t page = DISPLAY_PAGE_IGNORE;
    if (datastore_get_int32(datastore, RESOURCE_ID_DISPLAY_PAGE, instance, &page) == DATASTORE_STATUS_OK)
    {
        if (page == DISPLAY_PAGE_SENSORS_FLOW)
        {
            gpio_matrix_out(CONFIG_ONBOARD_LED_GPIO, SIG_IN_FUNC228_IDX, false, false);
            gpio_matrix_in(CONFIG_FLOW_METER_PULSE_GPIO, SIG_IN_FUNC228_IDX, false);
        }
        else
        {
            gpio_matrix_out(CONFIG_ONBOARD_LED_GPIO, SIG_GPIO_OUT_IDX, false, false);
            gpio_matrix_in(GPIO_CONSTANT_LOW, SIG_IN_FUNC228_IDX, false);
        }
    }
}

static void sensor_flow_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    const datastore_t * datastore = task_inputs->datastore;

    init_rmt(task_inputs->rmt_gpio, task_inputs->rmt_channel, task_inputs->rmt_clk_div);
    init_pcnt(task_inputs->pcnt_gpio, task_inputs->rmt_gpio, task_inputs->pcnt_unit, task_inputs->pcnt_channel, task_inputs->filter_length);

    // assuming 80MHz APB clock
    const double rmt_period = (double)(task_inputs->rmt_clk_div) / 80000000.0;

    rmt_item32_t rmt_items[RMT_MEM_ITEM_NUM] = { 0 };
    int num_rmt_items = create_rmt_window(rmt_items, task_inputs->sampling_window, rmt_period);
    assert(num_rmt_items < RMT_MEM_ITEM_NUM);

    TickType_t last_wake_time = xTaskGetTickCount();

    // subscribe to display changes
    datastore_add_set_callback(datastore, RESOURCE_ID_DISPLAY_PAGE, 0, _display_page_changed, NULL);

    float flow_model_a = 0.0;
    float flow_model_b = 0.0;
    datastore_get_float(datastore, RESOURCE_ID_FLOW_MODEL_A, 0, &flow_model_a);
    datastore_get_float(datastore, RESOURCE_ID_FLOW_MODEL_B, 0, &flow_model_b);

    ESP_LOGD(TAG, "flow coefficients: A %f, B %f", flow_model_a, flow_model_b);

    while (1)
    {
        // clear counter
        pcnt_counter_clear(task_inputs->pcnt_unit);

        // start sampling window
        rmt_write_items(task_inputs->rmt_channel, rmt_items, num_rmt_items, false);

        // wait for window to finish
        rmt_wait_tx_done(task_inputs->rmt_channel, portMAX_DELAY);

        // read counter
        int16_t count = 0;
        pcnt_get_counter_value(task_inputs->pcnt_unit, &count);

        double frequency_hz = 0.0;
        double rate_lpm = 0.0;
        bool override = false;

        datastore_age_t override_age = 0;
        datastore_get_age(datastore, RESOURCE_ID_FLOW_RATE_OVERRIDE, 0, &override_age);
        override = override_age < (esp_timer_get_time() - 10);
        if (override)
        {
            float override_value = 0.0f;
            datastore_get_float(datastore, RESOURCE_ID_FLOW_RATE_OVERRIDE, 0, &override_value);
            frequency_hz = -1.0;  // to indicate override
            rate_lpm = override_value;
        }
        else
        {
            // TODO: check for overflow?
            frequency_hz = count / 2.0 / task_inputs->sampling_window;
            rate_lpm = calc_flow_rate_lpm(frequency_hz, flow_model_a, flow_model_b);
        }

        datastore_set_float(datastore, RESOURCE_ID_FLOW_FREQUENCY, 0, frequency_hz);
        datastore_set_float(datastore, RESOURCE_ID_FLOW_RATE, 0, rate_lpm);
        ESP_LOGI(TAG, "counter %d, frequency %f Hz, rate %f LPM%s", count, frequency_hz, rate_lpm, override ? " OVERRIDE" : "");

        vTaskDelayUntil(&last_wake_time, task_inputs->sampling_period * 1000 / portTICK_PERIOD_MS);
    }

    free(task_inputs);
    _task_handle = NULL;
    vTaskDelete(NULL);
}

void sensor_flow_init(uint8_t pcnt_gpio, pcnt_unit_t pcnt_unit, pcnt_channel_t pcnt_channel,
                      uint8_t rmt_gpio, rmt_channel_t rmt_channel, uint8_t rmt_clk_div,
                      float sampling_period, float sampling_window, uint16_t filter_length, UBaseType_t priority, const datastore_t * datastore)
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
        task_inputs->datastore = datastore;
        xTaskCreate(&sensor_flow_task, "sensor_flow_task", 4096, task_inputs, priority, &_task_handle);
    }
}

void sensor_flow_delete(void)
{
    if (_task_handle)
        vTaskDelete(_task_handle);
}
