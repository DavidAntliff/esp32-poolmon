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
#include <time.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "control.h"
#include "resources.h"
#include "avr_support.h"
#include "utils.h"
#include "datastore/datastore.h"

#define POLL_PERIOD        (1000)  // control loop period in milliseconds
#define MEASUREMENT_EXPIRY (15 * 1000000)    // microseconds
#define SENSOR_HIGH        (1)     // instance of high temperature sensor
#define SENSOR_LOW         (0)     // instance of low temperature sensor
#define PP_HOLD_OFF        (30 * 1000000)    // to check for flow when CP is on, wait at least this many seconds before deciding to start PP if flow rate is below threshold

#define PP_EMERGENCY_THRESHOLD_ON  (80)  // temperature at which to initiate emergency PP cycle
#define PP_EMERGENCY_THRESHOLD_OFF (60)  // temperature at which to terminate emergency PP cycle

#define TAG "control"

typedef struct
{
    const datastore_t * datastore;
} task_inputs_t;

static void control_cp_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());
    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    const datastore_t * datastore = task_inputs->datastore;

    TickType_t last_wake_time = xTaskGetTickCount();

    avr_pump_state_t state = AVR_PUMP_STATE_OFF;
    avr_support_set_cp_pump(AVR_PUMP_STATE_OFF);

    datastore_age_t t_high_age = DATASTORE_INVALID_AGE;
    datastore_age_t t_low_age = DATASTORE_INVALID_AGE;

    // wait for stable sensor readings
    bool stable = false;
    while (!stable)
    {
        last_wake_time = xTaskGetTickCount();
        ESP_LOGD(TAG, "CP control loop: wait for stable sensors");

        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, SENSOR_HIGH, &t_high_age);
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, SENSOR_LOW, &t_low_age);

        if ((t_high_age < MEASUREMENT_EXPIRY) && (t_low_age < MEASUREMENT_EXPIRY))
        {
            stable = true;
            ESP_LOGI(TAG, "CP control loop: sensors stable");
        }

        vTaskDelayUntil(&last_wake_time, POLL_PERIOD / portTICK_PERIOD_MS);
    }

    while (1)
    {
        last_wake_time = xTaskGetTickCount();
        ESP_LOGD(TAG, "--");
        ESP_LOGD(TAG, "CP control loop: state %d", state);

        float t_high = 0.0f;
        float t_low = 0.0f;
        bool transition = false;

        t_high_age = DATASTORE_INVALID_AGE;
        t_low_age = DATASTORE_INVALID_AGE;
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, SENSOR_HIGH, &t_high_age);
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, SENSOR_LOW, &t_low_age);

        if (t_high_age < MEASUREMENT_EXPIRY)
        {
            if (t_low_age < MEASUREMENT_EXPIRY)
            {
                datastore_get_float(datastore, RESOURCE_ID_TEMP_VALUE, SENSOR_HIGH, &t_high);
                datastore_get_float(datastore, RESOURCE_ID_TEMP_VALUE, SENSOR_LOW, &t_low);
                ESP_LOGD(TAG, "CP control loop: T HIGH %.2f, T LOW %.2f", t_high, t_low);

                // transitions
                if (state == AVR_PUMP_STATE_OFF)
                {
                    float delta = 0.0f;
                    datastore_get_float(datastore, RESOURCE_ID_CONTROL_CP_ON_DELTA, 0, &delta);
                    ESP_LOGD(TAG, "CP control loop: delta on %f", delta);
                    if (t_high - t_low >= delta)
                    {
                        ESP_LOGI(TAG, "CP control loop: circulation pump ON");
                        datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Circulation pump on");
                        state = AVR_PUMP_STATE_ON;
                        transition = true;
                    }
                }
                else
                {
                    float delta = 0.0f;
                    datastore_get_float(datastore, RESOURCE_ID_CONTROL_CP_OFF_DELTA, 0, &delta);
                    ESP_LOGD(TAG, "CP control loop: delta off %f", delta);
                    if (t_high - t_low <= delta)
                    {
                        ESP_LOGI(TAG, "CP control loop: circulation pump OFF");
                        datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Circulation pump off");
                        state = AVR_PUMP_STATE_OFF;
                        transition = true;
                    }
                }

                // output
                if (transition)
                {
                    avr_support_set_cp_pump(state);
                }
            }
            else
            {
                ESP_LOGW(TAG, "CP control loop: T LOW measurement timeout");
            }
        }
        else
        {
            ESP_LOGW(TAG, "CP control loop: T HIGH measurement timeout");
        }

        vTaskDelayUntil(&last_wake_time, POLL_PERIOD / portTICK_PERIOD_MS);
    }
}

static void _get_local_time(time_t * now_time, struct tm * timeinfo)
{
    assert(now_time != NULL);
    assert(timeinfo != NULL);
    time(now_time);
    localtime_r(now_time, timeinfo);
}

static void control_pp_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());
    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    const datastore_t * datastore = task_inputs->datastore;

    TickType_t last_wake_time = xTaskGetTickCount();

    avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);

    // wait for stable sensor readings
    datastore_age_t age = DATASTORE_INVALID_AGE;
    bool stable = false;
    while (!stable)
    {
        last_wake_time = xTaskGetTickCount();
        ESP_LOGD(TAG, "PP control loop: wait for stable sensors");

        datastore_get_age(datastore, RESOURCE_ID_FLOW_RATE, 0, &age);

        if (age < MEASUREMENT_EXPIRY)
        {
            stable = true;
            ESP_LOGI(TAG, "PP control loop: sensors stable");
        }

        vTaskDelayUntil(&last_wake_time, POLL_PERIOD / portTICK_PERIOD_MS);
    }

    typedef enum
    {
        STATE_PP_OFF = 0,
        STATE_PP_ON,
        STATE_PP_PAUSE,
        STATE_PP_EMERGENCY,
    } state_t;

    state_t state = STATE_PP_OFF;
    uint32_t cycle_start_time = 0;
    uint32_t n = 0;
    struct tm last_timeinfo = { 0 };

    while (1)
    {
        last_wake_time = xTaskGetTickCount();
        uint32_t now = seconds_since_boot();

        ESP_LOGD(TAG, "PP control loop: state %d, n %d", state, n);

        // run cycle daily at configured time
        bool daily_trigger = false;
        bool system_time_set = false;
        datastore_get_bool(datastore, RESOURCE_ID_SYSTEM_TIME_SET, 0, &system_time_set);
        if (system_time_set)
        {
            int32_t daily_hour = -1;
            int32_t daily_minute = -1;
            datastore_get_int32(datastore, RESOURCE_ID_CONTROL_PP_DAILY_HOUR, 0, &daily_hour);
            datastore_get_int32(datastore, RESOURCE_ID_CONTROL_PP_DAILY_MINUTE, 0, &daily_minute);

            if (daily_hour >= 0 && daily_minute >= 0)
            {
                time_t now;
                struct tm timeinfo;
                _get_local_time(&now, &timeinfo);
                ESP_LOGD(TAG, "PP control loop: timeinfo.tm_hour %d, last_timeinfo.tm_hour %d", timeinfo.tm_hour, last_timeinfo.tm_hour);
                ESP_LOGD(TAG, "PP control loop: timeinfo.tm_min %d, last_timeinfo.tm_min %d", timeinfo.tm_min, last_timeinfo.tm_min);
                if ((timeinfo.tm_min != last_timeinfo.tm_min)
                    && (timeinfo.tm_hour == daily_hour && timeinfo.tm_min == daily_minute))
                {
                    daily_trigger = true;
                    char strftime_buf[64];
                    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
                    ESP_LOGI(TAG, "PP control loop: triggered Purge Pump at %s", strftime_buf);
                }
                else
                {
                    int now_minutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
                    int set_minutes = daily_hour * 60 + daily_minute;
                    int rem_minutes = now_minutes <= set_minutes ? set_minutes - now_minutes : 24 * 60 - now_minutes + set_minutes;
                    int hours_remaining = rem_minutes / 60;  // floor
                    int minutes_remaining = rem_minutes - (hours_remaining * 60);
                    ESP_LOGD(TAG, "PP control loop: %dh%02dm until daily purge (now %d, set %d, rem %d)", hours_remaining, minutes_remaining, now_minutes, set_minutes, rem_minutes);
                }
                last_timeinfo = timeinfo;
            }
            else
            {
                ESP_LOGD(TAG, "PP control loop: daily PP timer disabled");
            }
        }
        else
        {
            ESP_LOGD(TAG, "PP control loop: waiting for system time to be set");
        }

        // If the array temperature (hard-coded as T2) exceeds the safe threshold,
        // immediately run the purge pump cycle until the temperature returns to a safe level
        datastore_age_t t_high_age = DATASTORE_INVALID_AGE;
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, SENSOR_HIGH, &t_high_age);
        if (t_high_age < MEASUREMENT_EXPIRY)
        {
            float t_high = 0.0f;
            datastore_get_float(datastore, RESOURCE_ID_TEMP_VALUE, SENSOR_HIGH, &t_high);

            if (state == STATE_PP_EMERGENCY)
            {
                if (t_high < PP_EMERGENCY_THRESHOLD_OFF)
                {
                    ESP_LOGI(TAG, "Safe temperature restored");
                    state = STATE_PP_OFF;
                    ESP_LOGI(TAG, "PP control loop: purge pump OFF");
                    datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Safe temperature restored");
                    avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
                }
                else
                {
                    // keep the PP running
                    // TODO: in the case of emergency, do we cycle the PP?
                    // TODO: sound alarm?
                    // TODO: what to do if the temperature isn't dropping?
                }
            }
            else
            {
                if (t_high >= PP_EMERGENCY_THRESHOLD_ON)
                {
                    ESP_LOGE(TAG, "EMERGENCY - SAFE THRESHOLD EXCEEDED!");
                    datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Emergency purge");
                    state = STATE_PP_EMERGENCY;
                    ESP_LOGI(TAG, "PP control loop: purge pump ON");
                    avr_support_set_pp_pump(AVR_PUMP_STATE_ON);
                }
            }
        }

        // Outputs are changed on state transitions
        switch (state)
        {
            case STATE_PP_OFF:
            {
                datastore_get_age(datastore, RESOURCE_ID_FLOW_RATE, 0, &age);
                if (age < MEASUREMENT_EXPIRY)
                {
                    float flow_rate = 0.0f;
                    datastore_get_float(datastore, RESOURCE_ID_FLOW_RATE, 0, &flow_rate);
                    avr_pump_state_t cp_state = AVR_PUMP_STATE_OFF;

                    datastore_get_uint32(datastore, RESOURCE_ID_PUMPS_CP_STATE, 0, &cp_state);
                    datastore_age_t cp_state_age = 0;
                    datastore_get_age(datastore, RESOURCE_ID_PUMPS_CP_STATE, 0, &cp_state_age);
                    ESP_LOGD(TAG, "PP control loop: cp_state_age %" PRIu64, cp_state_age);

                    float flow_threshold = 0.0f;
                    datastore_get_float(datastore, RESOURCE_ID_CONTROL_FLOW_THRESHOLD, 0, &flow_threshold);

                    ESP_LOGD(TAG, "PP control loop: flow rate %f, cp state %d, threshold %f", flow_rate, cp_state, flow_threshold);

                    if (daily_trigger ||
                        ((cp_state == AVR_PUMP_STATE_ON) && (flow_rate <= flow_threshold) && (cp_state_age > PP_HOLD_OFF)))
                    {
                        datastore_get_uint32(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_COUNT, 0, &n);
                        ESP_LOGI(TAG, "PP control loop: purge pump ON (%d): %s", n, daily_trigger ? "time of day" : "low flow");
                        datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, daily_trigger ? "Purge pump on (time of day)" : "Purge pump on (low flow)");
                        state = STATE_PP_ON;
                        avr_support_set_pp_pump(AVR_PUMP_STATE_ON);
                        cycle_start_time = now;
                        --n;
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "PP control loop: flow measurement expired");
                }
                break;
            }

            case STATE_PP_ON:
            {
                uint32_t duration = 0;
                datastore_get_uint32(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_ON_DURATION, 0, &duration);
                uint32_t cycle_end_time = cycle_start_time + duration;

                if (now >= cycle_end_time)
                {
                    ESP_LOGI(TAG, "PP control loop: purge pump PAUSE (%d)", n);
                    state = STATE_PP_PAUSE;
                    cycle_start_time = now;
                    avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
                }
                break;
            }

            case STATE_PP_PAUSE:
            {
                uint32_t duration = 0;
                datastore_get_uint32(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_PAUSE_DURATION, 0, &duration);
                TickType_t cycle_end_time = cycle_start_time + duration;

                if (n > 0)
                {
                    if (now >= cycle_end_time)
                    {
                        ESP_LOGI(TAG, "PP control loop: purge pump ON (%d)", n);
                        state = STATE_PP_ON;
                        avr_support_set_pp_pump(AVR_PUMP_STATE_ON);
                        cycle_start_time = now;
                        --n;
                    }
                }
                else
                {
                    if (now >= cycle_end_time)
                    {
                        ESP_LOGI(TAG, "PP control loop: purge pump OFF");
                        datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Purge pump off");
                        state = STATE_PP_OFF;
                        avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
                    }
                }
                break;
            }

            case STATE_PP_EMERGENCY:
                // no-op
                break;

            default:
                ESP_LOGE(TAG, "PP control loop: invalid case %d", state);
        }

        // if PP in manual mode, drop out of cycle
        datastore_age_t pp_mode_switch_age = 0;
        datastore_get_age(datastore, RESOURCE_ID_SWITCHES_PP_MODE_VALUE, 0, &pp_mode_switch_age);
        if (pp_mode_switch_age < MEASUREMENT_EXPIRY)
        {
            avr_switch_mode_t pp_mode = AVR_SWITCH_MODE_AUTO;
            datastore_get_uint32(datastore, RESOURCE_ID_SWITCHES_PP_MODE_VALUE, 0, &pp_mode);
            if (pp_mode != AVR_SWITCH_MODE_AUTO)
            {
                ESP_LOGI(TAG, "PP control loop: purge pump OFF (manual)");
                datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Purge pump off (manual)");
                state = STATE_PP_OFF;
                avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
            }
        }

        vTaskDelayUntil(&last_wake_time, POLL_PERIOD / portTICK_PERIOD_MS);
    }
}

void control_init(UBaseType_t priority, const datastore_t * datastore)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->datastore = datastore;
        xTaskCreate(&control_cp_task, "control_cp_task", 4096, task_inputs, priority, NULL);
        xTaskCreate(&control_pp_task, "control_pp_task", 4096, task_inputs, priority, NULL);
    }
}



