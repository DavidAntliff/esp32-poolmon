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

    datastore_age_t t1_age = DATASTORE_INVALID_AGE;
    datastore_age_t t2_age = DATASTORE_INVALID_AGE;

    // wait for stable sensor readings
    bool stable = false;
    while(!stable)
    {
        last_wake_time = xTaskGetTickCount();
        ESP_LOGD(TAG, "CP control loop: wait for stable sensors");

        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, 0, &t1_age);
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, 1, &t2_age);

        if ((t1_age < MEASUREMENT_EXPIRY) && (t2_age < MEASUREMENT_EXPIRY))
        {
            stable = true;
            ESP_LOGI(TAG, "CP control loop: sensors stable");
        }

        vTaskDelayUntil(&last_wake_time, POLL_PERIOD / portTICK_PERIOD_MS);
    }

    while (1)
    {
        last_wake_time = xTaskGetTickCount();
        ESP_LOGD(TAG, "CP control loop: state %d", state);

        float t1 = 0.0f;
        float t2 = 0.0f;
        bool transition = false;

        t1_age = DATASTORE_INVALID_AGE;
        t2_age = DATASTORE_INVALID_AGE;
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, 0, &t1_age);
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, 1, &t2_age);

        if (t1_age < MEASUREMENT_EXPIRY)
        {
            if (t2_age < MEASUREMENT_EXPIRY)
            {
                datastore_get_float(datastore, RESOURCE_ID_TEMP_VALUE, 0, &t1);
                datastore_get_float(datastore, RESOURCE_ID_TEMP_VALUE, 1, &t2);
                ESP_LOGD(TAG, "T1 %f, T2 %f", t1, t2);

                // transitions
                if (state == AVR_PUMP_STATE_OFF)
                {
                    float delta = 0.0f;
                    datastore_get_float(datastore, RESOURCE_ID_CONTROL_CP_ON_DELTA, 0, &delta);
        //            ESP_LOGD(TAG, "delta on %f", delta);
                    if (t1 - t2 >= delta)
                    {
                        state = AVR_PUMP_STATE_ON;
                        transition = true;
                    }
                }
                else
                {
                    float delta = 0.0f;
                    datastore_get_float(datastore, RESOURCE_ID_CONTROL_CP_OFF_DELTA, 0, &delta);
        //            ESP_LOGD(TAG, "delta off %f", delta);
                    if (t1 - t2 <= delta)
                    {
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
                ESP_LOGW(TAG, "T2 measurement timeout");
            }
        }
        else
        {
            ESP_LOGW(TAG, "T1 measurement timeout");
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
    while(!stable)
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
                ESP_LOGD(TAG, "timeinfo.tm_hour %d, last_timeinfo.tm_hour %d", timeinfo.tm_hour, last_timeinfo.tm_hour);
                ESP_LOGD(TAG, "timeinfo.tm_min %d, last_timeinfo.tm_min %d", timeinfo.tm_min, last_timeinfo.tm_min);
                if ((timeinfo.tm_min != last_timeinfo.tm_min)
                    && (timeinfo.tm_hour == daily_hour && timeinfo.tm_min == daily_minute))
                {
                    daily_trigger = true;
                    char strftime_buf[64];
                    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
                    ESP_LOGI(TAG, "Triggered Purge Pump at %s", strftime_buf);
                }
                else
                {
                    int now_minutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
                    int set_minutes = daily_hour * 60 + daily_minute;
                    int rem_minutes = now_minutes <= set_minutes ? set_minutes - now_minutes : 24 * 60 - now_minutes + set_minutes;
                    int hours_remaining = rem_minutes / 60;  // floor
                    int minutes_remaining = rem_minutes - (hours_remaining * 60);
                    ESP_LOGD(TAG, "%dh%02dm until daily purge (now %d, set %d, rem %d)", hours_remaining, minutes_remaining, now_minutes, set_minutes, rem_minutes);
                }
                last_timeinfo = timeinfo;
            }
            else
            {
                ESP_LOGD(TAG, "Daily PP timer disabled");
            }
        }
        else
        {
            ESP_LOGD(TAG, "Waiting for system time to be set");
        }

        // TODO: handle tick count overflow

        // TODO: confirm pump state matches

        //ESP_LOGD(TAG, "cp_state %d, FR %f", cp_state, flow_rate);

        // outputs are on transitions

        switch(state)
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
                float flow_threshold = 0.0f;
                datastore_get_float(datastore, RESOURCE_ID_CONTROL_FLOW_THRESHOLD, 0, &flow_threshold);

                ESP_LOGD(TAG, "flow rate %f, cp state %d, threshold %f", flow_rate, cp_state, flow_threshold);

                if (daily_trigger || ((cp_state == AVR_PUMP_STATE_ON) && (flow_rate <= flow_threshold)))
                {
                    state = STATE_PP_ON;
                    avr_support_set_pp_pump(AVR_PUMP_STATE_ON);
                    cycle_start_time = now;
                    datastore_get_uint32(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_COUNT, 0, &n);
                    --n;
                }
            }
            else
            {
                ESP_LOGW(TAG, "Flow measurement expired");
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
                    state = STATE_PP_OFF;
                    avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
                }
            }
            break;
        }

        default:
            ESP_LOGE(TAG, "invalid case %d", state);
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



