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
#include "sensor_temp.h"

#define POLL_PERIOD                  (1000)            // control loop period in milliseconds
#define FLOW_RATE_MEASUREMENT_EXPIRY (15 * 1000000)    // microseconds
#define PP_HOLD_OFF                  (30 * 1000000)    // to check for flow when CP is on, wait at least this many seconds before deciding to start PP if flow rate is below threshold

#define TAG "control"

typedef struct
{
    const datastore_t * datastore;
} task_inputs_t;

static TaskHandle_t _cp_task_handle = NULL;
static TaskHandle_t _pp_task_handle = NULL;


void _avr_reset_handler(const datastore_t * datastore, datastore_resource_id_t id, datastore_instance_id_t instance, void * ctxt)
{
    bool * flag = (bool *)ctxt;
    assert(flag != NULL);
    *flag = true;
}

static void control_cp_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());
    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    const datastore_t * datastore = task_inputs->datastore;

    TickType_t last_wake_time = xTaskGetTickCount();

    control_cp_state_t state = CONTROL_CP_STATE_OFF;
    avr_support_set_cp_pump(AVR_PUMP_STATE_OFF);

    datastore_age_t t_high_age = DATASTORE_INVALID_AGE;
    datastore_age_t t_low_age = DATASTORE_INVALID_AGE;

    // scale measurement expiry threshold by current temp poll period
    datastore_age_t temp_expiry = sensor_temp_expiry(datastore);

    // wait for stable sensor readings
    bool stable = false;
    while (!stable)
    {
        last_wake_time = xTaskGetTickCount();
        ESP_LOGD(TAG, "CP control loop: wait for stable sensors");

        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, CONTROL_CP_SENSOR_HIGH_INSTANCE, &t_high_age);
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, CONTROL_CP_SENSOR_LOW_INSTANCE, &t_low_age);

        if ((t_high_age < temp_expiry) && (t_low_age < temp_expiry))
        {
            stable = true;
            ESP_LOGI(TAG, "CP control loop: sensors stable");
        }

        vTaskDelayUntil(&last_wake_time, POLL_PERIOD / portTICK_PERIOD_MS);
    }

    // In the case of an AVR reset, refresh the pump states
    bool refresh_cp = false;
    datastore_add_set_callback(datastore, RESOURCE_ID_AVR_COUNT_RESET, 0, _avr_reset_handler, &refresh_cp);

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
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, CONTROL_CP_SENSOR_HIGH_INSTANCE, &t_high_age);
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, CONTROL_CP_SENSOR_LOW_INSTANCE, &t_low_age);

        // update measurement expiry in case temp poll period has changed
        temp_expiry = sensor_temp_expiry(datastore);

        if (t_high_age < temp_expiry)
        {
            if (t_low_age < temp_expiry)
            {
                datastore_get_float(datastore, RESOURCE_ID_TEMP_VALUE, CONTROL_CP_SENSOR_HIGH_INSTANCE, &t_high);
                datastore_get_float(datastore, RESOURCE_ID_TEMP_VALUE, CONTROL_CP_SENSOR_LOW_INSTANCE, &t_low);
                ESP_LOGD(TAG, "CP control loop: T HIGH %.2f, T LOW %.2f", t_high, t_low);

                // transitions
                if (state == CONTROL_CP_STATE_OFF)
                {
                    float delta = 0.0f;
                    datastore_get_float(datastore, RESOURCE_ID_CONTROL_CP_ON_DELTA, 0, &delta);
                    ESP_LOGD(TAG, "CP control loop: delta on %f", delta);
                    if (t_high - t_low >= delta)
                    {
                        ESP_LOGI(TAG, "CP control loop: circulation pump ON");
                        datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Circulation pump on");
                        state = CONTROL_CP_STATE_ON;
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
                        state = CONTROL_CP_STATE_OFF;
                        transition = true;
                    }
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

        if (refresh_cp)
        {
            ESP_LOGW(TAG, "Refresh CP state (AVR reset)");
            transition = true;
            refresh_cp = false;
        }

        // output
        if (transition)
        {
            avr_support_set_cp_pump(state == CONTROL_CP_STATE_ON ? AVR_PUMP_STATE_ON : AVR_PUMP_STATE_OFF);
            datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_STATE_CP, 0, state);
        }

        vTaskDelayUntil(&last_wake_time, POLL_PERIOD / portTICK_PERIOD_MS);
    }

    free(task_inputs);
    _cp_task_handle = NULL;
    vTaskDelete(NULL);
}

// TODO: refactor into a Time module
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
    datastore_age_t flow_rate_age = DATASTORE_INVALID_AGE;
    bool stable = false;
    while (!stable)
    {
        last_wake_time = xTaskGetTickCount();
        ESP_LOGD(TAG, "PP control loop: wait for stable sensors");

        datastore_get_age(datastore, RESOURCE_ID_FLOW_RATE, 0, &flow_rate_age);

        if (flow_rate_age < FLOW_RATE_MEASUREMENT_EXPIRY)
        {
            stable = true;
            ESP_LOGI(TAG, "PP control loop: sensors stable");
        }

        vTaskDelayUntil(&last_wake_time, POLL_PERIOD / portTICK_PERIOD_MS);
    }

    control_pp_state_t state = CONTROL_PP_STATE_OFF;
    datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_STATE_PP, 0, state);
    uint32_t cycle_start_time = 0;
    uint32_t n = 0;
    struct tm last_timeinfo = { 0 };

    // scale measurement expiry threshold by current temp poll period
    datastore_age_t temp_expiry = sensor_temp_expiry(datastore);

    while (1)
    {
        last_wake_time = xTaskGetTickCount();
        uint32_t now = seconds_since_boot();

        ESP_LOGD(TAG, "PP control loop: state %d, n %d", state, n);

        bool daily_enable = false;
        datastore_get_bool(datastore, RESOURCE_ID_CONTROL_PP_DAILY_ENABLE, 0, &daily_enable);
        bool daily_trigger = false;

        if (daily_enable)
        {
            // run cycle daily at configured time
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
        }

        // update measurement expiry in case temp poll period has changed
        temp_expiry = sensor_temp_expiry(datastore);

        // If the array temperature (hard-coded as T2) exceeds the safe threshold,
        // immediately run the purge pump cycle until the temperature returns to a safe level
        datastore_age_t t_high_age = DATASTORE_INVALID_AGE;
        datastore_get_age(datastore, RESOURCE_ID_TEMP_VALUE, CONTROL_CP_SENSOR_HIGH_INSTANCE, &t_high_age);
        if (t_high_age < temp_expiry)
        {
            float t_high = 0.0f;
            datastore_get_float(datastore, RESOURCE_ID_TEMP_VALUE, CONTROL_CP_SENSOR_HIGH_INSTANCE, &t_high);

            if (state == CONTROL_PP_STATE_EMERGENCY)
            {
                float threshold = 0.0f;
                datastore_get_float(datastore, RESOURCE_ID_CONTROL_SAFE_TEMP_LOW, 0, &threshold);
                if (t_high < threshold)
                {
                    ESP_LOGI(TAG, "PP control loop: safe temperature restored - purge pump OFF");
                    datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Safe temperature restored - purge pump off");
                    state = CONTROL_PP_STATE_OFF;
                    datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_STATE_PP, 0, state);
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
                float threshold = 0.0f;
                datastore_get_float(datastore, RESOURCE_ID_CONTROL_SAFE_TEMP_HIGH, 0, &threshold);
                if (t_high >= threshold)
                {
                    ESP_LOGE(TAG, "EMERGENCY - SAFE THRESHOLD EXCEEDED!");
                    ESP_LOGI(TAG, "PP control loop: purge pump ON");
                    datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Safe threshold exceeded - purge pump on");
                    state = CONTROL_PP_STATE_EMERGENCY;
                    datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_STATE_PP, 0, state);
                    avr_support_set_pp_pump(AVR_PUMP_STATE_ON);
                }
            }
        }

        // Outputs are changed on state transitions
        switch (state)
        {
            case CONTROL_PP_STATE_OFF:
            {
                avr_switch_mode_t pp_mode = AVR_SWITCH_MODE_AUTO;
                datastore_get_uint32(datastore, RESOURCE_ID_SWITCHES_PP_MODE_VALUE, 0, &pp_mode);
                if (pp_mode == AVR_SWITCH_MODE_AUTO)
                {
                    bool flow_rate_trigger = false;
                    datastore_get_age(datastore, RESOURCE_ID_FLOW_RATE, 0, &flow_rate_age);
                    if (flow_rate_age < FLOW_RATE_MEASUREMENT_EXPIRY)
                    {
                        float flow_rate = 0.0f;
                        datastore_get_float(datastore, RESOURCE_ID_FLOW_RATE, 0, &flow_rate);
                        avr_pump_state_t cp_pump_state = AVR_PUMP_STATE_OFF;

                        datastore_get_uint32(datastore, RESOURCE_ID_PUMPS_CP_STATE, 0, &cp_pump_state);
                        datastore_age_t cp_state_age = 0;
                        datastore_get_age(datastore, RESOURCE_ID_PUMPS_CP_STATE, 0, &cp_state_age);
                        ESP_LOGD(TAG, "PP control loop: cp_state_age %" PRIu64, cp_state_age);

                        float flow_threshold = 0.0f;
                        datastore_get_float(datastore, RESOURCE_ID_CONTROL_FLOW_THRESHOLD, 0, &flow_threshold);

                        ESP_LOGD(TAG, "PP control loop: flow rate %f, cp state %d, threshold %f", flow_rate, cp_pump_state, flow_threshold);

                        flow_rate_trigger = (cp_pump_state == AVR_PUMP_STATE_ON) && (flow_rate <= flow_threshold) && (cp_state_age > PP_HOLD_OFF);
                    }

                    if (daily_trigger || flow_rate_trigger)
                    {
                        datastore_get_uint32(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_COUNT, 0, &n);
                        ESP_LOGI(TAG, "PP control loop: purge pump ON (%d): %s", n, daily_trigger ? "time of day" : "low flow");
                        datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, daily_trigger ? "Purge pump on (time of day)" : "Purge pump on (low flow)");
                        state = CONTROL_PP_STATE_ON;
                        datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_STATE_PP, 0, state);
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

            case CONTROL_PP_STATE_ON:
            {
                uint32_t duration = 0;
                datastore_get_uint32(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_ON_DURATION, 0, &duration);
                uint32_t cycle_end_time = cycle_start_time + duration;

                if (now >= cycle_end_time)
                {
                    ESP_LOGI(TAG, "PP control loop: purge pump PAUSE (%d)", n);
                    // don't set log
                    state = CONTROL_PP_STATE_PAUSE;
                    datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_STATE_PP, 0, state);
                    cycle_start_time = now;
                    avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
                }
                break;
            }

            case CONTROL_PP_STATE_PAUSE:
            {
                uint32_t duration = 0;
                datastore_get_uint32(datastore, RESOURCE_ID_CONTROL_PP_CYCLE_PAUSE_DURATION, 0, &duration);
                TickType_t cycle_end_time = cycle_start_time + duration;

                if (n > 0)
                {
                    if (now >= cycle_end_time)
                    {
                        ESP_LOGI(TAG, "PP control loop: purge pump ON (%d)", n);
                        // don't set log
                        state = CONTROL_PP_STATE_ON;
                        datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_STATE_PP, 0, state);
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
                        state = CONTROL_PP_STATE_OFF;
                        datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_STATE_PP, 0, state);
                        avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
                    }
                }
                break;
            }

            case CONTROL_PP_STATE_EMERGENCY:
                // no-op
                break;

            default:
                ESP_LOGE(TAG, "PP control loop: invalid case %d", state);
        }

        // if PP in manual mode, drop out of cycle
        if (state == CONTROL_PP_STATE_ON || state == CONTROL_PP_STATE_PAUSE)
        {
            avr_switch_mode_t pp_mode = AVR_SWITCH_MODE_AUTO;
            datastore_get_uint32(datastore, RESOURCE_ID_SWITCHES_PP_MODE_VALUE, 0, &pp_mode);
            if (pp_mode != AVR_SWITCH_MODE_AUTO)
            {
                ESP_LOGI(TAG, "PP control loop: purge pump OFF (manual)");
                datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "Purge pump off (manual)");
                state = CONTROL_PP_STATE_OFF;
                datastore_set_uint32(datastore, RESOURCE_ID_CONTROL_STATE_PP, 0, state);
                avr_support_set_pp_pump(AVR_PUMP_STATE_OFF);
            }
        }

        vTaskDelayUntil(&last_wake_time, POLL_PERIOD / portTICK_PERIOD_MS);
    }

    free(task_inputs);
    _pp_task_handle = NULL;
    vTaskDelete(NULL);
}

void control_init(UBaseType_t priority, const datastore_t * datastore)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // task will take ownership of this struct
    task_inputs_t * cp_task_inputs = malloc(sizeof(*cp_task_inputs));
    if (cp_task_inputs)
    {
        memset(cp_task_inputs, 0, sizeof(*cp_task_inputs));
        cp_task_inputs->datastore = datastore;
        xTaskCreate(&control_cp_task, "control_cp_task", 4096, cp_task_inputs, priority, &_cp_task_handle);
    }

    // task will take ownership of this struct
    task_inputs_t * pp_task_inputs = malloc(sizeof(*pp_task_inputs));
    if (pp_task_inputs)
    {
        memset(pp_task_inputs, 0, sizeof(*pp_task_inputs));
        pp_task_inputs->datastore = datastore;
        xTaskCreate(&control_pp_task, "control_pp_task", 4096, pp_task_inputs, priority, &_pp_task_handle);
    }
}

void control_delete(void)
{
    if (_cp_task_handle)
        vTaskDelete(_cp_task_handle);
    if (_pp_task_handle)
        vTaskDelete(_pp_task_handle);
}


