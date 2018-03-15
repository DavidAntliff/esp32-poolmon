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

#include "esp_log.h"
#include "apps/sntp/sntp.h"

#include "constants.h"
#include "wifi_support.h"
#include "resources.h"
#include "sntp_rtc.h"

#define TAG "sntp_rtc"

#define CHECK_PERIOD (5000) // milliseconds

typedef struct
{
    const datastore_t * datastore;
} task_inputs_t;

static void _initialise_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

static void _obtain_time(void)
{
    _initialise_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while(timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}

static void sntp_rtc_task(void * pvParameter)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    const datastore_t * datastore = task_inputs->datastore;

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        last_wake_time = xTaskGetTickCount();

        // check wifi is connected
        wifi_status_t wifi_status = WIFI_STATUS_DISCONNECTED;
        datastore_get_uint32(datastore, RESOURCE_ID_WIFI_STATUS, 0, &wifi_status);

        if (wifi_status == WIFI_STATUS_GOT_ADDRESS)
        {
            time_t now;
            struct tm timeinfo;
            time(&now);
            localtime_r(&now, &timeinfo);

            // Is time set? If not, tm_year will be (1970 - 1900).
            if (timeinfo.tm_year < (2016 - 1900)) {
                ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
                _obtain_time();
                // update 'now' variable with current time
                time(&now);
            }
            char strftime_buf[64];

            // Set timezone to New Zealand Daylight Savings Time and print local time
            // https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
            setenv("TZ", LOCAL_TIMEZONE_CODE, 1);
            tzset();
            localtime_r(&now, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "The current date/time in New Zealand is: %s", strftime_buf);

            // Set timezone to UTC and print time
            setenv("TZ", UTC_TIMEZONE_CODE, 1);
            tzset();
            localtime_r(&now, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "The current date/time in UTC is: %s", strftime_buf);
        }
        else
        {
            ESP_LOGD(TAG, "waiting for WiFi address");
        }

        vTaskDelayUntil(&last_wake_time, CHECK_PERIOD / portTICK_PERIOD_MS);
    }
}

void sntp_rtc_init(UBaseType_t priority, const datastore_t * datastore)
{
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->datastore = datastore;
        xTaskCreate(&sntp_rtc_task, "sntp_rtc_task", 4096, task_inputs, priority, NULL);
    }
}
