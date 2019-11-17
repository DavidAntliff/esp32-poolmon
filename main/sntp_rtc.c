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
#include "lwip/apps/sntp.h"

#include "constants.h"
#include "wifi_support.h"
#include "resources.h"
#include "sntp_rtc.h"

#define TAG "sntp_rtc"

#define OUTER_CHECK_PERIOD (10 * 1000)  // wait time between checking system time is set in milliseconds
#define INNER_CHECK_PERIOD (5 * 1000)   // wait time between inner checks in milliseconds
#define NTP_SERVER "pool.ntp.org"

typedef struct
{
    const datastore_t * datastore;
} task_inputs_t;

static TaskHandle_t _task_handle = NULL;

static void _initialise_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_stop();
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, NTP_SERVER);
    sntp_init();
}

static bool _obtain_time(void)
{
    _initialise_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && retry++ < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(INNER_CHECK_PERIOD / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry >= retry_count)
    {
        ESP_LOGE(TAG, "Unable to set system time");
    }
    return !(retry >= retry_count);
}

static void sntp_rtc_task(void * pvParameter)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    const datastore_t * datastore = task_inputs->datastore;

    TickType_t last_wake_time = xTaskGetTickCount();

    // Set timezone to New Zealand Daylight Savings Time
    // https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
    setenv("TZ", LOCAL_TIMEZONE_CODE, 1);
    tzset();

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
                datastore_set_bool(datastore, RESOURCE_ID_SYSTEM_TIME_SET, 0, false);
                if (_obtain_time())
                {
                    ESP_LOGI(TAG, "System time set");
                    datastore_set_bool(datastore, RESOURCE_ID_SYSTEM_TIME_SET, 0, true);
                    time(&now);

                    char strftime_buf[64];

                    // show local time
                    localtime_r(&now, &timeinfo);
                    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
                    ESP_LOGI(TAG, "The current date/time in New Zealand is: %s", strftime_buf);

                    // show UTC
                    gmtime_r(&now, &timeinfo);
                    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
                    ESP_LOGI(TAG, "The current date/time in UTC is: %s", strftime_buf);
                }
            }
        }
        else
        {
            ESP_LOGD(TAG, "waiting for WiFi address");
        }

        vTaskDelayUntil(&last_wake_time, OUTER_CHECK_PERIOD / portTICK_PERIOD_MS);
    }

    free(task_inputs);
    _task_handle = NULL;
    vTaskDelete(NULL);
}

void sntp_rtc_init(UBaseType_t priority, const datastore_t * datastore)
{
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->datastore = datastore;
        xTaskCreate(&sntp_rtc_task, "sntp_rtc_task", 4096, task_inputs, priority, &_task_handle);
    }
}

void sntp_rtc_delete(void)
{
    if (_task_handle)
        vTaskDelete(_task_handle);
}
