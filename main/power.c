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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "power.h"
#include "constants.h"
#include "resources.h"
#include "utils.h"
#include "datastore/datastore.h"

#define TAG "power"

// ignore any temperature measurements that have expired
#define MEASUREMENT_EXPIRY 15 // seconds

#define SHC_WATER (4184.0)               // J/kg/K
#define MASS_PER_VOLUME_WATER (1000.0)   // kg/m^3

static float _calculate_transfer_power_watts(float lpm, float temp_delta)
{
    // Power (W) = energy_per_time (J/s)
    //           = SHC_water (J/(kg.K)) * mass_per_volume (kg/m^3) * volume_change_per_time (m^3/s) * temperature_difference (K)

    float volume_change_per_time = lpm / 60.0 / 1000.0;
    float power = SHC_WATER * MASS_PER_VOLUME_WATER * volume_change_per_time * temp_delta;
    return power;
}

static void power_calculation_task(void * pvParameter)
{
    //assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    while (1)
    {
        // for now, use T1 and T2
        const uint8_t in = 0;
        const uint8_t out = 1;

        uint32_t timestamp_in = 0;
        uint32_t timestamp_out = 0;
        datastore_get_uint32(g_datastore, RESOURCE_ID_TEMP_TIMESTAMP, in, &timestamp_in);
        datastore_get_uint32(g_datastore, RESOURCE_ID_TEMP_TIMESTAMP, out, &timestamp_out);

        uint32_t now = seconds_since_boot();

        if (now - timestamp_in < MEASUREMENT_EXPIRY)
        {
            if (now - timestamp_out < MEASUREMENT_EXPIRY)
            {
                float temp_in = 0.0f;
                float temp_out = 0.0f;
                datastore_get_float(g_datastore, RESOURCE_ID_TEMP_VALUE, in, &temp_in);
                datastore_get_float(g_datastore, RESOURCE_ID_TEMP_VALUE, out, &temp_out);
                float delta = temp_out - temp_in;

                float lpm = 0.0f;
                datastore_get_float(g_datastore, RESOURCE_ID_FLOW_RATE, 0, &lpm);

                float power = _calculate_transfer_power_watts(lpm, delta);
                ESP_LOGI(TAG, "flow %f lpm, temp delta %f K, power %f W", lpm, delta, power);

                datastore_set_float(g_datastore, RESOURCE_ID_POWER_VALUE, 0, power);
                datastore_set_uint32(g_datastore, RESOURCE_ID_POWER_TIMESTAMP, 0, now);
            }
            else
            {
                ESP_LOGW(TAG, "T2 measurement has expired");
            }
        }
        else
        {
            ESP_LOGW(TAG, "T1 measurement has expired");
        }

        vTaskDelay(POWER_CALCULATION_RATE * 1000 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void power_init(UBaseType_t priority)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    xTaskCreate(&power_calculation_task, "power_calculation_task", 4096, NULL, priority, NULL);
}
