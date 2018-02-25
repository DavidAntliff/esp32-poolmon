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

#include "sensor_light.h"
#include "constants.h"
#include "resources.h"
#include "utils.h"
#include "smbus.h"
#include "tsl2561.h"
#include "publish.h"
#include "datastore/datastore.h"

#define TAG "sensor_light"

#define SAMPLE_PERIOD (5000)  // sensor sampling period in milliseconds

typedef struct
{
    i2c_master_info_t * i2c_master_info;
    const datastore_t * datastore;
} task_inputs_t;

static void sensor_light_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    i2c_master_info_t * i2c_master_info = task_inputs->i2c_master_info;
    i2c_port_t i2c_port = i2c_master_info->port;
    const datastore_t * datastore = task_inputs->datastore;

    uint8_t i2c_address = 0;
    datastore_get_uint8(datastore, RESOURCE_ID_LIGHT_I2C_ADDRESS, 0, &i2c_address);

    // before accessing I2C, use a lock to gain exclusive use of the bus
    i2c_master_lock(i2c_master_info, portMAX_DELAY);

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_port, i2c_address);
    smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);

    // Set up the TSL2561 device
    tsl2561_info_t * tsl2561_info = tsl2561_malloc();
    if (tsl2561_init(tsl2561_info, smbus_info) == ESP_OK)
    {
        // Set sensor integration time and gain
        tsl2561_set_integration_time_and_gain(tsl2561_info, TSL2561_INTEGRATION_TIME_402MS, TSL2561_GAIN_1X);
        //tsl2561_set_integration_time_and_gain(tsl2561_info, TSL2561_INTEGRATION_TIME_402MS, TSL2561_GAIN_16X);

        datastore_set_bool(datastore, RESOURCE_ID_LIGHT_DETECTED, 0, true);
        i2c_master_unlock(i2c_master_info);

        TickType_t last_wake_time = xTaskGetTickCount();

        while (1)
        {
            last_wake_time = xTaskGetTickCount();

            tsl2561_visible_t visible = 0;
            tsl2561_infrared_t infrared = 0;

            i2c_master_lock(i2c_master_info, portMAX_DELAY);
            esp_err_t err = tsl2561_read(tsl2561_info, &visible, &infrared);
            i2c_master_unlock(i2c_master_info);

            if (err == ESP_OK)
            {
                uint32_t lux = tsl2561_compute_lux(tsl2561_info, visible, infrared);

                ESP_LOGI(TAG, "Light Sensor Readings:")
                ESP_LOGI(TAG, "  Full spectrum: %d", visible + infrared);
                ESP_LOGI(TAG, "  Infrared:      %d", infrared);
                ESP_LOGI(TAG, "  Visible:       %d", visible);
                ESP_LOGI(TAG, "  Illuminance:   %d lux", lux);

                datastore_set_uint32(datastore, RESOURCE_ID_LIGHT_FULL, 0, visible + infrared);
                datastore_set_uint32(datastore, RESOURCE_ID_LIGHT_INFRARED, 0, infrared);
                datastore_set_uint32(datastore, RESOURCE_ID_LIGHT_VISIBLE, 0, visible);
                datastore_set_uint32(datastore, RESOURCE_ID_LIGHT_ILLUMINANCE, 0, lux);
                datastore_set_uint32(datastore, RESOURCE_ID_LIGHT_TIMESTAMP, 0, seconds_since_boot());
            }
            else
            {
                ESP_LOGW(TAG, "sensor error");
            }

            vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
        }
    }
    else
    {
        datastore_set_bool(datastore, RESOURCE_ID_LIGHT_DETECTED, 0, false);
        i2c_master_unlock(i2c_master_info);
    }

    smbus_free(&smbus_info);
    tsl2561_free(&tsl2561_info);
    free(task_inputs);
    vTaskDelete(NULL);
}

void sensor_light_init(i2c_master_info_t * i2c_master_info, UBaseType_t priority, const datastore_t * datastore)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->i2c_master_info = i2c_master_info;
        task_inputs->datastore = datastore;
        xTaskCreate(&sensor_light_task, "sensor_light_task", 4096, task_inputs, priority, NULL);
    }
}

