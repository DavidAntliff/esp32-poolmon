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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "sensor_temp.h"
#include "constants.h"
#include "datastore.h"
#include "led.h"
#include "publish.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_10_BIT)
#define SAMPLE_PERIOD        (4000)  // sensor sampling period in milliseconds

#define TAG "sensor_temp"

extern datastore_t * datastore;

struct _temp_sensors_t
{
    OneWireBus * owb;
    OneWireBus_ROMCode * rom_codes;
    DS18B20_Info ** ds18b20_infos;
    int num_ds18b20s;
};

typedef struct
{
    temp_sensors_t * sensors;
    QueueHandle_t publish_queue;
} task_inputs_t;

static void read_temperatures(DS18B20_Info ** device_infos, float * readings, int num_devices)
{
    ds18b20_convert_all(device_infos[0]->bus);

    // in this application all devices use the same resolution,
    // so use the first device to determine the delay
    ds18b20_wait_for_conversion(device_infos[0]);

    // read the results immediately after conversion otherwise it may fail
    // (using printf before reading may take too long)
    for (int i = 0; i < num_devices; ++i)
    {
        readings[i] = ds18b20_read_temp(device_infos[i]);
    }
}

/**
 * @brief Find all (or the first N) ROM codes for devices connected to the One Wire Bus.
 * @param[in] owb Pointer to initialised bus instance.
 * @param[in] devices A pre-existing array of ROM code structures, sufficient to hold max_devices entries.
 * @param[in] max_devices Maximum number of ROM codes to find.
 * @return Number of ROM codes found, or zero if no devices found.
 */
static int find_owb_rom_codes(const OneWireBus * owb, OneWireBus_ROMCode * rom_codes, int max_codes)
{
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);

    ESP_LOGI(TAG, "Find devices:");
    while (found && num_devices <= max_codes)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGI(TAG, "  %d : %s", num_devices, rom_code_s);
        rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    return num_devices;
}

static void associate_ds18b20_devices(const OneWireBus * owb,
                                      const OneWireBus_ROMCode * rom_codes,
                                      DS18B20_Info ** device_infos,
                                      int num_devices)
{
    for (int i = 0; i < num_devices; ++i)
    {
        DS18B20_Info * ds18b20_info = ds18b20_malloc();
        device_infos[i] = ds18b20_info;

        if (num_devices == 1)
        {
            ESP_LOGD(TAG, "Single device optimisations enabled");
            ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
        }
        else
        {
            ds18b20_init(ds18b20_info, owb, rom_codes[i]); // associate with bus and device
        }
        ds18b20_use_crc(ds18b20_info, true);           // enable CRC check for temperature readings
        ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
    }
}

static temp_sensors_t * detect_sensors(uint8_t gpio)
{
    // set up the One Wire Bus
    OneWireBus * owb = NULL;
    temp_sensors_t * sensors = NULL;
    owb_rmt_driver_info * rmt_driver_info = malloc(sizeof(*rmt_driver_info));

    // TODO: Need to keep hold of rmt_driver_info so we can free it later

    if (rmt_driver_info)
    {
        owb = owb_rmt_initialize(rmt_driver_info, gpio, OWB_RMT_CHANNEL_TX, OWB_RMT_CHANNEL_RX);
        owb_use_crc(owb, true);       // enable CRC check for ROM code and measurement readings

        // locate attached devices
        OneWireBus_ROMCode * device_rom_codes = calloc(MAX_DEVICES, sizeof(*device_rom_codes));

        int num_devices = find_owb_rom_codes(owb, device_rom_codes, MAX_DEVICES);

        // free up unused space
        device_rom_codes = realloc(device_rom_codes, num_devices * sizeof(*device_rom_codes));

        // associate devices on bus with DS18B20 device driver
        DS18B20_Info ** device_infos = calloc(num_devices, sizeof(*device_infos));
        associate_ds18b20_devices(owb, device_rom_codes, device_infos, num_devices);

        sensors = malloc(sizeof(*sensors));
        if (sensors)
        {
            memset(sensors, 0, sizeof(*sensors));
            sensors->owb = owb;
            sensors->rom_codes = device_rom_codes;
            sensors->ds18b20_infos = device_infos;
            sensors->num_ds18b20s = num_devices;
        }
        else
        {
            ESP_LOGE(TAG, "malloc failed");
        }
    }
    else
    {
        ESP_LOGE(TAG, "malloc failed");
    }
    return sensors;
}

static void sensor_temp_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGW(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    int sample_count = 0;

    // TODO
    datastore_set_string(datastore, DATASTORE_ID_TEMP_LABEL, 0, "LABEL1");
    datastore_set_string(datastore, DATASTORE_ID_TEMP_LABEL, 1, "LABEL2");
    datastore_set_string(datastore, DATASTORE_ID_TEMP_LABEL, 2, "LABEL3");
    datastore_set_string(datastore, DATASTORE_ID_TEMP_LABEL, 3, "LABEL4");
    datastore_set_string(datastore, DATASTORE_ID_TEMP_LABEL, 4, "LABEL5");

    int errors[MAX_DEVICES] = {0};
    if (task_inputs->sensors->num_ds18b20s > 0)
    {
        // wait a second before starting commencing sampling
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        TickType_t last_wake_time = xTaskGetTickCount();

        while (1)
        {
            last_wake_time = xTaskGetTickCount();

            led_on();

            float readings[MAX_DEVICES] = { 0 };

            read_temperatures(task_inputs->sensors->ds18b20_infos, readings, task_inputs->sensors->num_ds18b20s);
            ++sample_count;

            led_off();

            // print results in a separate loop, after all have been read
            ESP_LOGI(TAG, "Temperature readings (degrees C): sample %d", sample_count);
            for (int i = 0; i < task_inputs->sensors->num_ds18b20s; ++i)
            {
                // filter out invalid readings
                if (readings[i] != DS18B20_INVALID_READING)
                {
                    publish_value(PUBLISH_VALUE_TEMP_1 + i, readings[i], task_inputs->publish_queue);
                    datastore_set_float(datastore, DATASTORE_ID_TEMP_VALUE, i, readings[i]);
                }
                else
                {
                    ++errors[i];
                }
                ESP_LOGI(TAG, "  %d: %.1f    %d errors", i, readings[i], errors[i]);
            }

            //vTaskDelayUntil(&last_wake_time, 1000 / portTICK_PERIOD_MS); -- not yet supported by ESP-IDF
            vTaskDelay(SAMPLE_PERIOD / portTICK_PERIOD_MS - (xTaskGetTickCount() - last_wake_time));
        }
    }
    else
    {
        ESP_LOGE(TAG, "No devices found");
    }

    free(task_inputs);
    vTaskDelete(NULL);
}

temp_sensors_t * sensor_temp_init(uint8_t gpio, UBaseType_t priority, QueueHandle_t publish_queue)
{
    // Assume that new devices are not connected during operation.
    temp_sensors_t * sensors = detect_sensors(gpio);

    // Blink the LED to indicate the number of temperature devices detected:
    led_flash(100, 200, sensors->num_ds18b20s);

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->sensors = sensors;
        task_inputs->publish_queue = publish_queue;
        xTaskCreate(&sensor_temp_task, "sensor_temp_task", 4096, task_inputs, priority, NULL);
    }
    return sensors;
}

void sensor_temp_close(temp_sensors_t * sensors)
{
    if (sensors != NULL)
    {
        // clean up dynamically allocated data
        free(sensors->rom_codes);
        for (int i = 0; i < sensors->num_ds18b20s; ++i)
        {
            ds18b20_free(&(sensors->ds18b20_infos[i]));
        }
        free(sensors->ds18b20_infos);
        owb_uninitialize(sensors->owb);
        // TODO: free the owb_rmt_driver_info struct
        free(sensors);
    }
}


