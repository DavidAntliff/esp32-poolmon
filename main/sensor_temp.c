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
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "led.h"
#include "sensor_temp.h"
#include "publish.h"
#include "owb.h"
#include "ds18b20.h"

#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_10_BIT)
#define SAMPLE_PERIOD        (4000)  // sensor sampling period in milliseconds

#define TAG "sensor_temp"

struct _TempSensors
{
    OneWireBus * owb;
    OneWireBus_ROMCode * rom_codes;
    DS18B20_Info ** ds18b20_infos;
    int num_ds18b20s;
};

typedef struct
{
    TempSensors * sensors;
    QueueHandle_t publish_queue;
} SensorTaskInputs;

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
    bool found = owb_search_first(owb, &search_state);

    ESP_LOGI(TAG, "Find devices:");
    while (found && num_devices <= max_codes)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGI(TAG, "  %d : %s", num_devices, rom_code_s);
        rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        found = owb_search_next(owb, &search_state);
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

static TempSensors * detect_sensors(uint8_t gpio)
{
    // set up the One Wire Bus
    OneWireBus * owb = owb_malloc();
    owb_init(owb, gpio);
    owb_use_crc(owb, true);       // enable CRC check for ROM code

    // locate attached devices
    OneWireBus_ROMCode * device_rom_codes = calloc(MAX_DEVICES, sizeof(*device_rom_codes));

    int num_devices = find_owb_rom_codes(owb, device_rom_codes, MAX_DEVICES);

    // free up unused space
    device_rom_codes = realloc(device_rom_codes, num_devices * sizeof(*device_rom_codes));

    // associate devices on bus with DS18B20 device driver
    DS18B20_Info ** device_infos = calloc(num_devices, sizeof(*device_infos));
    associate_ds18b20_devices(owb, device_rom_codes, device_infos, num_devices);

    TempSensors * sensors = malloc(sizeof(*sensors));
    sensors->owb = owb;
    sensors->rom_codes = device_rom_codes;
    sensors->ds18b20_infos = device_infos;
    sensors->num_ds18b20s = num_devices;

    return sensors;
}

static void sensor_task(void * pvParameter)
{
    SensorTaskInputs * inputs = (SensorTaskInputs *)pvParameter;
    int sample_count = 0;

    ESP_LOGW(TAG":sensor_task", "Core ID %d", xPortGetCoreID());

    int errors[MAX_DEVICES] = {0};
    if (inputs->sensors->num_ds18b20s > 0)
    {
        // wait a second before starting commencing sampling
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        while (1)
        {
            TickType_t start_ticks = xTaskGetTickCount();

            led_on();

            float readings[MAX_DEVICES] = { 0 };

            read_temperatures(inputs->sensors->ds18b20_infos, readings, inputs->sensors->num_ds18b20s);
            ++sample_count;

            led_off();

            // print results in a separate loop, after all have been read
            printf("\nTemperature readings (degrees C): sample %d\n", sample_count);
            for (int i = 0; i < inputs->sensors->num_ds18b20s; ++i)
            {
                // filter out invalid readings
                if (readings[i] != DS18B20_INVALID_READING)
                {
                    SensorReading sensor_reading = {
                        .sensor_id = i,
                        .value = readings[i],
                    };
                    BaseType_t status = xQueueSendToBack(inputs->publish_queue, &sensor_reading, 0);
                    if (status != pdPASS)
                    {
                        ESP_LOGE(TAG":sensor", "Could not send to queue");
                    }
                }
                else
                {
                    ++errors[i];
                }
                printf("  %d: %.1f    %d errors\n", i, readings[i], errors[i]);
            }

            // make up delay to approximate sample period
            vTaskDelay(SAMPLE_PERIOD / portTICK_PERIOD_MS - (xTaskGetTickCount() - start_ticks));
        }
    }
    else
    {
        ESP_LOGE(TAG, "No devices found");
    }

    free(inputs);
    vTaskDelete(NULL);
}

TempSensors * sensor_temp_init(uint8_t gpio, UBaseType_t priority, QueueHandle_t publish_queue)
{
    // Assume that new devices are not connected during operation.
    TempSensors * sensors = detect_sensors(gpio);

    // Blink the LED to indicate the number of temperature devices detected:
    led_flash(100, 200, sensors->num_ds18b20s);

    // task will take ownership of this struct
    SensorTaskInputs * sensor_task_inputs = malloc(sizeof(*sensor_task_inputs));
    sensor_task_inputs->sensors = sensors;
    sensor_task_inputs->publish_queue = publish_queue;
    xTaskCreate(&sensor_task, "sensor_task", 4096, sensor_task_inputs, priority, NULL);

    return sensors;
}

void sensor_temp_close(TempSensors * sensors)
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
        owb_free(&(sensors->owb));
        free(sensors);
    }
}


