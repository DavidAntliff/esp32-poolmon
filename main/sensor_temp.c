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
#include "resources.h"
#include "utils.h"
#include "led.h"
#include "publish.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "datastore/datastore.h"
#include "display.h"

#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_10_BIT)

#define TAG "sensor_temp"

struct _temp_sensors_t
{
    OneWireBus * owb;
    OneWireBus_ROMCode * rom_codes;   // a pointer to an *array* of ROM codes
    DS18B20_Info ** ds18b20_infos;
    int num_ds18b20s;
};

typedef struct
{
    temp_sensors_t * sensors;
    const datastore_t * datastore;
} task_inputs_t;

static void read_temperatures(DS18B20_Info ** device_infos, float * readings, DS18B20_ERROR * errors, int num_devices)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    assert(readings);
    assert(errors);

    ds18b20_convert_all(device_infos[0]->bus);

    // in this application all devices use the same resolution,
    // so use the first device to determine the delay
    ds18b20_wait_for_conversion(device_infos[0]);

    // read the results immediately after conversion otherwise it may fail
    // (using printf before reading may take too long)
    for (int i = 0; i < num_devices; ++i)
    {
        errors[i] = ds18b20_read_temp(device_infos[i], &readings[i]);
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
    ESP_LOGD(TAG, "%s", __FUNCTION__);
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
    ESP_LOGD(TAG, "%s", __FUNCTION__);
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
    ESP_LOGD(TAG, "%s", __FUNCTION__);
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

typedef struct
{
    int * map;
    OneWireBus_ROMCode * rom_codes;
} context_t;

void _recalc_assignments_handler(const datastore_t * datastore, datastore_resource_id_t id, datastore_instance_id_t instance, void * ctxt)
{
    ESP_LOGD(TAG, "_recalc_assignments: datastore %p, resource id %d, instance id %d, ctxt %p", datastore, id, instance, ctxt);
    context_t * context = (context_t *)ctxt;
    assert(context != NULL);

    // get the assigned rom code
    char rom_code_str[SENSOR_TEMP_LEN_ROM_CODE] = "";
    datastore_get_as_string(datastore, id, instance, rom_code_str, SENSOR_TEMP_LEN_ROM_CODE);
    ESP_LOGD(TAG, "rom_code_str [%s]", rom_code_str);

    // convert ROM code as string into OneWireBus_ROMCode - note that rom code is LSB-first
    OneWireBus_ROMCode rom_code = { 0 };
    for (size_t i = 0; i < sizeof(OneWireBus_ROMCode); ++i)
    {
        char * in = &rom_code_str[2 * (sizeof(OneWireBus_ROMCode) - i - 1)];  // reverse order because LSB is first
        uint8_t hn = in[0] > '9' ? (in[0] | 0x20) - 'a' + 10 : in[0] - '0';
        uint8_t ln = in[1] > '9' ? (in[1] | 0x20) - 'a' + 10 : in[1] - '0';
        rom_code.bytes[i] = (hn << 4) | ln;
    }
    ESP_LOG_BUFFER_HEXDUMP(TAG, &rom_code, sizeof(rom_code), ESP_LOG_DEBUG);

    // search for match between assignment rom code and detected rom codes
    if (context->rom_codes != NULL)
    {
        for (size_t i = 0; i < SENSOR_TEMP_INSTANCES; ++i)
        {
            ESP_LOG_BUFFER_HEXDUMP(TAG, &context->rom_codes[i], sizeof(rom_code), ESP_LOG_DEBUG);
            if (memcmp(&rom_code, &context->rom_codes[i], sizeof(OneWireBus_ROMCode)) == 0)
            {
                context->map[instance] = i;
                ESP_LOGI(TAG, "Assigned rom code [%s] (detected instance %d) to T%d", rom_code_str, i, instance + 1);
                break;
            }
        }
    }
}

static void sensor_temp_task(void * pvParameter)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    const datastore_t * datastore = task_inputs->datastore;

    int sample_count = 0;

    // Map sensor instances (enumerated in ROM code order as detected) with T1, ..., T5 instances
    int map[SENSOR_TEMP_INSTANCES] = { -1, -1, -1, -1, -1 };
    context_t context = {
        .map = &map[0],
        .rom_codes = &task_inputs->sensors->rom_codes[0],
    };

    // apply any default or nvs-loaded assignments
    for (size_t i = 0; i < SENSOR_TEMP_INSTANCES; ++i)
    {
        _recalc_assignments_handler(datastore, RESOURCE_ID_TEMP_ASSIGNMENT, i, &context);
    }

    // Callback if any assignments change
    datastore_add_set_callback(datastore, RESOURCE_ID_TEMP_ASSIGNMENT, _recalc_assignments_handler, &context);

    int errors_count[MAX_DEVICES] = {0};
    if (task_inputs->sensors->num_ds18b20s > 0)
    {
        // wait a second before starting commencing sampling
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        TickType_t last_wake_time = xTaskGetTickCount();

        while (1)
        {
            last_wake_time = xTaskGetTickCount();

            bool control_led = display_is_currently(datastore, DISPLAY_PAGE_SENSORS_TEMP) || display_is_currently(datastore, DISPLAY_PAGE_SENSORS_TEMP_2);
            if (control_led)
            {
                led_on();
            }

            float readings[MAX_DEVICES] = { 0 };
            DS18B20_ERROR errors[MAX_DEVICES] = { 0 };

            read_temperatures(task_inputs->sensors->ds18b20_infos, readings, errors, task_inputs->sensors->num_ds18b20s);
            ++sample_count;

            if (control_led)
            {
                led_off();
            }

            // print results in a separate loop, after all have been read
            ESP_LOGI(TAG, "Temperature readings (degrees C): sample %d", sample_count);
            for (int i = 0; i < task_inputs->sensors->num_ds18b20s; ++i)
            {
                // map actual sensors to T1, ..., T4 assignments
                float reading = map[i] >= 0 ? readings[map[i]] : -2048.0;
                int num_errors = map[i] >= 0 ? errors_count[map[i]] : 0;
                DS18B20_ERROR error = map[i] >= 0 ? errors[map[i]] : DS18B20_OK;

                datastore_age_t override_age = 0;
                datastore_get_age(datastore, RESOURCE_ID_TEMP_OVERRIDE, i, &override_age);
                if (override_age < (esp_timer_get_time() - 10))
                {
                    float override_value = 0.0f;
                    datastore_get_float(datastore, RESOURCE_ID_TEMP_OVERRIDE, i, &override_value);
                    datastore_set_float(datastore, RESOURCE_ID_TEMP_VALUE, i, override_value);
                    ESP_LOGI(TAG, "  T%d: %.1f    %d errors  OVERRIDE", i + 1, override_value, num_errors);
                }
                else
                {
                    // filter out unmapped and errored readings
                    if (map[i] >= 0)
                    {
                        if (error == DS18B20_OK)
                        {
                            datastore_set_float(datastore, RESOURCE_ID_TEMP_VALUE, i, reading);
                            ESP_LOGI(TAG, "  T%d: %.1f    %d errors", i + 1, reading, num_errors);
                        }
                        else
                        {
                            ++errors_count[map[i]];
                        }
                    }
                }
            }

            uint32_t poll_period = 0;
            datastore_get_uint32(datastore, RESOURCE_ID_TEMP_PERIOD, 0, &poll_period);
            vTaskDelayUntil(&last_wake_time, poll_period / portTICK_PERIOD_MS);
        }
    }
    else
    {
        ESP_LOGE(TAG, "No devices found");
    }

    free(task_inputs);
    vTaskDelete(NULL);
}

static void sensor_temp_sim_task(void * pvParameter)
{
    // When no devices are detected, this task can be used to simulate temperature sensors
    // The override resources are also available.

    ESP_LOGD(TAG, "%s", __FUNCTION__);
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    const datastore_t * datastore = task_inputs->datastore;

    int sample_count = 0;

    // wait a second before starting commencing sampling
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        last_wake_time = xTaskGetTickCount();

        bool control_led = display_is_currently(datastore, DISPLAY_PAGE_SENSORS_TEMP) || display_is_currently(datastore, DISPLAY_PAGE_SENSORS_TEMP_2);
        if (control_led)
        {
            led_on();
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
        ++sample_count;

        if (control_led)
        {
            led_off();
        }

        ESP_LOGI(TAG, "Temperature readings (degrees C): sample %d ** SIMULATED **", sample_count);
        for (int i = 0; i < SENSOR_TEMP_INSTANCES; ++i)
        {
            float reading = -2048.0;
            int num_errors = -1;  // indicate that something isn't quite right about these measurements
            datastore_age_t override_age = 0;
            datastore_get_age(datastore, RESOURCE_ID_TEMP_OVERRIDE, i, &override_age);
            bool override = override_age < (esp_timer_get_time() - 10);

            if (override)
            {
                datastore_get_float(datastore, RESOURCE_ID_TEMP_OVERRIDE, i, &reading);
            }
            else
            {
                reading = 20.0 + 1.0 * i;
            }

            datastore_set_float(datastore, RESOURCE_ID_TEMP_VALUE, i, reading);
            ESP_LOGI(TAG, "  T%d: %.1f    %d errors%s", i + 1, reading, num_errors, override ? " OVERRIDE" : " SIMULATED");
        }

        uint32_t poll_period = 0;
        datastore_get_uint32(datastore, RESOURCE_ID_TEMP_PERIOD, 0, &poll_period);
        vTaskDelayUntil(&last_wake_time, poll_period / portTICK_PERIOD_MS);
    }

    free(task_inputs);
    vTaskDelete(NULL);
}

temp_sensors_t * sensor_temp_init(uint8_t gpio, UBaseType_t priority, const datastore_t * datastore)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    // Assume that new devices are not connected during operation.
    temp_sensors_t * sensors = detect_sensors(gpio);

    // assignments are made by the user (via a list of detected devices) and stored in NV.

    // publish detected ROM codes
    for (size_t i = 0; i < sensors->num_ds18b20s; ++i)
    {
        // render device ID (LSB first) as sixteen hex digits
        char rom_code[2 * sizeof(OneWireBus_ROMCode) + 1] = "";
        for (size_t j = 0; j < sizeof(OneWireBus_ROMCode); ++j)
        {
            char byte[3] = "";
            snprintf(byte, 3, "%02x", sensors->ds18b20_infos[i]->rom_code.bytes[sizeof(OneWireBus_ROMCode) - j - 1]);
            strcat(rom_code, byte);
        }
        datastore_set_string(datastore, RESOURCE_ID_TEMP_DETECTED, i, rom_code);
    }

    // Blink the LED to indicate the number of temperature devices detected:
    led_flash(100, 200, sensors->num_ds18b20s);

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->sensors = sensors;
        task_inputs->datastore = datastore;
        if (sensors->num_ds18b20s > 0)
        {
            xTaskCreate(&sensor_temp_task, "sensor_temp_task", 4096, task_inputs, priority, NULL);
        }
        else
        {
            ESP_LOGW(TAG, "No temperature sensors detected - using simulated sensors");
            xTaskCreate(&sensor_temp_sim_task, "sensor_temp_task", 4096, task_inputs, priority, NULL);
        }
    }
    return sensors;
}

datastore_age_t sensor_temp_expiry(const datastore_t * datastore)
{
    // 1.5x the temp poll period
    uint32_t poll_period = 0;
    datastore_get_uint32(datastore, RESOURCE_ID_TEMP_PERIOD, 0, &poll_period);
    datastore_age_t expiry = (3 * poll_period * 1000) / 2;  // microseconds
    return expiry;
}

void sensor_temp_close(temp_sensors_t * sensors)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
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


