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
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_log.h"

#include "owb.h"
#include "ds18b20.h"

#define GPIO_LED             (GPIO_NUM_2)
#define GPIO_ONE_WIRE        (CONFIG_ONE_WIRE_GPIO)
#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_10_BIT)
#define SAMPLE_PERIOD        (10000)  // sensor sampling period in milliseconds

static const char * TAG = "poolmon";

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

    ESP_LOGD(TAG, "Find devices:");
    while (found && num_devices <= max_codes)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGD(TAG, "  %d : %s", num_devices, rom_code_s);
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

//TODO: LED task, to blink LED when required
// - count number of connected devices
// - indicate when sampling
// - indicate MQTT connection state
// - indicate MQTT activity (publish, receive)
void led_init(void)
{
    gpio_pad_select_gpio(GPIO_LED);
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
}

void led_on(void)
{
    gpio_set_level(GPIO_LED, 1);
}

void led_off(void)
{
    gpio_set_level(GPIO_LED, 0);
}

void led_indicate_num_devices(int num_devices)
{
    for (int i = 0; i < num_devices; ++i)
    {
        led_on();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_off();
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void read_temperatures(DS18B20_Info ** device_infos, float * readings, int num_devices)
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

void sensor_task(void *pvParameter)
{
    // set up LED GPIO
    led_init();

    // set up the One Wire Bus
    OneWireBus * owb = owb_malloc();
    owb_init(owb, GPIO_ONE_WIRE);
    owb_use_crc(owb, true);       // enable CRC check for ROM code

    // locate attached devices
    OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
    int num_devices = find_owb_rom_codes(owb, device_rom_codes, MAX_DEVICES);

    // associate devices on bus with DS18B20 device driver
    DS18B20_Info * device_infos[MAX_DEVICES] = {0};
    associate_ds18b20_devices(owb, device_rom_codes, device_infos, (num_devices < MAX_DEVICES) ? num_devices : MAX_DEVICES);

    // blink the LED to indicate the number of devices detected:
    led_indicate_num_devices(num_devices);

    if (num_devices > 0)
    {
        // wait a second before starting sampling
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        while (1)
        {
            TickType_t start_ticks = xTaskGetTickCount();

            led_on();

            float readings[MAX_DEVICES] = { 0 };
            read_temperatures(device_infos, readings, num_devices);

            led_off();

            // print results in a separate loop, after all have been read
            printf("\nTemperature readings (degrees C):\n");
            for (int i = 0; i < num_devices; ++i)
            {
                printf("  %d: %.1f\n", i, readings[i]);
            }

            // make up delay to approximate sample period
            vTaskDelay(SAMPLE_PERIOD / portTICK_PERIOD_MS - (xTaskGetTickCount() - start_ticks));
        }
    }

    // clean up dynamically allocated data
    for (int i = 0; i < num_devices; ++i)
    {
        ds18b20_free(&device_infos[i]);
    }
    owb_free(&owb);

    vTaskDelete(NULL);
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_DEBUG);
    ESP_LOGD(TAG, "PoolMon-ESP32");

    xTaskCreate(&sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
