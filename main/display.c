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
#include "esp_log.h"

#include "display.h"
#include "constants.h"
#include "i2c_master.h"
#include "smbus.h"
#include "i2c-lcd1602.h"

#define TAG "display"

#define SMBUS_TIMEOUT     1000   // milliseconds
#define DISPLAY_WIDTH     I2C_LCD1602_NUM_VISIBLE_COLUMNS
#define ROW_STRING_WIDTH  (DISPLAY_WIDTH + 1)    // room for null terminator

#ifndef BUILD_TIMESTAMP
#  warning "Please ensure BUILD_TIMESTAMP is defined"
#  define BUILD_TIMESTAMP "undefined"
#endif

typedef enum
{
    PAGE_SPLASH = 0,
    PAGE_TEMP_VALUES,
    PAGE_LIGHT_VALUES,
    PAGE_FLOW_VALUES,
    PAGE_PUMP_VALUES,
    PAGE_ALARM_VALUES,
    PAGE_LAST_ERROR,
    PAGE_WIFI_STATUS,
    PAGE_MQTT_STATUS,
    PAGE_SENSOR_STATUS_1,
    PAGE_SENSOR_STATUS_2,
    PAGE_SYSTEM_ESP32,
    PAGE_SYSTEM_AVR_1,
    PAGE_SYSTEM_AVR_2,
    PAGE_SYSTEM_AVR_3,
    PAGE_LAST,
} page_id_t;

typedef struct
{
    i2c_master_info_t * i2c_master_info;
} task_inputs_t;

static const uint8_t degrees_C[8]  = { 0x10, 0x06, 0x09, 0x08, 0x08, 0x09, 0x06, 0x00 };

static bool _display_splash(const i2c_lcd1602_info_t * lcd_info, uint8_t major_version, uint8_t minor_version, const char * build_time)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));

    char line[ROW_STRING_WIDTH] = "";
    snprintf(line, ROW_STRING_WIDTH, "PoolControl v%d.%d", major_version, minor_version);
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line));

    snprintf(line, ROW_STRING_WIDTH, " %s", build_time);
    ESP_ERROR_CHECK(i2c_lcd1602_move_cursor(lcd_info, 0, 1));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line));

    return false;  // no refresh required - values won't change dynamically
}

static bool _display_temp_values(const i2c_lcd1602_info_t * lcd_info)
{
    // TODO: perhaps avoid clearing the display if we've already drawn it?
    // Need to maintain a page state - could use static vars, but need to know when it's a "first" call

    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));

    char line[ROW_STRING_WIDTH] = "";
//    snprintf(line, ROW_STRING_WIDTH, "Temp%cC %2.1f %2.1f", I2C_LCD1602_CHARACTER_DEGREE, 18.1, 20.2);
//    snprintf(line, ROW_STRING_WIDTH, "TempC  %2.1f %2.1f", 18.1, 20.2);
    snprintf(line, ROW_STRING_WIDTH, "Temp%c  %2.1f %2.1f", I2C_LCD1602_CHARACTER_CUSTOM_0, 18.1, 20.2);
//    snprintf(line, ROW_STRING_WIDTH, "Temp\x08  %2.1f %2.1f", 18.1, 20.2);
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line));

    snprintf(line, ROW_STRING_WIDTH, "  %2.1f %2.1f %2.1f", 22.9, 21.8, 21.7);
    ESP_ERROR_CHECK(i2c_lcd1602_move_cursor(lcd_info, 0, 1));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line));

    return true;  // refresh required - values may change
}

static page_id_t get_next_page_id(page_id_t current_page)
{
    // for now, step through pages
    page_id_t page = (current_page + 1) % PAGE_LAST;

    switch (current_page)
    {
    case PAGE_SPLASH:
        page = PAGE_TEMP_VALUES;
        break;
    case PAGE_TEMP_VALUES:
        page = PAGE_SPLASH;
        break;
    case PAGE_LIGHT_VALUES:
        break;
    case PAGE_FLOW_VALUES:
        break;
    case PAGE_PUMP_VALUES:
        break;
    case PAGE_ALARM_VALUES:
        break;
    case PAGE_LAST_ERROR:
        break;
    case PAGE_WIFI_STATUS:
        break;
    case PAGE_MQTT_STATUS:
        break;
    case PAGE_SENSOR_STATUS_1:
        break;
    case PAGE_SENSOR_STATUS_2:
        break;
    case PAGE_SYSTEM_ESP32:
        break;
    case PAGE_SYSTEM_AVR_1:
        break;
    case PAGE_SYSTEM_AVR_2:
        break;
    case PAGE_SYSTEM_AVR_3:
        break;
    default:
        ESP_LOGE(TAG, "invalid page");
        page = PAGE_SPLASH;
        break;
    }

    return page;
}

static bool display_page(i2c_lcd1602_info_t * lcd_info, page_id_t current_page, bool refresh)
{
    if (refresh)
    {
        switch (current_page)
        {
        case PAGE_SPLASH:
            refresh = _display_splash(lcd_info, VERSION_MAJOR, VERSION_MINOR, BUILD_TIMESTAMP);
            break;
        case PAGE_TEMP_VALUES:
            refresh = _display_temp_values(lcd_info);
            break;
        case PAGE_LIGHT_VALUES:
            break;
        case PAGE_FLOW_VALUES:
            break;
        case PAGE_PUMP_VALUES:
            break;
        case PAGE_ALARM_VALUES:
            break;
        case PAGE_LAST_ERROR:
            break;
        case PAGE_WIFI_STATUS:
            break;
        case PAGE_MQTT_STATUS:
            break;
        case PAGE_SENSOR_STATUS_1:
            break;
        case PAGE_SENSOR_STATUS_2:
            break;
        case PAGE_SYSTEM_ESP32:
            break;
        case PAGE_SYSTEM_AVR_1:
            break;
        case PAGE_SYSTEM_AVR_2:
            break;
        case PAGE_SYSTEM_AVR_3:
            break;
        default:
            ESP_LOGE(TAG, "invalid page");
            break;
        }
    }
    return refresh;
}

static void display_task(void * pvParameter)
{
    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    i2c_port_t i2c_port = task_inputs->i2c_master_info->port;

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_port, CONFIG_LCD1602_I2C_ADDRESS));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, SMBUS_TIMEOUT / portTICK_RATE_MS));

    // Set up the LCD1602 device with backlight on
    i2c_lcd1602_info_t * lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true));

    // Define custom characters
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_0, degrees_C);

    // Move to home position
    ESP_ERROR_CHECK(i2c_lcd1602_move_cursor(lcd_info, 0, 0));

    page_id_t current_page = PAGE_SPLASH;

    // set to true if the display needs to be refreshed
    bool refresh = true;
    while (1)
    {
        // don't bother updating display if it's not required
        refresh = display_page(lcd_info, current_page, refresh);

        current_page = get_next_page_id(current_page);
        refresh = true;

        // update any dynamic pages once per second
        ESP_LOGI(TAG, "display loop");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    free(task_inputs);
    vTaskDelete(NULL);
}

void display_init(i2c_master_info_t * i2c_master_info, UBaseType_t priority)
{
    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->i2c_master_info = i2c_master_info;
        xTaskCreate(&display_task, "display_task", 4096, task_inputs, priority, NULL);
    }
}
