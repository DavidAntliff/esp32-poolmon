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

#define SMBUS_TIMEOUT 1000   // milliseconds
#define DISPLAY_WIDTH I2C_LCD1602_NUM_VISIBLE_COLUMNS

#define BUILD "goo"
#ifndef BUILD
#  warning "Please ensure the build timestamp is defined"
#  define BUILD "undefined"
#endif

typedef struct
{
    i2c_master_info_t * i2c_master_info;
} task_inputs_t;

static void _splash(const i2c_lcd1602_info_t * lcd_info, uint8_t major_version, uint8_t minor_version, const char * build_time)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));

    char line0[DISPLAY_WIDTH + 1] = "";
    snprintf(line0, DISPLAY_WIDTH, "PoolControl v%d.%d", major_version, minor_version);
    char line1[DISPLAY_WIDTH + 1] = "";
    snprintf(line1, DISPLAY_WIDTH, " %s", build_time);

    ESP_ERROR_CHECK(i2c_lcd1602_move_cursor(lcd_info, 0, 0));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line0));
    ESP_ERROR_CHECK(i2c_lcd1602_move_cursor(lcd_info, 0, 1));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line1));
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
    ESP_ERROR_CHECK(i2c_lcd1602_move_cursor(lcd_info, 0, 0));

    _splash(lcd_info, VERSION_MAJOR, VERSION_MINOR, BUILD);

    while (1)
    {
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
