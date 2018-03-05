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
#include "driver/i2c.h"
#include "esp_log.h"

#include "i2c_master.h"

#define TAG "i2c"

i2c_master_info_t * i2c_master_init(i2c_port_t i2c_port, gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    i2c_master_info_t * info = malloc(sizeof(*info));
    if (info)
    {
        memset(info, 0, sizeof(*info));
        info->port = i2c_port;
        info->config.mode = I2C_MODE_MASTER;
        info->config.sda_io_num = sda_io_num;
        info->config.sda_pullup_en = GPIO_PULLUP_DISABLE;  // use external pullups
        info->config.scl_io_num = scl_io_num;
        info->config.scl_pullup_en = GPIO_PULLUP_DISABLE;  // use external pullups
        info->config.master.clk_speed = clk_speed;         // Hz

        ESP_ERROR_CHECK(i2c_param_config(i2c_port, &info->config));
        ESP_ERROR_CHECK(i2c_driver_install(i2c_port, info->config.mode,
                                           I2C_MASTER_RX_BUF_LEN,
                                           I2C_MASTER_TX_BUF_LEN, 0));
        info->semaphore = xSemaphoreCreateMutex();
    }
    return info;
}

int i2c_master_scan(const i2c_master_info_t * info)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    int num_detected = 0;
    if (info)
    {
        xSemaphoreTake(info->semaphore, portMAX_DELAY);
        for (int address = 1; address < 0x7f; ++address)
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, address << 1, true /*ACK_CHECK*/);
            i2c_master_stop(cmd);
            esp_err_t err = i2c_master_cmd_begin(info->port, cmd, (1000 / portTICK_RATE_MS));
            //ESP_LOGI(TAG, "address 0x%02x %d", address, err);
            if (err == 0)
            {
                ++num_detected;
                ESP_LOGI(TAG, "detected I2C address on master %d at address 0x%02x", info->port, address);
            }
            i2c_cmd_link_delete(cmd);
        }
        xSemaphoreGive(info->semaphore);
    }
    else
    {
        ESP_LOGE(TAG, "info is NULL");
    }
    return num_detected;
}

void i2c_master_close(i2c_master_info_t * info)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    free(info);
}

bool i2c_master_lock(const i2c_master_info_t * info, TickType_t ticks_to_wait)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    bool result = false;
    if (info)
    {
        result = xSemaphoreTake(info->semaphore, ticks_to_wait) == pdTRUE ? true : false;
    }
    else
    {
        ESP_LOGE(TAG, "info is NULL");
    }    return result;
}

void i2c_master_unlock(const i2c_master_info_t * info)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    if (info)
    {
        xSemaphoreGive(info->semaphore);
    }
    else
    {
        ESP_LOGE(TAG, "info is NULL");
    }
}
