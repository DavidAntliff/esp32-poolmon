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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "avr_support.h"
#include "i2c_master.h"
#include "smbus.h"

#define TAG "avr_support"

static void avr_support_task(void * pvParameter)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    i2c_master_init(i2c_num, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL, I2C_MASTER_FREQ_HZ);

    int num_devices = i2c_scan(i2c_num);
    ESP_LOGI(TAG, "%d devices detected", num_devices);
    vTaskDelay(2000 / portTICK_RATE_MS);

    // Set up the SMBus
    int address = 0x44;
    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_num, address);
    smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);

    int idata = 0;
    while (1)
    {
        uint8_t data = 0;

        // write addr 0x00 then read
        smbus_send_byte(smbus_info, 0x00);
        smbus_receive_byte(smbus_info, &data);
        ESP_LOGI(TAG, "I2C REG 0x00: 0x%02x", data);

        vTaskDelay(1000 / portTICK_RATE_MS);

        // write addr 0x01 then read
        smbus_send_byte(smbus_info, 0x01);
        smbus_receive_byte(smbus_info, &data);
        ESP_LOGI(TAG, "I2C REG 0x01: 0x%02x", data);

        vTaskDelay(1000 / portTICK_RATE_MS);

        // write to AVR control register
        smbus_write_byte(smbus_info, 0x00, 0x10);
        vTaskDelay(1000 / portTICK_RATE_MS);
        smbus_write_byte(smbus_info, 0x00, 0x11);
        vTaskDelay(1000 / portTICK_RATE_MS);
        smbus_write_byte(smbus_info, 0x00, 0x12);
        vTaskDelay(1000 / portTICK_RATE_MS);
        smbus_write_byte(smbus_info, 0x00, 0x03);
        vTaskDelay(1000 / portTICK_RATE_MS);
        smbus_write_byte(smbus_info, 0x00, 0x02);
        vTaskDelay(1000 / portTICK_RATE_MS);
        smbus_write_byte(smbus_info, 0x00, 0x01);
        vTaskDelay(1000 / portTICK_RATE_MS);
        smbus_write_byte(smbus_info, 0x00, 0x00);
        vTaskDelay(1000 / portTICK_RATE_MS);

//        // try a write to 0x44
//        ESP_LOGI(TAG, "I2c send 0x%02x", idata);
//
//        smbus_send_byte(smbus_info, idata);
//        idata = (idata + 1) % 8;
//
//        vTaskDelay(2000 / portTICK_RATE_MS);
//
//        // try a read from 0x044
//        uint8_t data = 0;
//        smbus_receive_byte(smbus_info, &data);
//        ESP_LOGI(TAG, "I2c recv 0x%02x", data);
//
//        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void avr_support_init(UBaseType_t priority)
{
    xTaskCreate(&avr_support_task, "avr_support_task", 4096, NULL, priority, NULL);
}
