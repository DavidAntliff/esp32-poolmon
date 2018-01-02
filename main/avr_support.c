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
#include "smbus.h"

#define TAG "avr_support"

// TODO: move I2C support elsewhere
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

static void i2c_master_init(i2c_port_t i2c_port, gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_num;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // use external pullups
    conf.scl_io_num = scl_io_num;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // use external pullups
    conf.master.clk_speed = clk_speed;         // Hz
    i2c_param_config(i2c_port, &conf);
    i2c_driver_install(i2c_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

int i2c_scan(i2c_port_t i2c_num)
{
    int num_detected = 0;
    for (int address = 1; address < 0x7f; ++address)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, address << 1, true /*ACK_CHECK*/);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, (1000 / portTICK_RATE_MS));
        //ESP_LOGI(TAG, "address 0x%02x %d", address, err);
        if (err == 0)
        {
            ++num_detected;
            ESP_LOGI(TAG, "detected I2C address on master %d at address 0x%02x", i2c_num, address);
        }
        i2c_cmd_link_delete(cmd);
    }
    return num_detected;
}

static void avr_support_task(void * pvParameter)
{
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    i2c_master_init(i2c_num, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    int num_devices = i2c_scan(i2c_num);
    ESP_LOGI(TAG, "%d devices detected", num_devices);
    vTaskDelay(2000 / portTICK_RATE_MS);

    // Set up the SMBus
    int address = 0x44;
    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_num, address);
    smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);

    while (1)
    {
        // try a write to 0x44
        smbus_send_byte(smbus_info, 0x04);

        vTaskDelay(2000 / portTICK_RATE_MS);

        // try a read from 0x044
        uint8_t data = 0;
        smbus_receive_byte(smbus_info, &data);
        ESP_LOGI(TAG, "I2c recv 0x%02x", data);

        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void avr_support_init(UBaseType_t priority)
{
    xTaskCreate(&avr_support_task, "avr_support_task", 4096, NULL, priority, NULL);
}
