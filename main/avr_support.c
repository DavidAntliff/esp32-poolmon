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

#include "constants.h"
#include "avr_support.h"
#include "i2c_master.h"
#include "smbus.h"
#include "publish.h"
#include "../avr/avr-poolmon/registers.h"

#define TAG "avr_support"

#define SMBUS_TIMEOUT 1000   // milliseconds

typedef struct
{
    i2c_master_info_t * i2c_master_info;
    QueueHandle_t publish_queue;
} task_inputs_t;

static uint8_t _read_register(const smbus_info_t * smbus_info, uint8_t address)
{
    uint8_t value = 0;
    smbus_send_byte(smbus_info, address);
    smbus_receive_byte(smbus_info, &value);
    return value;
}

static void _write_register(const smbus_info_t * smbus_info, uint8_t address, uint8_t value)
{
    smbus_write_byte(smbus_info, address, value);
}

static uint8_t _decode_switch_states(uint8_t status)
{
    uint8_t new_states = 0;
    new_states |= status & REGISTER_STATUS_SW1 ? 0b0001 : 0b0000;
    new_states |= status & REGISTER_STATUS_SW2 ? 0b0010 : 0b0000;
    new_states |= status & REGISTER_STATUS_SW3 ? 0b0100 : 0b0000;
    new_states |= status & REGISTER_STATUS_SW4 ? 0b1000 : 0b0000;
    return new_states;
}

static void _publish_switch_states(uint8_t switch_states, QueueHandle_t publish_queue)
{
    ESP_LOGI(TAG, "Publishing all switch states");
    publish_value(PUBLISH_VALUE_SWITCH_1, switch_states & 0b0001 ? 1.0 : 0.0, publish_queue);
    publish_value(PUBLISH_VALUE_SWITCH_2, switch_states & 0b0010 ? 1.0 : 0.0, publish_queue);
    publish_value(PUBLISH_VALUE_SWITCH_3, switch_states & 0b0100 ? 1.0 : 0.0, publish_queue);
    publish_value(PUBLISH_VALUE_SWITCH_4, switch_states & 0b1000 ? 1.0 : 0.0, publish_queue);
}

static void _publish_switch_changes(uint8_t last_switch_states, uint8_t new_switch_states, QueueHandle_t publish_queue)
{
    uint8_t changed = last_switch_states ^ new_switch_states;
    ESP_LOGI(TAG, "last_switch_states 0x%02x, new_switch_states 0x%02x, changed 0x%02x", last_switch_states, new_switch_states, changed);

    // Switches 1 and 3 report "0" when in Auto position, and "1" in Manual position
    // Switches 2 and 4 report "0" when in Off position, and "1" in On position.
    if (changed & 0b0001)
    {
        publish_value(PUBLISH_VALUE_SWITCH_1, new_switch_states & 0b0001 ? 1.0 : 0.0, publish_queue);
    }

    if (changed & 0b0010)
    {
        publish_value(PUBLISH_VALUE_SWITCH_2, new_switch_states & 0b0010 ? 1.0 : 0.0, publish_queue);
    }

    if (changed & 0b0100)
    {
        publish_value(PUBLISH_VALUE_SWITCH_3, new_switch_states & 0b0100 ? 1.0 : 0.0, publish_queue);
    }

    if (changed & 0b1000)
    {
        publish_value(PUBLISH_VALUE_SWITCH_4, new_switch_states & 0b1000 ? 1.0 : 0.0, publish_queue);
    }
}

static void avr_support_task(void * pvParameter)
{
    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    i2c_port_t i2c_port = task_inputs->i2c_master_info->port;

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_port, CONFIG_AVR_I2C_ADDRESS);
    smbus_set_timeout(smbus_info, SMBUS_TIMEOUT / portTICK_RATE_MS);

    uint8_t status = _read_register(smbus_info, REGISTER_STATUS);
    ESP_LOGI(TAG, "I2C %d, REG 0x01: 0x%02x", i2c_port, status);
    uint8_t switch_states = _decode_switch_states(status);
    _publish_switch_states(switch_states, task_inputs->publish_queue);

    while (1)
    {
        // read control register
        ESP_LOGI(TAG, "I2C %d, REG 0x00: 0x%02x", i2c_port, _read_register(smbus_info, REGISTER_CONTROL));
        vTaskDelay(1000 / portTICK_RATE_MS);

        // write control register
        _write_register(smbus_info, REGISTER_CONTROL, REGISTER_CONTROL_BUZZER);
        vTaskDelay(1000 / portTICK_RATE_MS);

        _write_register(smbus_info, REGISTER_CONTROL, REGISTER_CONTROL_BUZZER | REGISTER_CONTROL_SSR1);
        vTaskDelay(1000 / portTICK_RATE_MS);

        _write_register(smbus_info, REGISTER_CONTROL, REGISTER_CONTROL_BUZZER | REGISTER_CONTROL_SSR2);
        vTaskDelay(1000 / portTICK_RATE_MS);

        _write_register(smbus_info, REGISTER_CONTROL, REGISTER_CONTROL_SSR1 | REGISTER_CONTROL_SSR2);
        vTaskDelay(1000 / portTICK_RATE_MS);

        _write_register(smbus_info, REGISTER_CONTROL, REGISTER_CONTROL_SSR2);
        vTaskDelay(1000 / portTICK_RATE_MS);

        _write_register(smbus_info, REGISTER_CONTROL, REGISTER_CONTROL_SSR1);
        vTaskDelay(1000 / portTICK_RATE_MS);

        _write_register(smbus_info, REGISTER_CONTROL, 0);
        vTaskDelay(1000 / portTICK_RATE_MS);

        // read status register
        status = _read_register(smbus_info, REGISTER_STATUS);
        ESP_LOGI(TAG, "I2C %d, REG 0x01: 0x%02x", i2c_port, status);
        vTaskDelay(1000 / portTICK_RATE_MS);

        // if any switches have changed state, publish them
        uint8_t new_switch_states = _decode_switch_states(status);
        _publish_switch_changes(switch_states, new_switch_states, task_inputs->publish_queue);
        switch_states = new_switch_states;
    }

    free(task_inputs);
    vTaskDelete(NULL);
}

void avr_support_init(i2c_master_info_t * i2c_master_info, UBaseType_t priority, QueueHandle_t publish_queue)
{
    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->i2c_master_info = i2c_master_info;
        task_inputs->publish_queue = publish_queue;
        xTaskCreate(&avr_support_task, "avr_support_task", 4096, task_inputs, priority, NULL);
    }

    // set up the AVR reset line
    // and make sure it is not held in reset
//    gpio_pad_select_gpio(CONFIG_AVR_RESET_GPIO);
//    gpio_set_level(CONFIG_AVR_RESET_GPIO, 1);
//    gpio_set_direction(CONFIG_AVR_RESET_GPIO, GPIO_MODE_OUTPUT);

    // Notes:
    //   Seems that the AVR reset causes issues, even when not actually driven by the ESP32. Causes I2C bus failure.

}

void avr_support_reset(void)
{
    gpio_set_level(CONFIG_AVR_RESET_GPIO, 0);
    vTaskDelay(1);
    gpio_set_level(CONFIG_AVR_RESET_GPIO, 1);
}
