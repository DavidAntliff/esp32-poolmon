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

#include "avr_support.h"
#include "constants.h"
#include "utils.h"
#include "i2c_master.h"
#include "smbus.h"
#include "publish.h"
#include "resources.h"
#include "../avr/avr-poolmon/registers.h"
#include "datastore/datastore.h"

#define TAG "avr_support"

#define SMBUS_TIMEOUT     1000   // milliseconds
#define TICKS_PER_UPDATE  (250 / portTICK_RATE_MS)
#define EXPECTED_ID       0x44
#define EXPECTED_VERSION  1
#define SCRATCH_VALUE     0x55

// use as bitwise OR combinations
typedef enum
{
    // AVR reset command
    COMMAND_AVR_RESET    = 0b10000000,

    // control commands
    COMMAND_OFF          = 0b01000000,
    COMMAND_ON           = 0b00100000,

    // select commands
    COMMAND_SELECT_ALARM = 0b00000001,
    COMMAND_SELECT_CP    = 0b00000010,
    COMMAND_SELECT_PP    = 0b00000100,

} command_t;

static QueueHandle_t command_queue;

typedef struct
{
    i2c_master_info_t * i2c_master_info;
    const datastore_t * datastore;
} task_inputs_t;

static TaskHandle_t _task_handle = NULL;

#define I2C_ERROR_CHECK(x) do {                                             \
        esp_err_t rc = (x);                                                 \
        if (rc != ESP_OK) {                                                 \
            ESP_LOGW(TAG, "I2C error %d at %s:%d", rc, __FILE__, __LINE__); \
        }                                                                   \
    } while(0);

static uint8_t _read_register(const smbus_info_t * smbus_info, uint8_t address)
{
    uint8_t value = 0;
    I2C_ERROR_CHECK(smbus_send_byte(smbus_info, address));
    I2C_ERROR_CHECK(smbus_receive_byte(smbus_info, &value));
    return value;
}

static void _write_register(const smbus_info_t * smbus_info, uint8_t address, uint8_t value)
{
    I2C_ERROR_CHECK(smbus_write_byte(smbus_info, address, value));
}

static uint8_t _decode_switch_states(uint8_t status)
{
    uint8_t new_states = 0;
    new_states |= status & AVR_REGISTER_STATUS_SW1 ? 0b0001 : 0b0000;
    new_states |= status & AVR_REGISTER_STATUS_SW2 ? 0b0010 : 0b0000;
    new_states |= status & AVR_REGISTER_STATUS_SW3 ? 0b0100 : 0b0000;
    new_states |= status & AVR_REGISTER_STATUS_SW4 ? 0b1000 : 0b0000;
    return new_states;
}

static void _publish_switch_states(uint8_t switch_states, const datastore_t * datastore)
{
    ESP_LOGD(TAG, "Publishing all switch states");
    datastore_set_uint32(datastore, RESOURCE_ID_SWITCHES_CP_MODE_VALUE, 0, switch_states & 0b0001 ? 1.0 : 0.0);
    datastore_set_uint32(datastore, RESOURCE_ID_SWITCHES_CP_MAN_VALUE,  0, switch_states & 0b0010 ? 1.0 : 0.0);
    datastore_set_uint32(datastore, RESOURCE_ID_SWITCHES_PP_MODE_VALUE, 0, switch_states & 0b0100 ? 1.0 : 0.0);
    datastore_set_uint32(datastore, RESOURCE_ID_SWITCHES_PP_MAN_VALUE,  0, switch_states & 0b1000 ? 1.0 : 0.0);
}

static void _publish_switch_changes(uint8_t last_switch_states, uint8_t new_switch_states, const datastore_t * datastore)
{
    uint8_t changed = last_switch_states ^ new_switch_states;
    ESP_LOGD(TAG, "last_switch_states 0x%02x, new_switch_states 0x%02x, changed 0x%02x", last_switch_states, new_switch_states, changed);

    // Switches 1 and 3 report "0" when in Auto position, and "1" in Manual position
    // Switches 2 and 4 report "0" when in Off position, and "1" in On position.
    if (changed & 0b0001)
    {
        datastore_set_uint32(datastore, RESOURCE_ID_SWITCHES_CP_MODE_VALUE, 0, new_switch_states & 0b0001 ? 1.0 : 0.0);
    }

    if (changed & 0b0010)
    {
        datastore_set_uint32(datastore, RESOURCE_ID_SWITCHES_CP_MAN_VALUE,  0, new_switch_states & 0b0010 ? 1.0 : 0.0);
    }

    if (changed & 0b0100)
    {
        datastore_set_uint32(datastore, RESOURCE_ID_SWITCHES_PP_MODE_VALUE, 0, new_switch_states & 0b0100 ? 1.0 : 0.0);
    }

    if (changed & 0b1000)
    {
        datastore_set_uint32(datastore, RESOURCE_ID_SWITCHES_PP_MAN_VALUE,  0, new_switch_states & 0b1000 ? 1.0 : 0.0);
    }
}

static uint8_t _decode_pump_states(uint8_t status)
{
    uint8_t new_states = 0;
    new_states |= status & AVR_REGISTER_STATUS_SSR1 ? 0b0001 : 0b0000;
    new_states |= status & AVR_REGISTER_STATUS_SSR2 ? 0b0010 : 0b0000;
    return new_states;
}

static void _publish_pump_states(uint8_t pump_states, const datastore_t * datastore)
{
    ESP_LOGD(TAG, "Publishing all pump states");
    datastore_set_uint32(datastore, RESOURCE_ID_PUMPS_CP_STATE, 0, pump_states & 0b0001 ? 1.0 : 0.0);
    datastore_set_uint32(datastore, RESOURCE_ID_PUMPS_PP_STATE, 0, pump_states & 0b0010 ? 1.0 : 0.0);
}

static void _publish_pump_changes(uint8_t last_pump_states, uint8_t new_pump_states, const datastore_t * datastore)
{
    uint8_t changed = last_pump_states ^ new_pump_states;
    ESP_LOGD(TAG, "last_pump_states 0x%02x, new_pump_states 0x%02x, changed 0x%02x", last_pump_states, new_pump_states, changed);

    // Pumps report "0" when in Off position, and "1" in On position.
    if (changed & 0b0001)
    {
        datastore_set_uint32(datastore, RESOURCE_ID_PUMPS_CP_STATE, 0, new_pump_states & 0b0001 ? 1.0 : 0.0);
    }

    if (changed & 0b0010)
    {
        datastore_set_uint32(datastore, RESOURCE_ID_PUMPS_PP_STATE,  0, new_pump_states & 0b0010 ? 1.0 : 0.0);
    }
}

static bool _test_command(command_t command, command_t flag)
{
    // handle multi-bit flags
    return (command & flag) == flag;
}

static void avr_support_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    i2c_master_info_t * i2c_master_info = task_inputs->i2c_master_info;
    i2c_port_t i2c_port = i2c_master_info->port;

    // before accessing I2C, use a lock to gain exclusive use of the bus
    i2c_master_lock(i2c_master_info, portMAX_DELAY);

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, i2c_port, CONFIG_AVR_I2C_ADDRESS);
    I2C_ERROR_CHECK(smbus_set_timeout(smbus_info, SMBUS_TIMEOUT / portTICK_RATE_MS));

    // Verify the ID register
    uint8_t id = _read_register(smbus_info, AVR_REGISTER_ID);
    ESP_LOGD(TAG, "ID %d", id);
    if (id != EXPECTED_ID)
    {
        ESP_LOGE(TAG, "Unexpected device at I2C address 0x%02x: ID 0x%02x, expected 0x%02x", CONFIG_AVR_I2C_ADDRESS, id, EXPECTED_ID);
        goto stop;
    }

    // Verify the Version register
    uint8_t version = _read_register(smbus_info, AVR_REGISTER_VERSION);
    ESP_LOGD(TAG, "Version %d", version);
    if (version != EXPECTED_VERSION)
    {
        ESP_LOGE(TAG, "Unexpected firmware version %d, expected 0x%02x", version, EXPECTED_VERSION);
        goto stop;
    }
    else
    {
        ESP_LOGI(TAG, "Firmware version %d", version);
        datastore_set_uint8(task_inputs->datastore, RESOURCE_ID_AVR_VERSION, 0, version);
    }

    uint8_t status = _read_register(smbus_info, AVR_REGISTER_STATUS);
    ESP_LOGD(TAG, "I2C %d, REG 0x01: 0x%02x", i2c_port, status);
    uint8_t switch_states = _decode_switch_states(status);
    _publish_switch_states(switch_states, task_inputs->datastore);

    uint8_t pump_states = _decode_pump_states(status);
    _publish_pump_states(pump_states, task_inputs->datastore);

    // write a known value to scratch register
    _write_register(smbus_info, AVR_REGISTER_SCRATCH, SCRATCH_VALUE);

    i2c_master_unlock(i2c_master_info);

    uint8_t control_reg = 0x0;

    TickType_t last_wake_time = xTaskGetTickCount();

    // Issue #3 - inhibit circulation pump operation when the purge pump is active.
    // Maintain the last set state of the circulation and purge pumps.
    // When the purge pump is enabled, disable the circulation pump.
    // When the purge pump is disabled, restore the circulation pump state.
    // If the circulation pump state is changed by a command, update the circulation pump state.
    avr_pump_state_t last_cp_state = AVR_PUMP_STATE_OFF;
    avr_pump_state_t last_pp_state = AVR_PUMP_STATE_OFF;

    while (1)
    {
        command_t command = 0;
        while (xQueueReceive(command_queue, &command, 0) == pdTRUE)
        {
            char binary_digits[9] = "";
            uint8_t command8 = command;
            ESP_LOGD(TAG, "command 0x%02x, 0b%s", command, bits_to_string(binary_digits, 9, &command8, sizeof(command8)));

            if (_test_command(command, COMMAND_ON))
            {
                if (_test_command(command, COMMAND_SELECT_CP))
                {
                    last_cp_state = AVR_PUMP_STATE_ON;

                    // If the PP is on, do not engage the CP pump
                    if (last_pp_state == AVR_PUMP_STATE_ON)
                    {
                        ESP_LOGI(TAG, "CP on (inhibited by PP)");
                    }
                    else
                    {
                        ESP_LOGI(TAG, "CP on");
                        control_reg |= AVR_REGISTER_CONTROL_SSR1;
                    }
                }
                if (_test_command(command, COMMAND_SELECT_PP))
                {
                    ESP_LOGI(TAG, "PP on");
                    last_pp_state = AVR_PUMP_STATE_ON;
                    control_reg |= AVR_REGISTER_CONTROL_SSR2;

                    // if CP is on, turn it off
                    if (last_cp_state == AVR_PUMP_STATE_ON)
                    {
                        ESP_LOGI(TAG, "CP off (inhibited by PP)");
                        control_reg &= ~AVR_REGISTER_CONTROL_SSR1;
                    }
                }
                if (_test_command(command, COMMAND_SELECT_ALARM))
                {
                    ESP_LOGI(TAG, "Alarm on");
                    control_reg |= AVR_REGISTER_CONTROL_BUZZER;
                }
                i2c_master_lock(i2c_master_info, portMAX_DELAY);
                _write_register(smbus_info, AVR_REGISTER_CONTROL, control_reg);
                i2c_master_unlock(i2c_master_info);
            }
            else if (_test_command(command, COMMAND_OFF))
            {
                if (_test_command(command, COMMAND_SELECT_CP))
                {
                    ESP_LOGI(TAG, "CP off");
                    last_cp_state = AVR_PUMP_STATE_OFF;
                    control_reg &= ~AVR_REGISTER_CONTROL_SSR1;
                }
                if (_test_command(command, COMMAND_SELECT_PP))
                {
                    ESP_LOGI(TAG, "PP off");
                    last_pp_state = AVR_PUMP_STATE_OFF;
                    control_reg &= ~AVR_REGISTER_CONTROL_SSR2;

                    // If the CP pump *was* on, or should be on, turn it back on now
                    if (last_cp_state == AVR_PUMP_STATE_ON)
                    {
                        ESP_LOGI(TAG, "CP on (PP inhibit removed)");
                        control_reg |= AVR_REGISTER_CONTROL_SSR1;
                    }
                }
                if (_test_command(command, COMMAND_SELECT_ALARM))
                {
                    ESP_LOGI(TAG, "Alarm off");
                    control_reg &= ~AVR_REGISTER_CONTROL_BUZZER;
                }
                i2c_master_lock(i2c_master_info, portMAX_DELAY);
                _write_register(smbus_info, AVR_REGISTER_CONTROL, control_reg);
                i2c_master_unlock(i2c_master_info);            }

            if (_test_command(command, COMMAND_AVR_RESET))
            {
                ESP_LOGI(TAG, "AVR reset");
                i2c_master_lock(i2c_master_info, portMAX_DELAY);
                gpio_set_level(CONFIG_AVR_RESET_GPIO, 0);
                vTaskDelay(10);
                gpio_set_level(CONFIG_AVR_RESET_GPIO, 1);

                // give the I2C bus some time to stabilise after AVR reset
                vTaskDelay(10);
                i2c_master_unlock(i2c_master_info);
            }
        }

        i2c_master_lock(i2c_master_info, portMAX_DELAY);

        // read control register
        ESP_LOGD(TAG, "I2C %d, REG 0x00: 0x%02x", i2c_port, _read_register(smbus_info, AVR_REGISTER_CONTROL));

        // read status register
        status = _read_register(smbus_info, AVR_REGISTER_STATUS);
        ESP_LOGD(TAG, "I2C %d, REG 0x01: 0x%02x", i2c_port, status);

        // read value from scratch register
        // compare with expected value
        uint8_t scratch = _read_register(smbus_info, AVR_REGISTER_SCRATCH);
        if (scratch != SCRATCH_VALUE)
        {
            ESP_LOGW(TAG, "AVR reset detected");
            datastore_set_string(task_inputs->datastore, RESOURCE_ID_SYSTEM_LOG, 0, "AVR reset");
            _write_register(smbus_info, AVR_REGISTER_SCRATCH, SCRATCH_VALUE);
            datastore_increment(task_inputs->datastore, RESOURCE_ID_AVR_COUNT_RESET, 0);
        }

        // read and increment counters
        uint8_t count_cp = _read_register(smbus_info, AVR_REGISTER_COUNT_CP);
        datastore_add(task_inputs->datastore, RESOURCE_ID_AVR_COUNT_CP, 0, count_cp);

        uint8_t count_pp = _read_register(smbus_info, AVR_REGISTER_COUNT_PP);
        datastore_add(task_inputs->datastore, RESOURCE_ID_AVR_COUNT_PP, 0, count_pp);

        uint8_t count_buzzer = _read_register(smbus_info, AVR_REGISTER_COUNT_BUZZER);
        datastore_add(task_inputs->datastore, RESOURCE_ID_AVR_COUNT_BUZZER, 0, count_buzzer);

        uint8_t count_sw1 = _read_register(smbus_info, AVR_REGISTER_COUNT_CP_MODE);
        datastore_add(task_inputs->datastore, RESOURCE_ID_AVR_COUNT_CP_MODE, 0, count_sw1);

        uint8_t count_sw2 = _read_register(smbus_info, AVR_REGISTER_COUNT_CP_MAN);
        datastore_add(task_inputs->datastore, RESOURCE_ID_AVR_COUNT_CP_MAN, 0, count_sw2);

        uint8_t count_sw3 = _read_register(smbus_info, AVR_REGISTER_COUNT_PP_MODE);
        datastore_add(task_inputs->datastore, RESOURCE_ID_AVR_COUNT_PP_MODE, 0, count_sw3);

        uint8_t count_sw4 = _read_register(smbus_info, AVR_REGISTER_COUNT_PP_MAN);
        datastore_add(task_inputs->datastore, RESOURCE_ID_AVR_COUNT_PP_MAN, 0, count_sw4);

        i2c_master_unlock(i2c_master_info);

        // if any switches have changed state, publish them
        uint8_t new_switch_states = _decode_switch_states(status);
        _publish_switch_changes(switch_states, new_switch_states, task_inputs->datastore);
        switch_states = new_switch_states;

        // if any pumps have changed state, publish them
        uint8_t new_pump_states = _decode_pump_states(status);
        _publish_pump_changes(pump_states, new_pump_states, task_inputs->datastore);
        pump_states = new_pump_states;

        vTaskDelayUntil(&last_wake_time, TICKS_PER_UPDATE);
    }

  stop: ;
    i2c_master_unlock(i2c_master_info);
    free(task_inputs);
    _task_handle = NULL;
    vTaskDelete(NULL);
}

void avr_support_init(i2c_master_info_t * i2c_master_info, UBaseType_t priority, const datastore_t * datastore)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    command_queue = xQueueCreate(10, sizeof(command_t));

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->i2c_master_info = i2c_master_info;
        task_inputs->datastore = datastore;
        xTaskCreate(&avr_support_task, "avr_support_task", 4096, task_inputs, priority, &_task_handle);
    }

    // set up the AVR reset line
    // and make sure it is not held in reset
    gpio_pad_select_gpio(CONFIG_AVR_RESET_GPIO);
    gpio_set_level(CONFIG_AVR_RESET_GPIO, 1);
    gpio_set_direction(CONFIG_AVR_RESET_GPIO, GPIO_MODE_OUTPUT);
}

void avr_support_delete(void)
{
    if (_task_handle)
        vTaskDelete(_task_handle);
}

void avr_support_reset(void)
{
    ESP_LOGD(TAG, "request AVR reset");

    command_t cmd = COMMAND_OFF | COMMAND_SELECT_CP | COMMAND_SELECT_PP | COMMAND_SELECT_ALARM | COMMAND_AVR_RESET;
    if (xQueueSendToBack(command_queue, &cmd, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "xQueueSendToBack failed");
    }
}

void avr_support_set_cp_pump(avr_pump_state_t state)
{
    ESP_LOGD(TAG, "request set CP pump %s", state == AVR_PUMP_STATE_ON ? "on" : "off");
    command_t cmd = (state == AVR_PUMP_STATE_ON ? COMMAND_ON : COMMAND_OFF) | COMMAND_SELECT_CP;
    if (xQueueSendToBack(command_queue, &cmd, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "xQueueSendToBack failed");
    }
}

void avr_support_set_pp_pump(avr_pump_state_t state)
{
    ESP_LOGD(TAG, "request set PP pump %s", state == AVR_PUMP_STATE_ON ? "on" : "off");
    command_t cmd = (state == AVR_PUMP_STATE_ON ? COMMAND_ON : COMMAND_OFF) | COMMAND_SELECT_PP;
    if (xQueueSendToBack(command_queue, &cmd, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "xQueueSendToBack failed");
    }
}

void avr_support_set_alarm(avr_alarm_state_t state)
{
    ESP_LOGD(TAG, "request set alarm %s", state == AVR_ALARM_STATE_ON ? "on" : "off");
    command_t cmd = (state == AVR_ALARM_STATE_ON ? COMMAND_ON : COMMAND_OFF) | COMMAND_SELECT_ALARM;
    if (xQueueSendToBack(command_queue, &cmd, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "xQueueSendToBack failed");
    }
}
