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
    PAGE_IGNORE = -1,        // when used in a transition, ignore the rule
    PAGE_BLANK = 0,
    PAGE_SPLASH ,
    PAGE_SENSORS_TEMP_1_2,
    PAGE_SENSORS_TEMP_3_4,
    PAGE_SENSORS_TEMP_5_P,   // Temp5 & Power
    PAGE_SENSORS_LIGHT,
    PAGE_SENSORS_FLOW,
    PAGE_PUMPS_SSRS,
    PAGE_ALARM,
    PAGE_ADVANCED,
    PAGE_LAST_ERROR,
    PAGE_WIFI_STATUS,
    PAGE_MQTT_STATUS,
    PAGE_SENSORS_STATUS_1,
    PAGE_SENSORS_STATUS_2,
    PAGE_SENSORS_STATUS_3,
    PAGE_SENSORS_STATUS_4,
    PAGE_ESP32_STATUS,
    PAGE_AVR_STATUS,
    PAGE_LAST,
} page_id_t;

typedef enum
{
    INPUT_NONE,
    INPUT_SHORT,   // single button press less than 100ms
    INPUT_LONG,    // single button press less than 500ms
} input_t;

typedef void (*page_handler_t)(const i2c_lcd1602_info_t * lcd_info, void * state);

typedef struct
{
    page_id_t id;
    page_handler_t handler;
    void * state;
} page_spec_t;

// page handlers are responsible for displaying their content
static void _handle_page_blank(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_splash(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_sensors_temp_1_2(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_sensors_temp_3_4(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_sensors_temp_5_P(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_sensors_light(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_sensors_flow(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_pump_ssrs(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_alarm(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_advanced(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_last_error(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_wifi_status(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_mqtt_status(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_sensors_status_1(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_sensors_status_2(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_sensors_status_3(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_sensors_status_4(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_esp32_status(const i2c_lcd1602_info_t * lcd_info, void * state);
static void _handle_page_avr_status(const i2c_lcd1602_info_t * lcd_info, void * state);

static int alarm_display_count = 0;

static const page_spec_t page_specs[] = {
    // ID                       handler                        state
    { PAGE_BLANK,               _handle_page_blank,            NULL },
    { PAGE_SPLASH,              _handle_page_splash,           NULL },
    { PAGE_SENSORS_TEMP_1_2,    _handle_page_sensors_temp_1_2, NULL },
    { PAGE_SENSORS_TEMP_3_4,    _handle_page_sensors_temp_3_4, NULL },
    { PAGE_SENSORS_TEMP_5_P,    _handle_page_sensors_temp_5_P, NULL },
    { PAGE_SENSORS_LIGHT,       _handle_page_sensors_light,    NULL },
    { PAGE_SENSORS_FLOW,        _handle_page_sensors_flow,     NULL },
    { PAGE_PUMPS_SSRS,          _handle_page_pump_ssrs,        NULL },
    { PAGE_ALARM,               _handle_page_alarm,            &alarm_display_count },
    { PAGE_ADVANCED,            _handle_page_advanced,         NULL },
    { PAGE_LAST_ERROR,          _handle_page_last_error,       NULL },
    { PAGE_WIFI_STATUS,         _handle_page_wifi_status,      NULL },
    { PAGE_MQTT_STATUS,         _handle_page_mqtt_status,      NULL },
    { PAGE_SENSORS_STATUS_1,    _handle_page_sensors_status_1, NULL },
    { PAGE_SENSORS_STATUS_2,    _handle_page_sensors_status_2, NULL },
    { PAGE_SENSORS_STATUS_3,    _handle_page_sensors_status_3, NULL },
    { PAGE_SENSORS_STATUS_4,    _handle_page_sensors_status_4, NULL },
    { PAGE_ESP32_STATUS,        _handle_page_esp32_status,     NULL },
    { PAGE_AVR_STATUS,          _handle_page_avr_status,       NULL },
};

// any press longer than this is considered "long"
#define SHORT_PRESS_THRESHOLD 40  // ticks

// page transition table
typedef struct
{
    page_id_t current;
    page_id_t on_short;    // new page on single short press
    page_id_t on_long;     // new page on single long press
} transition_t;

static const transition_t transitions[] = {
    // ID                       short                  long
    { PAGE_BLANK,               PAGE_IGNORE,           PAGE_IGNORE },
    { PAGE_SPLASH,              PAGE_SENSORS_TEMP_1_2, PAGE_IGNORE },
    { PAGE_SENSORS_TEMP_1_2,    PAGE_SENSORS_TEMP_3_4, PAGE_IGNORE },
    { PAGE_SENSORS_TEMP_3_4,    PAGE_SENSORS_TEMP_5_P, PAGE_IGNORE },
    { PAGE_SENSORS_TEMP_5_P,    PAGE_SENSORS_LIGHT,    PAGE_IGNORE },
    { PAGE_SENSORS_LIGHT,       PAGE_SENSORS_FLOW,     PAGE_IGNORE },
    { PAGE_SENSORS_FLOW,        PAGE_PUMPS_SSRS,       PAGE_IGNORE },
    { PAGE_PUMPS_SSRS,          PAGE_ALARM,            PAGE_IGNORE },
    { PAGE_ALARM,               PAGE_ADVANCED,         PAGE_IGNORE },
    { PAGE_ADVANCED,            PAGE_SENSORS_TEMP_1_2, PAGE_LAST_ERROR },
    { PAGE_LAST_ERROR,          PAGE_WIFI_STATUS,      PAGE_SENSORS_TEMP_1_2 },
    { PAGE_WIFI_STATUS,         PAGE_MQTT_STATUS,      PAGE_SENSORS_TEMP_1_2 },
    { PAGE_MQTT_STATUS,         PAGE_SENSORS_STATUS_1, PAGE_SENSORS_TEMP_1_2 },
    { PAGE_SENSORS_STATUS_1,    PAGE_SENSORS_STATUS_2, PAGE_SENSORS_TEMP_1_2 },
    { PAGE_SENSORS_STATUS_2,    PAGE_SENSORS_STATUS_3, PAGE_SENSORS_TEMP_1_2 },
    { PAGE_SENSORS_STATUS_3,    PAGE_SENSORS_STATUS_4, PAGE_SENSORS_TEMP_1_2 },
    { PAGE_SENSORS_STATUS_4,    PAGE_ESP32_STATUS,     PAGE_SENSORS_TEMP_1_2 },
    { PAGE_ESP32_STATUS,        PAGE_AVR_STATUS,       PAGE_SENSORS_TEMP_1_2 },
    { PAGE_AVR_STATUS,          PAGE_LAST_ERROR,       PAGE_SENSORS_TEMP_1_2 },
};

static QueueHandle_t button_queue;

typedef struct
{
    i2c_master_info_t * i2c_master_info;
} task_inputs_t;

static const uint8_t degrees_C[8]  = { 0x10, 0x06, 0x09, 0x08, 0x08, 0x09, 0x06, 0x00 };

static void _handle_page_blank(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "BLANK"));
}

static void _handle_page_splash(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    char line[ROW_STRING_WIDTH] = "";
    snprintf(line, ROW_STRING_WIDTH, "PoolControl v%d.%d", VERSION_MAJOR, VERSION_MINOR);
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line));

    snprintf(line, ROW_STRING_WIDTH, " %s", BUILD_TIMESTAMP);
    ESP_ERROR_CHECK(i2c_lcd1602_move_cursor(lcd_info, 0, 1));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line));
}

static void _handle_page_sensors_temp_1_2(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    char line[ROW_STRING_WIDTH] = "";
    snprintf(line, ROW_STRING_WIDTH, "Temp%c  %2.1f %2.1f", I2C_LCD1602_CHARACTER_CUSTOM_0, 18.1, 20.2);
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line));

    snprintf(line, ROW_STRING_WIDTH, "  %2.1f %2.1f %2.1f", 22.9, 21.8, 21.7);
    ESP_ERROR_CHECK(i2c_lcd1602_move_cursor(lcd_info, 0, 1));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line));
 }

static void _handle_page_sensors_temp_3_4(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "SENSORS_TEMP_3_4"));
}

static void _handle_page_sensors_temp_5_P(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "SENSORS_TEMP_5_P"));
}

static void _handle_page_sensors_light(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "SENSORS_LIGHT"));
}

static void _handle_page_sensors_flow(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "SENSORS_FLOW"));
}

static void _handle_page_pump_ssrs(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "PUMPS_SSRS"));
}

static void _handle_page_alarm(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    int * display_count = (int *)state;
    ++*display_count;
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    char line[ROW_STRING_WIDTH] = "";
    snprintf(line, ROW_STRING_WIDTH, "ALARM %d", *display_count);
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, line));
}

static void _handle_page_advanced(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "ADVANCED"));
}

static void _handle_page_last_error(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "LAST_ERROR"));
}

static void _handle_page_wifi_status(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "WIFI_STATUS"));
}

static void _handle_page_mqtt_status(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "MQTT_STATUS"));
}

static void _handle_page_sensors_status_1(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "SENSORS_STATUS_1"));
}

static void _handle_page_sensors_status_2(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "SENSORS_STATUS_2"));
}

static void _handle_page_sensors_status_3(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "SENSORS_STATUS_3"));
}

static void _handle_page_sensors_status_4(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "SENSORS_STATUS_4"));
}

static void _handle_page_esp32_status(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "ESP32_STATUS"));
}

static void _handle_page_avr_status(const i2c_lcd1602_info_t * lcd_info, void * state)
{
    ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
    ESP_ERROR_CHECK(i2c_lcd1602_write_string(lcd_info, "AVR_STATUS"));
}

static void dispatch_to_handler(i2c_lcd1602_info_t * lcd_info, page_id_t current_page)
{
    assert(sizeof(page_specs) / sizeof(page_specs[0]) == PAGE_LAST);

    if (current_page >= 0 && current_page < PAGE_LAST)
    {
        if (page_specs[current_page].id == current_page)
        {
            if (page_specs[current_page].handler)
            {
                page_specs[current_page].handler(lcd_info, page_specs[current_page].state);
            }
            else
            {
                ESP_LOGE(TAG, "page %d has no handler", current_page);
                current_page = PAGE_BLANK;
            }
        }
        else
        {
            ESP_LOGE(TAG, "page spec mismatch at position %d", current_page);
            current_page = PAGE_BLANK;
        }
    }
    else
    {
        ESP_LOGE(TAG, "current page %d out of range", current_page);
        current_page = PAGE_BLANK;
    }
}

static page_id_t handle_transition(input_t input, page_id_t current_page)
{
    page_id_t new_page = PAGE_BLANK;
    if (current_page >= 0 && current_page < PAGE_LAST)
    {
        switch (input)
        {
            case INPUT_NONE:
                new_page = current_page;
                break;
            case INPUT_SHORT:
                new_page = transitions[current_page].on_short;
                break;
            case INPUT_LONG:
                new_page = transitions[current_page].on_long;
                break;
            default:
                ESP_LOGE(TAG, "invalid input %d", input);
                break;
        }
    }
    return new_page;
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
    i2c_lcd1602_write_char(lcd_info, 'B');

    // Define custom characters
    i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_0, degrees_C);

    // Move to home position
    ESP_ERROR_CHECK(i2c_lcd1602_move_cursor(lcd_info, 0, 0));

    page_id_t current_page = PAGE_SPLASH;

    // update pages once per second
    while (1)
    {
        ESP_LOGI(TAG, "display loop");
        dispatch_to_handler(lcd_info, current_page);
        input_t input = INPUT_NONE;
        BaseType_t rc = xQueueReceive(button_queue, &input, 1000 / portTICK_RATE_MS);
        if (rc == pdTRUE)
        {
            ESP_LOGI(TAG, "from queue: %d", input);
            page_id_t new_page = handle_transition(input, current_page);
            if (new_page != current_page && new_page >= 0 && new_page < PAGE_LAST)
            {
                ESP_LOGI(TAG, "change to page %d", new_page);
                current_page = new_page;
                ESP_ERROR_CHECK(i2c_lcd1602_clear(lcd_info));
            }
        }
    }

    free(task_inputs);
    vTaskDelete(NULL);
}

static void button_task(void * pvParameter)
{
    // TODO: improve this, it's too naive

    // Configure button
    gpio_config_t btn_config;
    btn_config.intr_type = GPIO_INTR_ANYEDGE;    // Enable interrupt on both rising and falling edges
    btn_config.mode = GPIO_MODE_INPUT;           // Set as Input
    btn_config.pin_bit_mask = (1 << CONFIG_DISPLAY_BUTTON_GPIO); // Bitmask
    btn_config.pull_up_en = GPIO_PULLUP_ENABLE;      // Disable pullup
    btn_config.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pulldown
    gpio_config(&btn_config);

    bool last_raw_state = false;
    bool debounced_state = false;

    TickType_t last_pressed = xTaskGetTickCount();
    TickType_t last_released = last_pressed;

    while (1)
    {
        // debounce first
        bool raw_state = gpio_get_level(CONFIG_DISPLAY_BUTTON_GPIO) ? false : true;
        if (raw_state == last_raw_state)
        {
            // detect edges
            if (raw_state != debounced_state)
            {
                debounced_state = raw_state;
                TickType_t now = xTaskGetTickCount();

                // if a falling edge, measure time since rising edge
                input_t input = INPUT_NONE;
                if (!debounced_state)
                {
                    ESP_LOGI(TAG, "pressed for %d ticks", now - last_pressed);
                    if (now - last_pressed < SHORT_PRESS_THRESHOLD)
                    {
                        input = INPUT_SHORT;
                        ESP_LOGI(TAG, "short");
                    }
                    else
                    {
                        input = INPUT_LONG;
                        ESP_LOGI(TAG, "long");
                    }
                    last_released = now;
                }
                else
                {
                    last_pressed = now;
                }

                if (input != INPUT_NONE)
                {
                    if (xQueueSendToBack(button_queue, &input, 0) != pdTRUE)
                    {
                        ESP_LOGE(TAG, "xQueueSendToBack failed");
                    }
                }

            }
        }
        last_raw_state = raw_state;

        //vTaskDelay(100 / portTICK_RATE_MS);
        vTaskDelay(5);
    }

    vTaskDelete(NULL);
}

void display_init(i2c_master_info_t * i2c_master_info, UBaseType_t priority)
{
    button_queue = xQueueCreate(10, sizeof(input_t));

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->i2c_master_info = i2c_master_info;
        xTaskCreate(&display_task, "display_task", 4096, task_inputs, priority, NULL);
    }

    xTaskCreate(&button_task, "button_task", 2048, NULL, priority, NULL);
}