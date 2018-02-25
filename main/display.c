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
#include "esp_wifi.h"

#include "display.h"
#include "constants.h"
#include "resources.h"
#include "utils.h"
#include "i2c_master.h"
#include "smbus.h"
#include "i2c-lcd1602.h"
#include "avr_support.h"
#include "sensor_temp.h"
#include "convert_string.h"
#include "wifi_support.h"
#include "mqtt.h"
#include "datastore/datastore.h"

#define TAG "display"

#define SMBUS_TIMEOUT     1000   // milliseconds
#define DISPLAY_WIDTH     I2C_LCD1602_NUM_VISIBLE_COLUMNS
#define ROW_STRING_WIDTH  (DISPLAY_WIDTH + 1)    // room for null terminator
#define TICKS_PER_UPDATE  (500 / portTICK_RATE_MS)
#define TICKS_PER_POLL    (5)    // ticks per button poll

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

//#define INITIAL_PAGE PAGE_SPLASH
#define INITIAL_PAGE PAGE_MQTT_STATUS

typedef enum
{
    INPUT_NONE,
    INPUT_SHORT,   // single button press less than 100ms
    INPUT_LONG,    // single button press less than 500ms
} input_t;

typedef void (*page_handler_t)(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);

typedef struct
{
    page_id_t id;
    page_handler_t handler;
    void * state;
} page_spec_t;

// page handlers are responsible for displaying their content
static void _handle_page_blank(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_splash(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_sensors_temp_1_2(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_sensors_temp_3_4(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_sensors_temp_5_P(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_sensors_light(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_sensors_flow(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_pump_ssrs(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_alarm(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_advanced(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_last_error(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_wifi_status(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_mqtt_status(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_sensors_status_1(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_sensors_status_2(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_sensors_status_3(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_sensors_status_4(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_esp32_status(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);
static void _handle_page_avr_status(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore);

static int alarm_display_count = 0;
static bool splash_activity = false;
static int esp32_status_state = 0;
static int mqtt_status_state = 0;

static const page_spec_t page_specs[] = {
    // ID                       handler                        state
    { PAGE_BLANK,               _handle_page_blank,            NULL },
    { PAGE_SPLASH,              _handle_page_splash,           &splash_activity },
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
    { PAGE_MQTT_STATUS,         _handle_page_mqtt_status,      &mqtt_status_state },
    { PAGE_SENSORS_STATUS_1,    _handle_page_sensors_status_1, NULL },
    { PAGE_SENSORS_STATUS_2,    _handle_page_sensors_status_2, NULL },
    { PAGE_SENSORS_STATUS_3,    _handle_page_sensors_status_3, NULL },
    { PAGE_SENSORS_STATUS_4,    _handle_page_sensors_status_4, NULL },
    { PAGE_ESP32_STATUS,        _handle_page_esp32_status,     &esp32_status_state },
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
    { PAGE_BLANK,               PAGE_IGNORE,           PAGE_LAST_ERROR /*PAGE_IGNORE*/ },
    { PAGE_SPLASH,              PAGE_SENSORS_TEMP_1_2, PAGE_LAST_ERROR /*PAGE_IGNORE*/ },
    { PAGE_SENSORS_TEMP_1_2,    PAGE_SENSORS_TEMP_3_4, PAGE_LAST_ERROR /*PAGE_IGNORE*/ },
    { PAGE_SENSORS_TEMP_3_4,    PAGE_SENSORS_TEMP_5_P, PAGE_LAST_ERROR /*PAGE_IGNORE*/ },
    { PAGE_SENSORS_TEMP_5_P,    PAGE_SENSORS_LIGHT,    PAGE_LAST_ERROR /*PAGE_IGNORE*/ },
    { PAGE_SENSORS_LIGHT,       PAGE_SENSORS_FLOW,     PAGE_LAST_ERROR /*PAGE_IGNORE*/ },
    { PAGE_SENSORS_FLOW,        PAGE_PUMPS_SSRS,       PAGE_LAST_ERROR /*PAGE_IGNORE*/ },
    { PAGE_PUMPS_SSRS,          PAGE_ALARM,            PAGE_LAST_ERROR /*PAGE_IGNORE*/ },
    { PAGE_ALARM,               PAGE_ADVANCED,         PAGE_LAST_ERROR /*PAGE_IGNORE*/ },
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

static const char * BLANK_LINE = "                ";
#define MEASUREMENT_EXPIRY 15  // seconds after which a measurement is not displayed

typedef struct
{
    i2c_master_info_t * i2c_master_info;
    const datastore_t * datastore;
} task_inputs_t;

static const uint8_t degrees_C[8]  = { 0x10, 0x06, 0x09, 0x08, 0x08, 0x09, 0x06, 0x00 };
#define DEGREES_C "\x8"

static esp_err_t _display_reset(const i2c_lcd1602_info_t * lcd_info)
{
    ESP_LOGI(TAG, "display reset");
    esp_err_t err = i2c_lcd1602_reset(lcd_info);
    if (err == ESP_OK)
    {
        // Define custom characters
        err = i2c_lcd1602_define_char(lcd_info, I2C_LCD1602_INDEX_CUSTOM_0, degrees_C);
        I2C_LCD1602_ERROR_CHECK(err);
    }
    return err;
}

// display wrappers to reset and reinitialise display on any I2C error
static esp_err_t _clear(const i2c_lcd1602_info_t * lcd_info)
{
    esp_err_t err = ESP_FAIL;
    int count = 0;
    while (count < 10 && (err = i2c_lcd1602_clear(lcd_info)) != ESP_OK)
    {
        ++count;
        vTaskDelay(10 / portTICK_RATE_MS);
        _display_reset(lcd_info);
        ESP_LOGW(TAG, "retry _clear %d", count);
    }
    return err;
}

static esp_err_t _move_cursor(const i2c_lcd1602_info_t * lcd_info, uint8_t col, uint8_t row)
{
    esp_err_t err = ESP_FAIL;
    int count = 0;
    while (count < 10 && (err = i2c_lcd1602_move_cursor(lcd_info, col, row)) != ESP_OK)
    {
        ++count;
        vTaskDelay(10 / portTICK_RATE_MS);
        _display_reset(lcd_info);
        ESP_LOGW(TAG, "retry _move_cursor %d", count);
    }
    return err;
}

static esp_err_t _write_string(const i2c_lcd1602_info_t * lcd_info, const char * string)
{
    esp_err_t err = ESP_FAIL;
    int count = 0;
    while (count < 10 && (err = i2c_lcd1602_write_string(lcd_info, string)) != ESP_OK)
    {
        ++count;
        vTaskDelay(10 / portTICK_RATE_MS);
        _display_reset(lcd_info);
        ESP_LOGW(TAG, "retry _write_string %d", count);
    }
    return err;
}


static void _handle_page_blank(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    I2C_LCD1602_ERROR_CHECK(_clear(lcd_info));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "BLANK"));
}

static void _handle_page_splash(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    bool * activity = (bool *)state;

    char version[SYSTEM_LEN_VERSION] = "";
    char build_date_time[SYSTEM_LEN_BUILD_DATE_TIME] = "";
    datastore_get_string(datastore, RESOURCE_ID_SYSTEM_VERSION, 0, version);
    datastore_get_string(datastore, RESOURCE_ID_SYSTEM_BUILD_DATE_TIME, 0, build_date_time);

    char line[ROW_STRING_WIDTH] = "";
    snprintf(line, ROW_STRING_WIDTH, "PoolControl v%s", version);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

    snprintf(line, ROW_STRING_WIDTH, "%c%s", *activity ? I2C_LCD1602_CHARACTER_DOT : ' ', build_date_time);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

    *activity = !(*activity);
}

static void _get_temp_sensor(const datastore_t * store, datastore_instance_id_t instance, float * value, char * label, uint32_t * timestamp, const datastore_t * datastore)
{
    assert(value);
    assert(label);
    assert(timestamp);
    *value = 0.0f;
    label[0] = '\0';
    *timestamp = 0;
    datastore_get_float(datastore, RESOURCE_ID_TEMP_VALUE, instance, value);
    datastore_get_string(datastore, RESOURCE_ID_TEMP_LABEL, instance, label);
    datastore_get_uint32(datastore, RESOURCE_ID_TEMP_TIMESTAMP, instance, timestamp);
}

static void _render_temp_line(char * line, unsigned int len, datastore_instance_id_t instance, uint32_t now, const datastore_t * datastore)
{
    float value = 0.0f;
    char label[SENSOR_TEMP_LEN_LABEL] = "";
    uint32_t timestamp = 0;

    _get_temp_sensor(datastore, instance, &value, label, &timestamp, datastore);
    if (now - timestamp < MEASUREMENT_EXPIRY)
    {
        snprintf(line, ROW_STRING_WIDTH, "T%d %-7s %4.1f"DEGREES_C, instance + 1, label, value);
    }
    else
    {
        snprintf(line, ROW_STRING_WIDTH, "T%d %-7s --.-"DEGREES_C, instance + 1, label);
    }
}

static void _handle_page_sensors_temp_1_2(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    char line[ROW_STRING_WIDTH] = "";
    uint32_t now = seconds_since_boot();

    _render_temp_line(line, ROW_STRING_WIDTH, 0, now, datastore);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

    _render_temp_line(line, ROW_STRING_WIDTH, 1, now, datastore);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
 }

static void _handle_page_sensors_temp_3_4(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    char line[ROW_STRING_WIDTH] = "";
    uint32_t now = seconds_since_boot();

    _render_temp_line(line, ROW_STRING_WIDTH, 2, now, datastore);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

    _render_temp_line(line, ROW_STRING_WIDTH, 3, now, datastore);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
}

static void _handle_page_sensors_temp_5_P(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    char line[ROW_STRING_WIDTH] = "";
    uint32_t now = seconds_since_boot();

    _render_temp_line(line, ROW_STRING_WIDTH, 4, now, datastore);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

    float power = 0.0f;
    uint32_t timestamp = 0;
    datastore_get_uint32(datastore, RESOURCE_ID_POWER_TIMESTAMP, 0, &timestamp);
    if (now - timestamp < MEASUREMENT_EXPIRY)
    {
        datastore_get_float(datastore, RESOURCE_ID_POWER_VALUE, 0, &power);
        snprintf(line, ROW_STRING_WIDTH, "Power   %7.1fW", power);
    }
    else
    {
        snprintf(line, ROW_STRING_WIDTH, "Power      --.-W");
    }
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
}

static void _handle_page_sensors_light(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    bool detected = false;
    uint32_t full = 0, visible = 0, infrared = 0, illuminance = 0;
    datastore_get_bool(datastore, RESOURCE_ID_LIGHT_DETECTED, 0, &detected);

    if (detected)
    {
        uint32_t timestamp = 0;
        datastore_get_uint32(datastore, RESOURCE_ID_LIGHT_TIMESTAMP, 0, &timestamp);

        if (seconds_since_boot() - timestamp < MEASUREMENT_EXPIRY)
        {
            datastore_get_uint32(datastore, RESOURCE_ID_LIGHT_FULL, 0, &full);
            datastore_get_uint32(datastore, RESOURCE_ID_LIGHT_VISIBLE, 0, &visible);
            datastore_get_uint32(datastore, RESOURCE_ID_LIGHT_INFRARED, 0, &infrared);
            datastore_get_uint32(datastore, RESOURCE_ID_LIGHT_ILLUMINANCE, 0, &illuminance);

            char line[ROW_STRING_WIDTH] = "";
            snprintf(line, ROW_STRING_WIDTH, "Li F%5d L%5d", full, illuminance);
            I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
            I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

            snprintf(line, ROW_STRING_WIDTH, "   I%5d V%5d", infrared, visible);
            I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
            I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
        }
        else
        {
            // measurement timed out
            I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
            I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "Li F ---- L ----"));
            I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
            I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "   I ---- V ----"));
        }
    }
    else
    {
        // sensor not detected at boot
        I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
        I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "Li F ???? L ????"));
        I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
        I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "   I ???? V ????"));
    }
}

static void _handle_page_sensors_flow(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    float frequency = 0.0f, rate = 0.0f;
    datastore_get_float(datastore, RESOURCE_ID_FLOW_FREQUENCY, 0, &frequency);
    datastore_get_float(datastore, RESOURCE_ID_FLOW_RATE, 0, &rate);

    char line[ROW_STRING_WIDTH] = "";
    snprintf(line, ROW_STRING_WIDTH, "Flow   %5.1f Hz ", frequency);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

    snprintf(line, ROW_STRING_WIDTH, "       %5.1f LPM", rate);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
}

static void _build_switch_string(char * string, uint8_t len, datastore_resource_id_t mode, datastore_resource_id_t man, const datastore_t * datastore)
{
    avr_switch_mode_t mode_value = 0;
    datastore_get_uint32(datastore, mode, 0, &mode_value);

    avr_switch_manual_t manual_value = 0;
    datastore_get_uint32(datastore, man, 0, &manual_value);

    // TODO: when in AUTO mode, the ON/OFF should be the currently
    // requested mode, not the Manual switch position. This allows the user
    // to compare the requested mode with the actual pump state.

    if (mode_value == AVR_SWITCH_MODE_MANUAL)
    {
        snprintf(string, len, "M:%s", manual_value == AVR_SWITCH_MANUAL_OFF ? "OFF" : "ON");
    }
    else
    {
//        avr_pump_state_t requested_state = 0;
//        datastore_get_uint32(datastore, state, 0, &requested_state);
//        snprintf(string, len, "A:%s", requested_state == AVR_PUMP_STATE_OFF ? "OFF" : "ON");
        snprintf(string, len, "A:---");
    }
}

static void _build_pump_string(char * string, uint8_t len, datastore_resource_id_t id, const datastore_t * datastore)
{
    avr_pump_state_t state = 0;
    datastore_get_uint32(datastore, id, 0, &state);

    snprintf(string, len, "%-3s", state == AVR_PUMP_STATE_OFF ? "OFF" : "ON");
}

static void _handle_page_pump_ssrs(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    uint32_t switches_timestamp = 0;
    datastore_get_uint32(datastore, RESOURCE_ID_SWITCHES_TIMESTAMP, 0, &switches_timestamp);

    static const uint8_t MODE_LEN = 6;
    char cp_switches[MODE_LEN];
    char pp_switches[MODE_LEN];

    _build_switch_string(cp_switches, MODE_LEN, RESOURCE_ID_SWITCHES_CP_MODE_VALUE, RESOURCE_ID_SWITCHES_CP_MAN_VALUE, datastore);
    _build_switch_string(pp_switches, MODE_LEN, RESOURCE_ID_SWITCHES_PP_MODE_VALUE, RESOURCE_ID_SWITCHES_PP_MAN_VALUE, datastore);

    static const uint8_t PUMP_LEN = 4;
    char cp_pump[PUMP_LEN];
    char pp_pump[PUMP_LEN];

    _build_pump_string(cp_pump, PUMP_LEN, RESOURCE_ID_PUMPS_CP_STATE, datastore);
    _build_pump_string(pp_pump, PUMP_LEN, RESOURCE_ID_PUMPS_PP_STATE, datastore);

    char line[ROW_STRING_WIDTH] = "";
    snprintf(line, ROW_STRING_WIDTH, "Pu CP %-5s  %-3s", cp_switches, cp_pump);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

    snprintf(line, ROW_STRING_WIDTH, "   PP %-5s  %-3s", pp_switches, pp_pump);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

}

static void _handle_page_alarm(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    int * display_count = (int *)state;
    ++*display_count;
    char line[ROW_STRING_WIDTH] = "";
    snprintf(line, ROW_STRING_WIDTH, "ALARM %-6d    ", *display_count);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "ABCDEFGHIJKLMNOP"));
}

static void _handle_page_advanced(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    I2C_LCD1602_ERROR_CHECK(_clear(lcd_info));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "ADVANCED"));
}

static void _handle_page_last_error(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    I2C_LCD1602_ERROR_CHECK(_clear(lcd_info));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "LAST_ERROR"));
}

static void _handle_page_wifi_status(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    wifi_status_t wifi_status = 0;
    datastore_get_uint32(datastore, RESOURCE_ID_WIFI_STATUS, 0, &wifi_status);

    char line[ROW_STRING_WIDTH] = "";

    char ssid[WIFI_LEN_SSID] = "";
    int8_t rssi = 0;

    datastore_get_string(datastore, RESOURCE_ID_WIFI_SSID, 0, ssid);
    datastore_get_int8(datastore, RESOURCE_ID_WIFI_RSSI, 0, &rssi);

    // truncate ssid at 7 characters
    ssid[8] = '\0';

    switch (wifi_status)
    {
        case WIFI_STATUS_DISCONNECTED:
            snprintf(line, ROW_STRING_WIDTH, "WiFi %-11s", "disconnect");
            break;
        case WIFI_STATUS_CONNECTED:
        case WIFI_STATUS_GOT_ADDRESS:
            snprintf(line, ROW_STRING_WIDTH, "WiFi %-7s %3d", ssid, rssi);
            break;
        default:
            ESP_LOGE(TAG, "unhandled wifi status %d", wifi_status);
            break;
    }

    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

    switch (wifi_status)
    {
        case WIFI_STATUS_DISCONNECTED:
            snprintf(line, ROW_STRING_WIDTH, BLANK_LINE);
            break;
        case WIFI_STATUS_CONNECTED:
            snprintf(line, ROW_STRING_WIDTH, "  waiting for IP");
            break;
        case WIFI_STATUS_GOT_ADDRESS:
        {
            uint32_t ip_address = 0;
            datastore_get_uint32(datastore, RESOURCE_ID_WIFI_ADDRESS, 0, &ip_address);
            snprintf(line, ROW_STRING_WIDTH, "%d.%d.%d.%d        ",
                     (ip_address & 0xff),
                     (ip_address & 0xff00) >> 8,
                     (ip_address & 0xff0000) >> 16,
                     (ip_address & 0xff000000) >> 24);
            break;
        }
        default:
            ESP_LOGE(TAG, "unhandled wifi status %d", wifi_status);
            break;
    }
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
}

static void _handle_page_mqtt_status_1(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    mqtt_status_t mqtt_status = 0;
    datastore_get_uint32(datastore, RESOURCE_ID_MQTT_STATUS, 0, &mqtt_status);

    char line[ROW_STRING_WIDTH] = "";

    switch (mqtt_status)
    {
        case MQTT_STATUS_DISCONNECTED:
            snprintf(line, ROW_STRING_WIDTH, "MQTT disconnect ");
            break;
        case MQTT_STATUS_CONNECTING:
            snprintf(line, ROW_STRING_WIDTH, "MQTT connecting ");
            break;
        case MQTT_STATUS_CONNECTED:
        {
            uint32_t connection_count = 0;
            datastore_get_uint32(datastore, RESOURCE_ID_MQTT_CONNECTION_COUNT, 0, &connection_count);
            snprintf(line, ROW_STRING_WIDTH, "MQTT connect %-3d", connection_count);
            break;
        }
        default:
            ESP_LOGE(TAG, "unhandled mqtt status %d", mqtt_status);
            break;
    }

    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

    uint32_t count_rx = 0, count_tx = 0;
    datastore_get_uint32(datastore, RESOURCE_ID_MQTT_MESSAGE_RX_COUNT, 0, &count_rx);
    datastore_get_uint32(datastore, RESOURCE_ID_MQTT_MESSAGE_TX_COUNT, 0, &count_tx);

    snprintf(line, ROW_STRING_WIDTH, "RX %-4d TX %-4d ", count_rx, count_tx);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
}

static void _handle_page_mqtt_status_2(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    char broker_address[MQTT_LEN_BROKER_ADDRESS] = "";
    uint32_t broker_port = 0;
    datastore_get_string(datastore, RESOURCE_ID_MQTT_BROKER_ADDRESS, 0, broker_address);
    datastore_get_uint32(datastore, RESOURCE_ID_MQTT_BROKER_PORT, 0, &broker_port);

    char line[ROW_STRING_WIDTH] = "";

    char port[6] = "";
    snprintf(port, 6, "%d", broker_port);
    int port_len = strlen(port);
    int addr_len = DISPLAY_WIDTH - port_len - 1;  // space for the colon
    snprintf(line, ROW_STRING_WIDTH, "%.*s:%-*d", addr_len, broker_address, port_len, broker_port);

    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

    // calculate time since last connection
    uint32_t timestamp = 0;
    datastore_get_uint32(datastore, RESOURCE_ID_MQTT_TIMESTAMP, 0, &timestamp);
    uint32_t connected_time = seconds_since_boot() - timestamp;
    uint32_t days = connected_time / 60 / 60 / 24;
    uint32_t hours = connected_time / 60 / 60 % 24;
    uint32_t minutes = connected_time / 60 % 60;
    uint32_t seconds = connected_time % 60;
    snprintf(line, ROW_STRING_WIDTH, "Up%4dd %02d:%02d:%02d", days, hours, minutes, seconds);
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
}

static void _handle_page_mqtt_status(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    int * page_state = (int *)state;
    mqtt_status_t mqtt_status = 0;
    datastore_get_uint32(datastore, RESOURCE_ID_MQTT_STATUS, 0, &mqtt_status);
    if (*page_state < 8 || mqtt_status != MQTT_STATUS_CONNECTED)
    {
        _handle_page_mqtt_status_1(lcd_info, state, datastore);
    }
    else if (*page_state < 16)
    {
        _handle_page_mqtt_status_2(lcd_info, state, datastore);
    }
    else
    {
        *page_state = -1;
    }
    ++*page_state;
}

static void _handle_page_sensors_status_1(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    I2C_LCD1602_ERROR_CHECK(_clear(lcd_info));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "SENSORS_STATUS_1"));
}

static void _handle_page_sensors_status_2(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    I2C_LCD1602_ERROR_CHECK(_clear(lcd_info));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "SENSORS_STATUS_2"));
}

static void _handle_page_sensors_status_3(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    I2C_LCD1602_ERROR_CHECK(_clear(lcd_info));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "SENSORS_STATUS_3"));
}

static void _handle_page_sensors_status_4(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    I2C_LCD1602_ERROR_CHECK(_clear(lcd_info));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "SENSORS_STATUS_4"));
}

static void _handle_page_esp32_status(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    int * page_state = (int *)state;
    char line[ROW_STRING_WIDTH] = "";
    if (*page_state < 8)
    {
        char version[SYSTEM_LEN_VERSION] = "";
        datastore_get_string(datastore, RESOURCE_ID_SYSTEM_VERSION, 0, version);

        snprintf(line, ROW_STRING_WIDTH, "ESP32 v%3s      ", version);
        I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
        I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

        uint32_t uptime = seconds_since_boot(); // in seconds
        uint32_t days = uptime / 60 / 60 / 24;
        uint32_t hours = uptime / 60 / 60 % 24;
        uint32_t minutes = uptime / 60 % 60;
        uint32_t seconds = uptime % 60;
        snprintf(line, ROW_STRING_WIDTH, "Up%4dd %02d:%02d:%02d", days, hours, minutes, seconds);
        I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
        I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
    }
    else if (*page_state < 10)
    {
        snprintf(line, ROW_STRING_WIDTH, "MEM Free %7d", esp_get_free_heap_size());
        I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
        I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));

        snprintf(line, ROW_STRING_WIDTH, "IRAM Free %6d", heap_caps_get_free_size(MALLOC_CAP_32BIT));
        I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 1));
        I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, line));
    }
    else
    {
        *page_state = -1;
    }
    ++*page_state;
}

static void _handle_page_avr_status(const i2c_lcd1602_info_t * lcd_info, void * state, const datastore_t * datastore)
{
    I2C_LCD1602_ERROR_CHECK(_clear(lcd_info));
    I2C_LCD1602_ERROR_CHECK(_write_string(lcd_info, "AVR_STATUS"));
}

static void dispatch_to_handler(i2c_lcd1602_info_t * lcd_info, page_id_t current_page, const datastore_t * datastore)
{
    assert(sizeof(page_specs) / sizeof(page_specs[0]) == PAGE_LAST);

    if (current_page >= 0 && current_page < PAGE_LAST)
    {
        if (page_specs[current_page].id == current_page)
        {
            if (page_specs[current_page].handler)
            {
                page_specs[current_page].handler(lcd_info, page_specs[current_page].state, datastore);
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
    assert(pvParameter);
    ESP_LOGI(TAG, "[display] Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    i2c_master_info_t * i2c_master_info = task_inputs->i2c_master_info;
    i2c_port_t i2c_port = i2c_master_info->port;
    const datastore_t * datastore = task_inputs->datastore;

    // before accessing I2C, use a lock to gain exclusive use of the bus
    i2c_master_lock(i2c_master_info, portMAX_DELAY);

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_port, CONFIG_LCD1602_I2C_ADDRESS));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, SMBUS_TIMEOUT / portTICK_RATE_MS));

    // Set up the LCD1602 device with backlight on
    i2c_lcd1602_info_t * lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true));
    ESP_ERROR_CHECK(_display_reset(lcd_info));

    // Move to home position
    I2C_LCD1602_ERROR_CHECK(_move_cursor(lcd_info, 0, 0));
    i2c_lcd1602_write_char(lcd_info, 'B');

    i2c_master_unlock(i2c_master_info);

    page_id_t current_page = INITIAL_PAGE;

    // update pages once per second
    while (1)
    {
        ESP_LOGD(TAG, "display loop");

        i2c_master_lock(i2c_master_info, portMAX_DELAY);
        dispatch_to_handler(lcd_info, current_page, datastore);
        i2c_master_unlock(i2c_master_info);

        input_t input = INPUT_NONE;
        BaseType_t rc = xQueueReceive(button_queue, &input, TICKS_PER_UPDATE);
        if (rc == pdTRUE)
        {
            ESP_LOGI(TAG, "from queue: %d", input);
            page_id_t new_page = handle_transition(input, current_page);
            if (new_page != current_page && new_page >= 0 && new_page < PAGE_LAST)
            {
                ESP_LOGI(TAG, "change to page %d", new_page);
                current_page = new_page;

                // reset display when changing page
                _display_reset(lcd_info);

                I2C_LCD1602_ERROR_CHECK(_clear(lcd_info));

                // special case - when changing to the Last Error page, dump the entire datastore
                if (current_page == PAGE_LAST_ERROR)
                {
                    datastore_dump(datastore);
                }
            }
        }

        // TODO: reset every 5 seconds as a precaution
    }

    free(task_inputs);
    vTaskDelete(NULL);
}

static void button_task(void * pvParameter)
{
    ESP_LOGI(TAG, "[button] Core ID %d", xPortGetCoreID());

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

    bool long_sent = false;

    while (1)
    {
        // debounce first
        bool raw_state = gpio_get_level(CONFIG_DISPLAY_BUTTON_GPIO) ? false : true;
        if (raw_state == last_raw_state)
        {
            input_t input = INPUT_NONE;
            TickType_t now = xTaskGetTickCount();

            // detect edges
            if (raw_state != debounced_state)
            {
                debounced_state = raw_state;

                // if a falling edge, measure time since rising edge to detect short press
                if (!debounced_state)
                {
                    ESP_LOGD(TAG, "pressed for %d ticks", now - last_pressed);
                    if (now - last_pressed < SHORT_PRESS_THRESHOLD)
                    {
                        input = INPUT_SHORT;
                        ESP_LOGD(TAG, "short");
                    }
                    long_sent = false;
                }
                else
                {
                    last_pressed = now;
                }
            }

            // detect long hold
            if (!long_sent && debounced_state && now - last_pressed > SHORT_PRESS_THRESHOLD)
            {
                input = INPUT_LONG;
                ESP_LOGD(TAG, "long");
                long_sent = true;
            }

            if (input != INPUT_NONE)
            {
                if (xQueueSendToBack(button_queue, &input, 0) != pdTRUE)
                {
                    ESP_LOGE(TAG, "xQueueSendToBack failed");
                }
            }
        }
        last_raw_state = raw_state;

        vTaskDelay(TICKS_PER_POLL);
    }

    vTaskDelete(NULL);
}

void display_init(i2c_master_info_t * i2c_master_info, UBaseType_t priority, const datastore_t * datastore)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    static bool init = false;
    if (!init)
    {
        button_queue = xQueueCreate(10, sizeof(input_t));

        // task will take ownership of this struct
        task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
        if (task_inputs)
        {
            memset(task_inputs, 0, sizeof(*task_inputs));
            task_inputs->i2c_master_info = i2c_master_info;
            task_inputs->datastore = datastore;
            xTaskCreate(&display_task, "display_task", 4096, task_inputs, priority, NULL);
        }

        xTaskCreate(&button_task, "button_task", 2048, NULL, priority, NULL);
        init = true;
    }
    else
    {
        ESP_LOGE(TAG, "display already initialised");
    }
}
