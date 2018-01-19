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
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "datastore.h"
#include "constants.h"

#define TAG "datastore"

struct _private_t
{
    SemaphoreHandle_t semaphore;

    struct data
    {
        struct system
        {
            char version[DATASTORE_LEN_VERSION];
            char build_date_time[DATASTORE_LEN_BUILD_DATE_TIME];
            uint32_t uptime;
        } system;

        struct i2c_master
        {
            uint8_t device_count;
            uint32_t error_count;
            uint32_t error_timestamp;
        } i2c_master;

        struct wifi
        {
            uint8_t ssid[DATASTORE_LEN_WIFI_SSID];
            uint8_t password[DATASTORE_LEN_WIFI_PASSWORD];
            datastore_wifi_status_t status;
            int8_t rssi;
            uint32_t address;
        } wifi;

        struct temp
        {
            float value[DATASTORE_INSTANCES_TEMP];
            char label[DATASTORE_INSTANCES_TEMP][DATASTORE_LEN_TEMP_LABEL];
            datastore_temp_assignment_t assignment[DATASTORE_INSTANCES_TEMP];
        } temp;

        struct light
        {
            bool detected;
            uint32_t full;
            uint32_t infrared;
            uint32_t visible;
            uint32_t illuminance;
            uint32_t timestamp;
        } light;

        struct flow
        {
            float frequency;  // Hz
            float rate;       // LPM
        } flow;

    } data;

};
typedef struct _private_t private_t;

typedef enum
{
    DATASTORE_TYPE_INVALID = 0,
    DATASTORE_TYPE_BOOL,
    DATASTORE_TYPE_UINT8,
    DATASTORE_TYPE_UINT32,
    DATASTORE_TYPE_INT8,
    DATASTORE_TYPE_INT32,
    DATASTORE_TYPE_FLOAT,
    DATASTORE_TYPE_DOUBLE,
    DATASTORE_TYPE_STRING,
    DATASTORE_TYPE_LAST,
} datastore_type_t;

//typedef void (*copy_func)(uint8_t * src, uint8_t * dest, size_t len);
//
static void _get_handler(uint8_t * src, uint8_t * dest, size_t len)
{
    memcpy(dest, src, len);
}

static void _set_handler(uint8_t * src, uint8_t * dest, size_t len)
{
    memcpy(dest, src, len);
}

typedef struct
{
    datastore_id_t id;
    const char * name;
    datastore_type_t type;
    uint8_t num_instances;
    size_t offset;
//    copy_func get_handler;
//    copy_func set_handler;
    size_t size;
} index_t;

#define NAME(X) #X

#define INDEX_ROW(name, type, num_instances, field) { name, NAME(name), type, num_instances, offsetof(private_t, field), sizeof(((private_t *)0)->field) }

static index_t INDEX[] = {
    { DATASTORE_ID_SYSTEM_VERSION,         NAME(DATASTORE_ID_SYSTEM_VERSION),         DATASTORE_TYPE_STRING, 1, offsetof(private_t, data.system.version),         sizeof(((private_t *)0)->data.system.version) },
    { DATASTORE_ID_SYSTEM_BUILD_DATE_TIME, NAME(DATASTORE_ID_SYSTEM_BUILD_DATE_TIME), DATASTORE_TYPE_STRING, 1, offsetof(private_t, data.system.build_date_time), sizeof(((private_t *)0)->data.system.build_date_time) },
    { DATASTORE_ID_SYSTEM_UPTIME,          NAME(DATASTORE_ID_SYSTEM_UPTIME),          DATASTORE_TYPE_UINT32, 1, offsetof(private_t, data.system.uptime),          sizeof(((private_t *)0)->data.system.uptime) },

    INDEX_ROW(DATASTORE_ID_WIFI_SSID,     DATASTORE_TYPE_STRING, 1, data.wifi.ssid),
    INDEX_ROW(DATASTORE_ID_WIFI_PASSWORD, DATASTORE_TYPE_STRING, 1, data.wifi.password),
    INDEX_ROW(DATASTORE_ID_WIFI_STATUS,   DATASTORE_TYPE_UINT32, 1, data.wifi.status),
    INDEX_ROW(DATASTORE_ID_WIFI_RSSI,     DATASTORE_TYPE_INT8,   1, data.wifi.rssi),
    INDEX_ROW(DATASTORE_ID_WIFI_ADDRESS,  DATASTORE_TYPE_UINT32, 1, data.wifi.address),

    { DATASTORE_ID_TEMP_VALUE,             NAME(DATASTORE_ID_TEMP_VALUE),             DATASTORE_TYPE_FLOAT,  DATASTORE_INSTANCES_TEMP, offsetof(private_t, data.temp.value),      sizeof(((private_t *)0)->data.temp.value) },
    { DATASTORE_ID_TEMP_LABEL,             NAME(DATASTORE_ID_TEMP_LABEL),             DATASTORE_TYPE_STRING, DATASTORE_INSTANCES_TEMP, offsetof(private_t, data.temp.label),      sizeof(((private_t *)0)->data.temp.label) },
    { DATASTORE_ID_TEMP_ASSIGNMENT,        NAME(DATASTORE_ID_TEMP_ASSIGNMENT),        DATASTORE_TYPE_UINT8,  DATASTORE_INSTANCES_TEMP, offsetof(private_t, data.temp.assignment), sizeof(((private_t *)0)->data.temp.assignment) },

    INDEX_ROW(DATASTORE_ID_LIGHT_DETECTED,  DATASTORE_TYPE_BOOL, 1, data.light.detected),
    { DATASTORE_ID_LIGHT_FULL,              NAME(DATASTORE_ID_LIGHT_FULL),             DATASTORE_TYPE_UINT32, 1, offsetof(private_t, data.light.full),             sizeof(((private_t *)0)->data.light.full) },
    { DATASTORE_ID_LIGHT_VISIBLE,           NAME(DATASTORE_ID_LIGHT_VISIBLE),          DATASTORE_TYPE_UINT32, 1, offsetof(private_t, data.light.visible),          sizeof(((private_t *)0)->data.light.visible) },
    { DATASTORE_ID_LIGHT_INFRARED,          NAME(DATASTORE_ID_LIGHT_INFRARED),         DATASTORE_TYPE_UINT32, 1, offsetof(private_t, data.light.infrared),         sizeof(((private_t *)0)->data.light.infrared) },
    { DATASTORE_ID_LIGHT_ILLUMINANCE,       NAME(DATASTORE_ID_LIGHT_ILLUMINANCE),      DATASTORE_TYPE_UINT32, 1, offsetof(private_t, data.light.illuminance),      sizeof(((private_t *)0)->data.light.illuminance) },
    INDEX_ROW(DATASTORE_ID_LIGHT_TIMESTAMP, DATASTORE_TYPE_UINT32, 1, data.light.timestamp),

    { DATASTORE_ID_FLOW_FREQUENCY,         NAME(DATASTORE_ID_FLOW_FREQUENCY),         DATASTORE_TYPE_FLOAT,  1, offsetof(private_t, data.flow.frequency),         sizeof(((private_t *)0)->data.flow.frequency) },
    { DATASTORE_ID_FLOW_RATE,              NAME(DATASTORE_ID_FLOW_RATE),              DATASTORE_TYPE_FLOAT,  1, offsetof(private_t, data.flow.rate),              sizeof(((private_t *)0)->data.flow.rate) },

};


static datastore_error_t _is_init(const datastore_t * store)
{
    datastore_error_t err = DATASTORE_ERROR_NOT_INITIALISED;
    if (store != NULL)
    {
        if (store->private)
        {
            // OK
            err = DATASTORE_OK;
        }
        else
        {
            ESP_LOGE(TAG, "datastore is not initialised");
        }
    }
    else
    {
        ESP_LOGE(TAG, "store is NULL");
        err = DATASTORE_ERROR_NULL_POINTER;
    }
    return err;
}

datastore_t * datastore_malloc(void)
{
    datastore_t * store = NULL;
    private_t * private = malloc(sizeof(*private));
    if (private != NULL)
    {
        memset(private, 0, sizeof(*private));
        ESP_LOGD(TAG, "malloc private %p", private);

        store = malloc(sizeof(*store));
        if (store)
        {
            ESP_LOGD(TAG, "malloc store %p", store);
            memset(store, 0, sizeof(*store));
            store->private = private;
        }
        else
        {
            ESP_LOGE(TAG, "malloc failed");
        }
    }
    else
    {
        ESP_LOGE(TAG, "malloc failed");
    }

    return store;
}

void datastore_free(datastore_t ** store)
{
    if (store != NULL && (*store != NULL))
    {
        ESP_LOGD(TAG, "free private %p", (*store)->private);
        free((*store)->private);
        free(*store);
        *store = NULL;
    }
}

datastore_error_t datastore_init(datastore_t * store)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // check that the index is correct
    assert(sizeof(INDEX) / sizeof(INDEX[0]) == DATASTORE_ID_LAST);
    for (datastore_id_t id = 0; id < DATASTORE_ID_LAST; ++id)
    {
        assert(INDEX[id].id == id);
        assert(INDEX[id].type < DATASTORE_TYPE_LAST);
    }

    datastore_error_t err = DATASTORE_ERROR_UNKNOWN;
    if (store != NULL)
    {
        private_t * private = (private_t *)store->private;
        if (private != NULL)
        {
            private->semaphore = xSemaphoreCreateMutex();

            // populate some fields
            strlcpy(private->data.system.version, VERSION, DATASTORE_LEN_VERSION);
            strlcpy(private->data.system.build_date_time, BUILD_TIMESTAMP, DATASTORE_LEN_BUILD_DATE_TIME);

            err = DATASTORE_OK;
        }
        else
        {
            ESP_LOGE(TAG, "store->private is NULL");
            err = DATASTORE_ERROR_NULL_POINTER;
        }
    }
    else
    {
        ESP_LOGE(TAG, "store is NULL");
        err = DATASTORE_ERROR_NULL_POINTER;
    }
    return err;
}

static datastore_error_t _set_value(const datastore_t * store, datastore_id_t id, instance_id_t instance, const void * value, datastore_type_t expected_type)
{
    ESP_LOGD(TAG":_set_value", "id %d, instance %d, value %p, expected_type %d", id, instance, value, expected_type);
    datastore_error_t err = DATASTORE_ERROR_UNKNOWN;
    if ((err = _is_init(store)) == DATASTORE_OK)
    {
        private_t * private = (private_t *)store->private;
        if (private != NULL)
        {
            if (id >= 0 && id < DATASTORE_ID_LAST)
            {
                // check type
                if (INDEX[id].type == expected_type)
                {
                    // check instance
                    if (/*instance >= 0 &&*/ instance < INDEX[id].num_instances)
                    {
                        if (value)
                        {
                            // finally, set the value
                            size_t instance_size = INDEX[id].size / INDEX[id].num_instances;
                            assert(instance_size * INDEX[id].num_instances == INDEX[id].size);

                            uint8_t * pdest = (uint8_t *)private + INDEX[id].offset + instance * instance_size;
                            ESP_LOGD(TAG, "_set_value: id %d, instance %d, value %p, type %d, private %p, offset 0x%x, size 0x%x, instance_size 0x%x, pdest %p",
                                     id, instance, value, INDEX[id].type, private, INDEX[id].offset, INDEX[id].size, instance_size, pdest);

                            xSemaphoreTake(private->semaphore, portMAX_DELAY);
                            _set_handler((uint8_t *)value, pdest, instance_size);
                            ESP_LOG_BUFFER_HEXDUMP(TAG, pdest, instance_size, ESP_LOG_DEBUG);
                            xSemaphoreGive(private->semaphore);

                            // TODO: call any registered callbacks with new value
                        }
                        else
                        {
                            ESP_LOGE(TAG, "_set_value: value is NULL");
                            err = DATASTORE_ERROR_NULL_POINTER;
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "_set_value: instance %d is invalid", instance);
                        err = DATASTORE_ERROR_INVALID_INSTANCE;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "_set_value: bad type %d (expected %d)", INDEX[id].type, expected_type);
                    err = DATASTORE_ERROR_INVALID_TYPE;
                }
            }
            else
            {
                ESP_LOGE(TAG, "_set_value: bad id %d", id);
                err = DATASTORE_ERROR_INVALID_ID;
            }
        }
        else
        {
            ESP_LOGE(TAG, "_set_value: private is NULL");
            err = DATASTORE_ERROR_NULL_POINTER;
        }
    }
    else
    {
        ESP_LOGE(TAG, "_set_value: datastore is not initialised");
        err = DATASTORE_ERROR_NOT_INITIALISED;
    }
    return err;
}

datastore_error_t datastore_set_bool(datastore_t * store, datastore_id_t id, instance_id_t instance, bool value)
{
    return _set_value(store, id, instance, &value, DATASTORE_TYPE_BOOL);
}

datastore_error_t datastore_set_uint8(datastore_t * store, datastore_id_t id, instance_id_t instance, uint8_t value)
{
    return _set_value(store, id, instance, &value, DATASTORE_TYPE_UINT8);
}

datastore_error_t datastore_set_uint32(datastore_t * store, datastore_id_t id, instance_id_t instance, uint32_t value)
{
    return _set_value(store, id, instance, &value, DATASTORE_TYPE_UINT32);
}

datastore_error_t datastore_set_int8(datastore_t * store, datastore_id_t id, instance_id_t instance, int8_t value)
{
    return _set_value(store, id, instance, &value, DATASTORE_TYPE_INT8);
}

datastore_error_t datastore_set_int32(datastore_t * store, datastore_id_t id, instance_id_t instance, int32_t value)
{
    return _set_value(store, id, instance, &value, DATASTORE_TYPE_UINT32);
}

datastore_error_t datastore_set_float(datastore_t * store, datastore_id_t id, instance_id_t instance, float value)
{
    return _set_value(store, id, instance, &value, DATASTORE_TYPE_FLOAT);
}

datastore_error_t datastore_set_double(datastore_t * store, datastore_id_t id, instance_id_t instance, double value)
{
    return _set_value(store, id, instance, &value, DATASTORE_TYPE_DOUBLE);
}

datastore_error_t datastore_set_string(datastore_t * store, datastore_id_t id, instance_id_t instance, const char * value)
{
    return _set_value(store, id, instance, value, DATASTORE_TYPE_STRING);
}

static datastore_error_t _get_value(const datastore_t * store, datastore_id_t id, instance_id_t instance, void * value, datastore_type_t expected_type)
{
    ESP_LOGD(TAG":_get_value", "id %d, instance %d, value %p, expected_type %d", id, instance, value, expected_type);
    datastore_error_t err = DATASTORE_ERROR_UNKNOWN;
    if ((err = _is_init(store)) == DATASTORE_OK)
    {
        private_t * private = (private_t *)store->private;
        if (private != NULL)
        {
            if (id >= 0 && id < DATASTORE_ID_LAST)
            {
                // check type
                if (INDEX[id].type == expected_type)
                {
                    // check instance
                    if (/*instance >= 0 &&*/ instance < INDEX[id].num_instances)
                    {
                        if (value)
                        {
                            // finally, get the value
                            size_t instance_size = INDEX[id].size / INDEX[id].num_instances;
                            assert(instance_size * INDEX[id].num_instances == INDEX[id].size);

                            uint8_t * psrc = (uint8_t *)private + INDEX[id].offset + instance * instance_size;
                            ESP_LOGD(TAG, "_get_value: id %d, instance %d, value %p, type %d, private %p, offset 0x%x, size 0x%x, instance_size 0x%x, psrc %p",
                                     id, instance, value, INDEX[id].type, private, INDEX[id].offset, INDEX[id].size, instance_size, psrc);
                            ESP_LOG_BUFFER_HEXDUMP(TAG, psrc, instance_size, ESP_LOG_DEBUG);

                            xSemaphoreTake(private->semaphore, portMAX_DELAY);
                            _get_handler(psrc, (uint8_t *)value, instance_size);
                            xSemaphoreGive(private->semaphore);

                            // TODO: call any registered callbacks with new value
                        }
                        else
                        {
                            ESP_LOGE(TAG, "_get_value: value is NULL");
                            err = DATASTORE_ERROR_NULL_POINTER;
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "_get_value: instance %d is invalid", instance);
                        err = DATASTORE_ERROR_INVALID_INSTANCE;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "_get_value: bad type %d (expected %d)", INDEX[id].type, expected_type);
                    err = DATASTORE_ERROR_INVALID_TYPE;
                }
            }
            else
            {
                ESP_LOGE(TAG, "_get_value: bad id %d", id);
                err = DATASTORE_ERROR_INVALID_ID;
            }
        }
        else
        {
            ESP_LOGE(TAG, "_get_value: private is NULL");
            err = DATASTORE_ERROR_NULL_POINTER;
        }
    }
    else
    {
        ESP_LOGE(TAG, "_get_value: datastore is not initialised");
        err = DATASTORE_ERROR_NOT_INITIALISED;
    }
    return err;
}

datastore_error_t datastore_get_bool(const datastore_t * store, datastore_id_t id, instance_id_t instance, bool * value)
{
    return _get_value(store, id, instance, value, DATASTORE_TYPE_BOOL);
}

datastore_error_t datastore_get_uint8(const datastore_t * store, datastore_id_t id, instance_id_t instance, uint8_t * value)
{
    return _get_value(store, id, instance, value, DATASTORE_TYPE_UINT8);
}

datastore_error_t datastore_get_uint32(const datastore_t * store, datastore_id_t id, instance_id_t instance, uint32_t * value)
{
    return _get_value(store, id, instance, value, DATASTORE_TYPE_UINT32);
}

datastore_error_t datastore_get_int8(const datastore_t * store, datastore_id_t id, instance_id_t instance, int8_t * value)
{
    return _get_value(store, id, instance, value, DATASTORE_TYPE_INT8);
}

datastore_error_t datastore_get_int32(const datastore_t * store, datastore_id_t id, instance_id_t instance, int32_t * value)
{
    return _get_value(store, id, instance, value, DATASTORE_TYPE_INT32);
}

datastore_error_t datastore_get_float(const datastore_t * store, datastore_id_t id, instance_id_t instance, float * value)
{
    return _get_value(store, id, instance, value, DATASTORE_TYPE_FLOAT);
}

datastore_error_t datastore_get_double(const datastore_t * store, datastore_id_t id, instance_id_t instance, double * value)
{
    return _get_value(store, id, instance, value, DATASTORE_TYPE_DOUBLE);
}

datastore_error_t datastore_get_string(const datastore_t * store, datastore_id_t id, instance_id_t instance, char * value)
{
    return _get_value(store, id, instance, value, DATASTORE_TYPE_STRING);
}

datastore_error_t _to_string(const datastore_t * store, datastore_id_t id, instance_id_t instance, char * buffer, size_t len)
{
    datastore_error_t err = DATASTORE_ERROR_UNKNOWN;
    if (id >= 0 && id < DATASTORE_ID_LAST)
    {
        switch (INDEX[id].type)
        {
            case DATASTORE_TYPE_BOOL:
            {
                bool value = 0;
                datastore_get_bool(store, id, instance, &value);
                snprintf(buffer, len, "%s", value ? "true" : "false");
                err = DATASTORE_OK;
                break;
            }
            case DATASTORE_TYPE_UINT8:
            {
                uint8_t value = 0;
                datastore_get_uint8(store, id, instance, &value);
                snprintf(buffer, len, "%u", value);
                err = DATASTORE_OK;
                break;
            }
            case DATASTORE_TYPE_UINT32:
            {
                uint32_t value = 0;
                datastore_get_uint32(store, id, instance, &value);
                snprintf(buffer, len, "%u", value);
                err = DATASTORE_OK;
                break;
            }
            case DATASTORE_TYPE_INT8:
            {
                int8_t value = 0;
                datastore_get_int8(store, id, instance, &value);
                snprintf(buffer, len, "%d", value);
                err = DATASTORE_OK;
                break;
            }
            case DATASTORE_TYPE_INT32:
            {
                int32_t value = 0;
                datastore_get_int32(store, id, instance, &value);
                snprintf(buffer, len, "%d", value);
                err = DATASTORE_OK;
                break;
            }
            case DATASTORE_TYPE_FLOAT:
            {
                float value = 0.0f;
                datastore_get_float(store, id, instance, &value);
                snprintf(buffer, len, "%f", value);
                err = DATASTORE_OK;
                break;
            }
            case DATASTORE_TYPE_DOUBLE:
            {
                double value = 0.0;
                datastore_get_double(store, id, instance, &value);
                snprintf(buffer, len, "%lf", value);
                err = DATASTORE_OK;
                break;
            }
            case DATASTORE_TYPE_STRING:
            {
                // hope that the supplied buffer is big enough...
                //ESP_LOGI(TAG, "len %d, INDEX[id].size %d", len, INDEX[id].size);
                assert(len >= INDEX[id].size);
                datastore_get_string(store, id, instance, buffer);
                err = DATASTORE_OK;
                break;
            }
            default:
                ESP_LOGE(TAG, "unhandled type %d", INDEX[id].type);
                err = DATASTORE_ERROR_INVALID_TYPE;
                break;
        }
    }
    else
    {
        ESP_LOGE(TAG, "invalid datastore ID %d", id);
        err = DATASTORE_ERROR_INVALID_ID;
    }
    return err;
}

datastore_error_t datastore_dump(const datastore_t * store)
{
    datastore_error_t err = DATASTORE_ERROR_UNKNOWN;
    if ((err = _is_init(store)) == DATASTORE_OK)
    {
        for (datastore_id_t id = 0; id < DATASTORE_ID_LAST; ++id)
        {
            err = DATASTORE_OK;
            for (instance_id_t instance = 0; err == DATASTORE_OK && instance < INDEX[id].num_instances; ++instance)
            {
                char value[256] = "";
                err = _to_string(store, id, instance, value, 256);
                ESP_LOGI(TAG, LOG_COLOR(LOG_COLOR_PURPLE)"%2d %-40s %d %s", id, INDEX[id].name, instance, value);
            }
            if (err != DATASTORE_OK)
            {
                ESP_LOGE(TAG, "Error %d", err);
            }
        }
    }
    return err;
}

