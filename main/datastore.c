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

        struct light
        {
            uint32_t full;
            uint32_t infrared;
            uint32_t visible;
            uint32_t illuminance;
        } light;

    } data;

};
typedef struct _private_t private_t;

typedef enum
{
    DATASTORE_TYPE_INVALID = 0,
    DATASTORE_TYPE_UINT8,
    DATASTORE_TYPE_UINT32,
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

static index_t INDEX[] = {
    { DATASTORE_ID_SYSTEM_VERSION,         NAME(DATASTORE_ID_SYSTEM_VERSION), DATASTORE_TYPE_STRING, 1, offsetof(private_t, data.system.version), sizeof(((private_t *)0)->data.system.version) },
    { DATASTORE_ID_SYSTEM_BUILD_DATE_TIME, NAME(DATASTORE_ID_SYSTEM_BUILD_DATE_TIME), DATASTORE_TYPE_STRING, 1, offsetof(private_t, data.system.build_date_time), sizeof(((private_t *)0)->data.system.build_date_time) },
    { DATASTORE_ID_SYSTEM_UPTIME,          NAME(DATASTORE_ID_SYSTEM_UPTIME), DATASTORE_TYPE_UINT32, 1, offsetof(private_t, data.system.uptime), sizeof(((private_t *)0)->data.system.uptime) },

    { DATASTORE_ID_LIGHT_FULL,             NAME(DATASTORE_ID_LIGHT_FULL), DATASTORE_TYPE_UINT32, 1, offsetof(private_t, data.light.full), sizeof(((private_t *)0)->data.light.full) },
    { DATASTORE_ID_LIGHT_VISIBLE,          NAME(DATASTORE_ID_LIGHT_VISIBLE), DATASTORE_TYPE_UINT32, 1, offsetof(private_t, data.light.visible), sizeof(((private_t *)0)->data.light.visible) },
    { DATASTORE_ID_LIGHT_INFRARED,         NAME(DATASTORE_ID_LIGHT_INFRARED), DATASTORE_TYPE_UINT32, 1, offsetof(private_t, data.light.infrared), sizeof(((private_t *)0)->data.light.infrared) },
    { DATASTORE_ID_LIGHT_ILLUMINANCE,      NAME(DATASTORE_ID_LIGHT_ILLUMINANCE), DATASTORE_TYPE_UINT32, 1, offsetof(private_t, data.light.illuminance), sizeof(((private_t *)0)->data.light.illuminance) },

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
    // check that the index is correct
    assert(sizeof(INDEX) / sizeof(INDEX[0]) == DATASTORE_ID_LAST);
    for (datastore_id_t id = 0; id < DATASTORE_ID_LAST; ++id)
    {
        assert(INDEX[id].id == id);
//        assert(INDEX[id].type < DATASTORE_TYPE_LAST);
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

datastore_error_t datastore_set_uint8(datastore_t * store, datastore_id_t id, instance_id_t instance, uint8_t value);

datastore_error_t datastore_set_uint32(datastore_t * store, datastore_id_t id, instance_id_t instance, uint32_t value)
{
    datastore_error_t err = DATASTORE_ERROR_UNKNOWN;
    if ((err = _is_init(store)) == DATASTORE_OK)
    {
        private_t * private = (private_t *)store->private;
        if (private != NULL)
        {
            if (id >= 0 && id < DATASTORE_ID_LAST)
            {
                // check type
                if (INDEX[id].type == DATASTORE_TYPE_UINT32)
                {
                    // check instance
                    if (instance >= 0 && instance < INDEX[id].num_instances)
                    {
                        ESP_LOGI(TAG, "datastore_set_uint32 %d %d %d", id, instance, value);

                        // get the mutex
                        xSemaphoreTake(private->semaphore, portMAX_DELAY);

                        // set the value
                        uint8_t * pdest = (uint8_t *)private + INDEX[id].offset;
//                        INDEX[id].set_handler((uint8_t *)&value, pdest, sizeof(uint32_t));
                        _set_handler((uint8_t *)&value, pdest, INDEX[id].size);

                        // release the mutex
                        xSemaphoreGive(private->semaphore);

                        // TODO: call any registered callbacks with new value
                    }
                }
            }
        }
    }
    return err;
}

datastore_error_t datastore_set_int8(datastore_t * store, datastore_id_t id, instance_id_t instance, int8_t value);
datastore_error_t datastore_set_int32(datastore_t * store, datastore_id_t id, instance_id_t instance, int32_t value);
datastore_error_t datastore_set_float(datastore_t * store, datastore_id_t id, instance_id_t instance, float value);
datastore_error_t datastore_set_double(datastore_t * store, datastore_id_t id, instance_id_t instance, double value);
datastore_error_t datastore_set_string(datastore_t * store, datastore_id_t id, instance_id_t instance, const char * value)
{
    datastore_error_t err = DATASTORE_ERROR_UNKNOWN;
    if ((err = _is_init(store)) == DATASTORE_OK)
    {
        private_t * private = (private_t *)store->private;
        if (private != NULL)
        {
            if (id >= 0 && id < DATASTORE_ID_LAST)
            {
                // check type
                if (INDEX[id].type == DATASTORE_TYPE_STRING)
                {
                    // check instance
                    if (instance >= 0 && instance < INDEX[id].num_instances)
                    {
                        ESP_LOGI(TAG, "datastore_set_string %d %d %s", id, instance, value);

                        // get the mutex
                        xSemaphoreTake(private->semaphore, portMAX_DELAY);

                        // set the value
                        uint8_t * pdest = (uint8_t *)private + INDEX[id].offset;
//                        INDEX[id].set_handler((uint8_t *)&value, pdest, sizeof(uint32_t));
                        _set_handler((uint8_t *)&value, pdest, INDEX[id].size);

                        // release the mutex
                        xSemaphoreGive(private->semaphore);

                        // TODO: call any registered callbacks with new value
                    }
                }
            }
        }
    }
    return err;
}

datastore_error_t datastore_get_uint8(const datastore_t * store, datastore_id_t id, instance_id_t instance, uint8_t * value)
{
    return DATASTORE_ERROR_UNKNOWN;
}
datastore_error_t datastore_get_uint32(const datastore_t * store, datastore_id_t id, instance_id_t instance, uint32_t * value)
{
    datastore_error_t err = DATASTORE_ERROR_UNKNOWN;
    if ((err = _is_init(store)) == DATASTORE_OK)
    {
        private_t * private = (private_t *)store->private;
        if (private != NULL)
        {
            if (id >= 0 && id < DATASTORE_ID_LAST)
            {
                // check type
                if (INDEX[id].type == DATASTORE_TYPE_UINT32)
                {
                    // check instance
                    if (instance >= 0 && instance < INDEX[id].num_instances)
                    {
                        if (value)
                        {
                            // get the mutex
                            xSemaphoreTake(private->semaphore, portMAX_DELAY);

                            // get the value
                            uint8_t * psrc = (uint8_t *)private + INDEX[id].offset;
//                            INDEX[id].get_handler(psrc, (uint8_t *)value, sizeof(uint32_t));
                            _get_handler(psrc, (uint8_t *)value, INDEX[id].size);

                            // release the mutex
                            xSemaphoreGive(private->semaphore);

                            ESP_LOGI(TAG, "datastore_get_uint32 %d %d %d", id, instance, *value);

                            // TODO: call any registered callbacks with new value
                        }
                    }
                }
            }
        }
    }
    return err;
}
datastore_error_t datastore_get_int8(const datastore_t * store, datastore_id_t id, instance_id_t instance, int8_t * value);
datastore_error_t datastore_get_int32(const datastore_t * store, datastore_id_t id, instance_id_t instance, int32_t * value);
datastore_error_t datastore_get_float(const datastore_t * store, datastore_id_t id, instance_id_t instance, float * value);
datastore_error_t datastore_get_double(const datastore_t * store, datastore_id_t id, instance_id_t instance, double * value);
datastore_error_t datastore_get_string(const datastore_t * store, datastore_id_t id, instance_id_t instance, char * value)
{
    datastore_error_t err = DATASTORE_ERROR_UNKNOWN;
    if ((err = _is_init(store)) == DATASTORE_OK)
    {
        private_t * private = (private_t *)store->private;
        if (private != NULL)
        {
            if (id >= 0 && id < DATASTORE_ID_LAST)
            {
                // check type
                if (INDEX[id].type == DATASTORE_TYPE_STRING)
                {
                    // check instance
                    if (instance >= 0 && instance < INDEX[id].num_instances)
                    {
                        if (value)
                        {
                            // get the mutex
                            xSemaphoreTake(private->semaphore, portMAX_DELAY);

                            // get the value
                            uint8_t * psrc = (uint8_t *)private + INDEX[id].offset;
//                            INDEX[id].get_handler(psrc, (uint8_t *)value, sizeof(uint32_t));
                            _get_handler(psrc, (uint8_t *)value, INDEX[id].size);

                            // release the mutex
                            xSemaphoreGive(private->semaphore);

                            ESP_LOGI(TAG, "datastore_get_string %d %d %s", id, instance, value);

                            // TODO: call any registered callbacks with new value
                        }
                        else
                        {
                            err = DATASTORE_ERROR_NULL_POINTER;
                        }

                    }
                    else
                    {
                        err = DATASTORE_ERROR_INVALID_INSTANCE;
                    }
                }
                else
                {
                    err = DATASTORE_ERROR_INVALID_TYPE;
                }
            }
            else
            {
                err = DATASTORE_ERROR_INVALID_ID;
            }
        }
        else
        {
            err = DATASTORE_ERROR_NULL_POINTER;
        }
    }
    else
    {
        err = DATASTORE_ERROR_NOT_INITIALISED;
    }
    return err;
}

datastore_error_t _to_string(const datastore_t * store, datastore_id_t id, instance_id_t instance, char * buffer, size_t len)
{
    datastore_error_t err = DATASTORE_ERROR_UNKNOWN;
    if (id >= 0 && id < DATASTORE_ID_LAST)
    {
        switch (INDEX[id].type)
        {
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
            case DATASTORE_TYPE_STRING:
            {
                // hope that the supplied buffer is big enough...
                ESP_LOGI(TAG, "len %d, INDEX[id].size %d", len, INDEX[id].size);
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
                ESP_LOGW(TAG, "%2d %-40s %d %s", id, INDEX[id].name, instance, value);
            }
            if (err != DATASTORE_OK)
            {
                ESP_LOGE(TAG, "Error %d", err);
            }
        }
    }
    return err;
}

