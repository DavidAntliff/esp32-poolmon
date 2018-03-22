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

#include <stdlib.h>
#include <inttypes.h>

#include "esp_log.h"

#include "convert_string.h"

#define TAG "convert_string"

static bool _to_uint(const char * in_str, uint32_t * value, uint64_t max_value)
{
    bool ok = true;
    char * end = 0;
    uint64_t full_value = strtoull(in_str, &end, 10);
    if (end == in_str)
    {
        ESP_LOGE(TAG, "numerical conversion failed: %s", in_str);
        ok = false;
    }
    else
    {
        if (full_value <= max_value)
        {
            if (value)
            {
                *value = full_value;
            }
        }
        else
        {
            ESP_LOGE(TAG, "out of range [0, %"PRIu64"]: %s", max_value, in_str);
            ok = false;
        }
    }
    return ok;
}

static bool _to_int(const char * in_str, int32_t * value, int64_t min_value, int64_t max_value)
{
    bool ok = true;
    char * end = 0;
    int64_t full_value = strtoull(in_str, &end, 10);
    if (end == in_str)
    {
        ESP_LOGE(TAG, "numerical conversion failed: %s", in_str);
        ok = false;
    }
    else
    {
//        ESP_LOGW(TAG, "min_value %"PRId64", full_value %"PRId64", max_value %"PRId64, min_value, full_value, max_value);
//        ESP_LOGW(TAG, "0 >= -128 = %d", 0LL >= -128LL);
//        ESP_LOGW(TAG, "full_value >= min_value = %d", full_value >= min_value);
//        ESP_LOGW(TAG, "full_value <= max_value = %d", full_value <= min_value);
        if ((full_value >= min_value) && (full_value <= max_value))
        {
            if (value)
            {
                *value = full_value;
            }
        }
        else
        {
            ESP_LOGE(TAG, "out of range [%"PRId64", %"PRId64"]: %s", min_value, max_value, in_str);
            ok = false;
        }
    }
    return ok;
}

bool string_to_uint8(const char * in_str, uint8_t * value)
{
    uint32_t temp = 0;
    bool ret = _to_uint(in_str, &temp, UINT8_MAX);
    if (ret)
    {
        *value = temp;
    }
    return ret;
}

bool string_to_uint16(const char * in_str, uint16_t * value)
{
    uint32_t temp = 0;
    bool ret = _to_uint(in_str, &temp, UINT16_MAX);
    if (ret)
    {
        *value = temp;
    }
    return ret;
}

bool string_to_uint32(const char * in_str, uint32_t * value)
{
    return _to_uint(in_str, value, UINT32_MAX);
}

bool string_to_int8(const char * in_str, int8_t * value)
{
    int32_t temp = 0;
    bool ret = _to_int(in_str, &temp, INT8_MIN, INT8_MAX);
    if (ret)
    {
        *value = temp;
    }
    return ret;
}

bool string_to_int16(const char * in_str, int16_t * value)
{
    int32_t temp = 0;
    bool ret = _to_int(in_str, &temp, INT16_MIN, INT16_MAX);
    if (ret)
    {
        *value = temp;
    }
    return ret;
}

bool string_to_int32(const char * in_str, int32_t * value)
{
    return _to_int(in_str, value, INT32_MIN, INT32_MAX);
}

bool string_to_float(const char * in_str, float * value)
{
    bool ok = true;
    char * end = 0;
    float fval = strtof(in_str, &end);
    if (end == in_str)
    {
        ESP_LOGE(TAG, "numerical conversion failed: %s", in_str);
        ok = false;
    }
    else
    {
        if (value)
        {
            *value = fval;
        }
    }
    return ok;
}

bool string_to_double(const char * in_str, double * value)
{
    bool ok = true;
    char * end = 0;
    double dval = strtod(in_str, &end);
    if (end == in_str)
    {
        ESP_LOGE(TAG, "numerical conversion failed: %s", in_str);
        ok = false;
    }
    else
    {
        if (value)
        {
            *value = dval;
        }
    }
    return ok;
}
