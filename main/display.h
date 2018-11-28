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

#ifndef DISPLAY_H
#define DISPLAY_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "i2c_master.h"
#include "datastore/datastore.h"

typedef enum
{
    DISPLAY_PAGE_IGNORE = -1,        // when used in a transition, ignore the rule
    DISPLAY_PAGE_BLANK = 0,
    DISPLAY_PAGE_MAIN,
    DISPLAY_PAGE_SENSORS_TEMP,
    DISPLAY_PAGE_SENSORS_TEMP_2,
    DISPLAY_PAGE_SENSORS_LIGHT,
    DISPLAY_PAGE_SENSORS_FLOW,
    DISPLAY_PAGE_POWER,
    DISPLAY_PAGE_SWITCHES,
    DISPLAY_PAGE_PUMP_STATUS,
    DISPLAY_PAGE_CP_CONTROL,
    DISPLAY_PAGE_PP_CONTROL,
    DISPLAY_PAGE_ALARM,
    DISPLAY_PAGE_WIFI_STATUS,
    DISPLAY_PAGE_MQTT_STATUS,
    DISPLAY_PAGE_RESOURCE_STATUS,
    DISPLAY_PAGE_AVR_STATUS,
    DISPLAY_PAGE_LAST,
} display_page_id_t;

void display_init(i2c_master_info_t * i2c_master_info, UBaseType_t priority, const datastore_t * datastore);
void display_delete(void);

// Return true if the specified page is currently being displayed
bool display_is_currently(const datastore_t * datastore, display_page_id_t page);

#endif // DISPLAY_H
