/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
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

#ifndef WIFI_SUPPORT_H
#define WIFI_SUPPORT_H

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "datastore/datastore.h"

typedef enum
{
    WIFI_STATUS_DISCONNECTED = 0,
    WIFI_STATUS_CONNECTED,
    WIFI_STATUS_GOT_ADDRESS,
} wifi_status_t;

#define WIFI_LEN_SSID        (sizeof(((wifi_sta_config_t *)0)->ssid))
#define WIFI_LEN_PASSWORD    (sizeof(((wifi_sta_config_t *)0)->password))

void wifi_support_init(UBaseType_t priority, const datastore_t * datastore);
void wifi_support_delete(void);

#endif // WIFI_SUPPORT_H
