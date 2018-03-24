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

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "nvs_support.h"

#define TAG "nvs_support"

esp_err_t nvs_support_init(const char * namespace)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_LOGW(TAG, "NVS partition was truncated and will be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err == ESP_OK && namespace != NULL && namespace[0] != '\0')
    {
        // attempt to open the namespace, and create it if it doesn't exist
        nvs_handle nh;
        err = nvs_open(namespace, NVS_READONLY, &nh);
        switch (err)
        {
            case ESP_ERR_NVS_NOT_FOUND:
            {
                // create the namespace
                ESP_LOGW(TAG, "Create NVS namespace [%s]", namespace);
                err = nvs_open(namespace, NVS_READWRITE, &nh);
                if (err == ESP_OK)
                {
                    nvs_commit(nh);
                    nvs_close(nh);
                }
                else
                {
                    ESP_LOGE(TAG, "Error %d opening NVS handle", err);
                }
                break;
            }
            case ESP_OK:
            {
                ESP_LOGI(TAG, "NVS namespace [%s] ready", namespace);
                break;
            }
            default:
            {
                ESP_LOGE(TAG, "Error %d opening NVS handle", err);
                break;
            }
        }
    }
    return err;
}

esp_err_t nvs_support_erase_all(const char * namespace)
{
    nvs_handle nh;
    esp_err_t err = nvs_open(namespace, NVS_READWRITE, &nh);
    if (err == ESP_OK)
    {
        err = nvs_erase_all(nh);
        nvs_commit(nh);
        nvs_close(nh);
    }
    else
    {
        ESP_LOGE(TAG, "Error %d opening NVS handle", err);
    }
    return err;
}

