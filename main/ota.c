/*
 * MIT License
 *
 * Copyright (c) 2019 David Antliff
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
 *
 * Parts of this file copied from Public Domain release:
 *   https://github.com/espressif/esp-idf/blob/release/v3.0/examples/system/ota/main/ota_example_main.c
 */

#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "ota.h"
#include "resources.h"

#define TAG "ota"

#define BUFFSIZE 1024
#define TEXT_BUFFSIZE 1024

/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = { 0 };
/*an packet receive buffer*/
static char text[BUFFSIZE + 1] = { 0 };
/* an image total length*/
static int binary_file_length = 0;
/*socket id*/
static int socket_id = -1;

#define OTA_PERIOD (1)

typedef struct
{
    const datastore_t * datastore;
} task_inputs_t;

static TaskHandle_t _task_handle = NULL;

/*read buffer by byte still delim ,return read bytes counts*/
static int read_until(char *buffer, char delim, int len)
{
//  /*TODO: delim check,buffer check,further: do an buffer length limited*/
    int i = 0;
    while (buffer[i] != delim && i < len) {
        ++i;
    }
    return i + 1;
}

/* Resolve a packet from http socket
 * return true if packet including \r\n\r\n that means http packet header finished,start to receive packet body
 * otherwise return false
 * */
static bool read_past_http_header(char text[], int total_len, esp_ota_handle_t update_handle)
{
    ESP_LOGD(TAG, "read_past_http_header: total len %d", total_len);
    // i means current position
    int i = 0, i_read_len = 0;
    while (text[i] != 0 && i < total_len)
    {
        i_read_len = read_until(&text[i], '\n', total_len);
        // if we resolve \r\n line,we think packet header is finished
        if (i_read_len == 2)
        {
            int i_write_len = total_len - (i + 2);
            memset(ota_write_data, 0, BUFFSIZE);
            // copy first http packet body to write buffer
            memcpy(ota_write_data, &(text[i + 2]), i_write_len);

            esp_err_t err = esp_ota_write(update_handle, (const void *) ota_write_data, i_write_len);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
                return false;
            }
            else
            {
                ESP_LOGI(TAG, "esp_ota_write header OK");
                binary_file_length += i_write_len;
            }
            return true;
        }
        i += i_read_len;
    }
    return false;
}

static bool connect_to_http_server(const char * server_name, const char * server_port)
{
    ESP_LOGI(TAG, "connect_to_http_server: server_name %s, server_port %s", server_name, server_port);

    // resolve address
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res, *rp;

    int s = getaddrinfo(server_name, server_port, &hints, &res);
    if (s != 0)
    {
        ESP_LOGE(TAG, "getaddrinfo failed: %d", s);
        return false;
    }

    // try each address until we successfully connect:
    for (rp = res; rp != NULL; rp = rp->ai_next)
    {
        // print the resolved IP address
        char addrstr[100];
        void *ptr;
        inet_ntop (res->ai_family, res->ai_addr->sa_data, addrstr, 100);
        switch (res->ai_family)
        {
            case AF_INET:
                ptr = &((struct sockaddr_in *) res->ai_addr)->sin_addr;
                break;
            case AF_INET6:
                ptr = &((struct sockaddr_in6 *) res->ai_addr)->sin6_addr;
                break;
        }
        inet_ntop (res->ai_family, ptr, addrstr, 100);
        ESP_LOGD(TAG, "IPv%d address: %s (%s)", res->ai_family == PF_INET6 ? 6 : 4,
                addrstr, res->ai_canonname);

        socket_id = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (socket_id == -1)
        {
            ESP_LOGE(TAG, "Create socket failed!");
            continue;
        }
        ESP_LOGD(TAG, "socket created");

        // connect to http server
        if (connect(socket_id, rp->ai_addr, rp->ai_addrlen) == -1)
        {
            ESP_LOGE(TAG, "Connect to server failed! errno=%d", errno);
            close(socket_id);
            continue;
        }
        else
        {
            ESP_LOGI(TAG, "Connected to server");
            break;
        }
    }

    if (rp == NULL)
    {
        ESP_LOGE(TAG, "Could not connect to %s:%s", server_name, server_port);
    }

    freeaddrinfo(res);
    return rp != NULL;
}

static bool _do_ota_upgrade(const char * server_name, const char * server_port, const char * filename)
{
    esp_err_t err;
    // update handle : set by esp_ota_begin(), must be freed via esp_ota_end()
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGD(TAG, "_do_ota_upgrade: http://%s:%s%s", server_name, server_port, filename);

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    ESP_LOGD(TAG, "esp_ota_get_boot_partition() -> %p", configured);
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGD(TAG, "esp_ota_get_running_partition() -> %p", running);

    if (configured != NULL && configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)", running->type, running->subtype, running->address);

    // connect to http server
    if (connect_to_http_server(server_name, server_port)) {
        ESP_LOGI(TAG, "Connected to http server");
    } else {
        ESP_LOGE(TAG, "Connect to http server failed!");
        return false;
    }

    // send GET request to http server
    const char * GET_FORMAT =
        "GET %s HTTP/1.0\r\n"
        "Host: %s:%s\r\n"
        "User-Agent: esp-idf/1.0 esp32\r\n\r\n";

    char * http_request = NULL;
    int get_len = asprintf(&http_request, GET_FORMAT, filename, server_name, server_port);
    if (get_len < 0)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for GET request buffer");
        return false;
    }
    int res = send(socket_id, http_request, get_len, 0);
    free(http_request);

    if (res < 0)
    {
        ESP_LOGE(TAG, "Send GET request to server failed");
        return false;
    }
    else
    {
        ESP_LOGI(TAG, "Send GET request to server succeeded");
    }

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGD(TAG, "esp_ota_get_next_update_partition(NULL) -> %p", update_partition);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x", update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed, error=%d", err);
        return false;
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");

    bool resp_body_start = false, flag = true;
    // deal with all receive packet
    while (flag)
    {
        ESP_LOGD(TAG, "Waiting for data");
        memset(text, 0, TEXT_BUFFSIZE);
        memset(ota_write_data, 0, BUFFSIZE);
        int buff_len = recv(socket_id, text, TEXT_BUFFSIZE, 0);
        if (buff_len < 0)
        {
            // receive error
            ESP_LOGE(TAG, "Error: receive data error! errno=%d", errno);
            return false;
        }
        else if (buff_len > 0 && !resp_body_start)
        {
            // deal with response header
            ESP_LOGD(TAG, "received response header");
            memcpy(ota_write_data, text, buff_len);
            resp_body_start = read_past_http_header(text, buff_len, update_handle);
        }
        else if (buff_len > 0 && resp_body_start)
        {
            // deal with response body
            memcpy(ota_write_data, text, buff_len);
            err = esp_ota_write(update_handle, (const void *) ota_write_data, buff_len);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
                return false;
            }
            binary_file_length += buff_len;
            ESP_LOGI(TAG, "Have written image length %d", binary_file_length);
        }
        else if (buff_len == 0)
        {
            // packet over
            flag = false;
            ESP_LOGI(TAG, "Connection closed, all packets received");
            close(socket_id);
        }
        else
        {
            ESP_LOGE(TAG, "Unexpected recv result");
        }

        // avoid watchdog with large binaries
        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);

    if (esp_ota_end(update_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        return false;
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
        return false;
    }
    return true;
}

static void _ota_url_changed(const datastore_t * datastore, datastore_resource_id_t id, datastore_instance_id_t instance, void * context)
{
    ESP_LOGD(TAG, "ota url changed");
    bool * trigger = (bool *)context;
    *trigger = true;
}

#define SERVER_NAME_LEN 255
#define SERVER_PORT_LEN 5
#define FILENAME_LEN 255

#define S_(x) #x
#define S(x) S_(x)

static bool _parse_url(const char * url,
                       char * server_name,
                       char * server_port,
                       char * filename)
{
    bool result = false;
    result = sscanf(url, "http://%" S(SERVER_NAME_LEN) "[^:]:%" S(SERVER_PORT_LEN) "[^/]%" S(FILENAME_LEN) "[^\n]", server_name, server_port, filename) == 3;
    if (!result)
    {
        result = sscanf(url, "http://%" S(SERVER_NAME_LEN) "[^/]%" S(FILENAME_LEN) "[^\n]", server_name, filename) == 2;
        // leave server_port unmodified
    }
    ESP_LOGD(TAG, "_parse_url(%s): server_name '%s', server_port '%s', filename '%s'", url, server_name, server_port, filename);
    return result;
}

static void ota_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());
    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    const datastore_t * datastore = task_inputs->datastore;

    TickType_t last_wake_time = xTaskGetTickCount();

    bool trigger = false;
    datastore_add_set_callback(datastore, RESOURCE_ID_OTA_URL, 0, _ota_url_changed, &trigger);

    while (1)
    {
        ESP_LOGD(TAG, "ota loop");
        last_wake_time = xTaskGetTickCount();

        if (trigger)
        {
            char server_name[SERVER_NAME_LEN + 1] = "";
            char server_port[SERVER_PORT_LEN + 1] = "80";
            char filename[FILENAME_LEN + 1] = "";
            binary_file_length = 0;

            char url[OTA_URL_LEN] = "";
            datastore_get_as_string(datastore, RESOURCE_ID_OTA_URL, 0, url, sizeof(url));

            char buffer[256] = "";
            snprintf(buffer, 256, "OTA upgrade initiated: %s", url);
            ESP_LOGI(TAG, buffer);
            datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, buffer);

            if (_parse_url(url, server_name, server_port, filename))
            {
                if (_do_ota_upgrade(server_name, server_port, filename))
                {
                    ESP_LOGI(TAG, "OTA upgrade successful - reboot to activate new software");
                    datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "OTA upgrade successful");
                }
                else
                {
                    ESP_LOGE(TAG, "OTA upgrade failed");
                    datastore_set_string(datastore, RESOURCE_ID_SYSTEM_LOG, 0, "OTA upgrade failed");
                }
            }
            else
            {
                ESP_LOGE(TAG, "Error parsing URL '%s'", url);
            }
            trigger = false;
        }

        vTaskDelayUntil(&last_wake_time, OTA_PERIOD * 1000 / portTICK_PERIOD_MS);
    }

    free(task_inputs);
    _task_handle = NULL;
    vTaskDelete(NULL);
}

void ota_init(UBaseType_t priority, const datastore_t * datastore)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->datastore = datastore;
        xTaskCreate(&ota_task, "ota_task", 8192, task_inputs, priority, &_task_handle);
    }
}

void ota_delete(void)
{
    if (_task_handle)
    {
        vTaskDelete(_task_handle);
    }
}
