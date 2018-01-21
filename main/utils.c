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

#include <time.h>
#include <sys/time.h>
#include <stdlib.h>

#include "utils.h"

static struct timeval _boot_tv = { 0 };

void init_boot_time_reference(void)
{
    gettimeofday(&_boot_tv, NULL);
}

uint32_t seconds_since_boot(void)
{
    // ignore sub-seconds
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec - _boot_tv.tv_sec;
//    printf("%ld\n", (((tv.tv_sec * 1000000 + tv.tv_usec)
//            - (tv_start.tv_sec * 1000000 + tv_start.tv_usec))));
}

// Based on https://stackoverflow.com/a/3974138/143397
// assumes little endian
char * bits_to_string(char * buffer, size_t buffer_size, void const * const ptr, size_t const size)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    size_t idx = 0;

    for (size_t i = size; i-- > 0; )
    {
        for (int j = 7 ; j >= 0; j--)
        {
            byte = (b[i] >> j) & 1;
            if (idx < buffer_size - 1)  // leave room for null terminator
            {
                buffer[idx++] = '0' + byte;
            }
        }
    }
    buffer[idx] = '\0';
    return buffer;
}
