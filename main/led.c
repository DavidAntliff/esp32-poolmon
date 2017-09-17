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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static uint8_t led_gpio = 0;

void led_init(uint8_t gpio)
{
    led_gpio = gpio;
    gpio_pad_select_gpio(led_gpio);
    gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
}

void led_on(void)
{
    gpio_set_level(led_gpio, 1);
}

void led_off(void)
{
    gpio_set_level(led_gpio, 0);
}

// LED on, then off, then repeat
void led_flash(int on_ms, int off_ms, int num)
{
    for (int i = 0; i < num; ++i)
    {
        led_on();
        vTaskDelay(on_ms / portTICK_PERIOD_MS);
        led_off();
        vTaskDelay(off_ms / portTICK_PERIOD_MS);
    }
}
