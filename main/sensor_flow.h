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

#ifndef SENSOR_FLOW_H
#define SENSOR_FLOW_H

#include "driver/rmt.h"
#include "driver/pcnt.h"

/* @brief Initialise Flow Meter sensor task.
 * @param[in] pcnt_gpio The GPIO from which to count events.
 * @param[in] pcnt_unit The PCNT unit to use.
 * @param[in] pcnt_channel The PCNT channel to use.
 * @param[in] rmt_gpio The GPIO used by RMT to define a sampling window.
 * @param[in] rmt_channel The RMT channel to use.
 * @param[in] rmt_clk_div RMT pulse length, as a divider of the APB clock.
 * @param[in] sampling_period The duration between the start of subsequent samples, in seconds.
 * @param[in] sampling_window The duration for which the counter is enabled.
 * @param[in] filter_length The threshold of the counter to ignore short glitches, as a multiple of the APB clock period (80 MHz).
 */
void sensor_flow_init(uint8_t pcnt_gpio, pcnt_unit_t pcnt_unit, pcnt_channel_t pcnt_channel,
                      uint8_t rmt_gpio, rmt_channel_t rmt_channel, uint8_t rmt_clk_div,
                      float sampling_period, float sampling_window, uint16_t filter_length, UBaseType_t priority, QueueHandle_t publish_queue);

#endif // SENSOR_FLOW_H
