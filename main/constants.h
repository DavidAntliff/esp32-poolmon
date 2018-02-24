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

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "sdkconfig.h"

#define VERSION "0.1"

#define SYSTEM_LEN_VERSION          4
#define SYSTEM_LEN_BUILD_DATE_TIME 16

#define PUBLISH_QUEUE_DEPTH      16

// RMT channel allocations
#define OWB_RMT_CHANNEL_RX       RMT_CHANNEL_0
#define OWB_RMT_CHANNEL_TX       RMT_CHANNEL_1
#define FLOW_METER_RMT_CHANNEL   RMT_CHANNEL_2

// Counter allocations
#define FLOW_METER_PCNT_UNIT     PCNT_UNIT_0
#define FLOW_METER_PCNT_CHANNEL  PCNT_CHANNEL_0

// Flow Meter default config - accurate to 0.1 Hz
#define FLOW_METER_RMT_CLK_DIV      160      // APB @ 80MHz / 160 = 0.5 MHz = 2us ticks
#define FLOW_METER_SAMPLING_PERIOD  (10.0)   // seconds
#define FLOW_METER_SAMPLING_WINDOW  (5.0)    // seconds
#define FLOW_METER_FILTER_LENGTH    1023     // APB @ 80MHz => limits maximum frequency to 39,100 Hz
#define FLOW_METER_MODEL_A          (0.319)  // rate (LPM) = A * x (Hz) + B
#define FLOW_METER_MODEL_B          (0.619)  // based on flow meter measurements, October 2017
#define FLOW_METER_MODEL_CUTOFF_HZ  (1.0)    // For values of x < cutoff, linear-interpolate to zero so that 0 Hz = 0 LPM

// TODO: light and temp sensor sampling periods etc.

#define POWER_CALCULATION_RATE      (10.0)   // seconds

#endif // CONSTANTS
