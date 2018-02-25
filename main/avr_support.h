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

#ifndef AVR_SUPPORT_H
#define AVR_SUPPORT_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "i2c_master.h"
#include "datastore/datastore.h"

// switch states
typedef enum
{
    AVR_SWITCH_MODE_AUTO = 0,
    AVR_SWITCH_MODE_MANUAL = 1,
} avr_switch_mode_t;

typedef enum
{
    AVR_SWITCH_MANUAL_OFF = 0,
    AVR_SWITCH_MANUAL_ON = 1,
} avr_switch_manual_t;

typedef enum
{
    AVR_PUMP_STATE_OFF = 0,
    AVR_PUMP_STATE_ON = 1,
} avr_pump_state_t;

typedef enum
{
    AVR_ALARM_STATE_OFF = 0,
    AVR_ALARM_STATE_ON = 1,
} avr_alarm_state_t;

void avr_support_init(i2c_master_info_t * i2c_master_info, UBaseType_t priority, const datastore_t * datastore);

// reset the AVR
void avr_support_reset(void);

void avr_support_set_cp_pump(avr_pump_state_t state);
void avr_support_set_pp_pump(avr_pump_state_t state);
void avr_support_set_alarm(avr_alarm_state_t state);


#endif // AVR_SUPPORT_H
