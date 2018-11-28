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

#ifndef CONTROL_H
#define CONTROL_H

#include "freertos/FreeRTOS.h"
#include "datastore/datastore.h"

typedef enum
{
    CONTROL_CP_STATE_OFF = 0,  //
    CONTROL_CP_STATE_ON,       //
} control_cp_state_t;

typedef enum
{
    CONTROL_PP_STATE_OFF = 0,   //
    CONTROL_PP_STATE_ON,        //
    CONTROL_PP_STATE_PAUSE,     //
    CONTROL_PP_STATE_EMERGENCY, //
} control_pp_state_t;

#define CONTROL_CP_SENSOR_HIGH_INSTANCE  (1)               // instance of high temperature sensor
#define CONTROL_CP_SENSOR_LOW_INSTANCE   (0)               // instance of low temperature sensor

void control_init(UBaseType_t priority, const datastore_t * datastore);
void control_delete(void);

#endif // CONTROL_H
