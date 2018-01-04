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

#ifndef PUBLISH_H
#define PUBLISH_H

typedef enum
{
    PUBLISH_VALUE_TEMP_1 = 0,           // Temperature sensor 1 - measurement in degrees Celsius
    PUBLISH_VALUE_TEMP_2,               // Temperature sensor 2 - measurement in degrees Celsius
    PUBLISH_VALUE_TEMP_3,               // Temperature sensor 3 - measurement in degrees Celsius
    PUBLISH_VALUE_TEMP_4,               // Temperature sensor 4 - measurement in degrees Celsius
    PUBLISH_VALUE_TEMP_5,               // Temperature sensor 5 - measurement in degrees Celsius

    PUBLISH_VALUE_LIGHT_FULL_SPECTRUM,  // Ambient Light sensor - full spectrum measurement
    PUBLISH_VALUE_LIGHT_VISIBLE,        // Ambient Light sensor - visible light measurement
    PUBLISH_VALUE_LIGHT_INFRARED,       // Ambient Light sensor - infrared light measurement
    PUBLISH_VALUE_LIGHT_LUX,            // Ambient Light sensor - lux calculation

    PUBLISH_VALUE_FLOW_FREQ,            // Flow meter frequency measurement - Hz
    PUBLISH_VALUE_FLOW_RATE,            // Flow meter flow rate calculation (litres per minute)

    PUBLISH_VALUE_SWITCH_1,             // Control panel switch 1 (CP Mode) state
    PUBLISH_VALUE_SWITCH_2,             // Control panel switch 2 (CP Manual) state
    PUBLISH_VALUE_SWITCH_3,             // Control panel switch 3 (PP Mode) state
    PUBLISH_VALUE_SWITCH_4,             // Control panel switch 4 (PP Manual) state

    PUBLISH_VALUE_SSR_1,                // Solid State Relay 1 state
    PUBLISH_VALUE_SSR_2,                // Solid State Relay 2 state

    PUBLISH_VALUE_ALARM,                // Alarm (buzzer) state

    PUBLISH_VALUE_LAST,
} publish_value_id_t;

typedef struct
{
    publish_value_id_t id;
    float value;
} published_value_t;

QueueHandle_t publish_init(unsigned int queue_depth, UBaseType_t priority);

void publish_value(publish_value_id_t value_id, float value, QueueHandle_t publish_queue);

#endif // PUBLISH_H
