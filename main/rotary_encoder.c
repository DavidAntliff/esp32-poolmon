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

#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "rotary_encoder.h"

#define TAG "rotary_encoder"

typedef struct
{
    QueueHandle_t input_queue;
    gpio_num_t gpio_a;
    gpio_num_t gpio_b;
} task_inputs_t;

static TaskHandle_t _task_handle = NULL;

// if defined, show direct state of Rotary Encoder pins
//#define DIRECT 1

// Values returned by 'process'
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

typedef struct {
    uint8_t state;
    uint8_t pin_a;
    uint8_t pin_b;
} rotary_encoder_t;

// Based on https://github.com/buxtronix/arduino/tree/master/libraries/Rotary
// Original header:

/* Rotary encoder handler for arduino. v1.1
 *
 * Copyright 2011 Ben Buxton. Licenced under the GNU GPL Version 3.
 * Contact: bb@cactii.net
 *
 * A typical mechanical rotary encoder emits a two bit gray code
 * on 3 output pins. Every step in the output (often accompanied
 * by a physical 'click') generates a specific sequence of output
 * codes on the pins.
 *
 * There are 3 pins used for the rotary encoding - one common and
 * two 'bit' pins.
 *
 * The following is the typical sequence of code on the output when
 * moving from one step to the next:
 *
 *   Position   Bit1   Bit2
 *   ----------------------
 *     Step1     0      0
 *      1/4      1      0
 *      1/2      1      1
 *      3/4      0      1
 *     Step2     0      0
 *
 * From this table, we can see that when moving from one 'click' to
 * the next, there are 4 changes in the output code.
 *
 * - From an initial 0 - 0, Bit1 goes high, Bit0 stays low.
 * - Then both bits are high, halfway through the step.
 * - Then Bit1 goes low, but Bit2 stays high.
 * - Finally at the end of the step, both bits return to 0.
 *
 * Detecting the direction is easy - the table simply goes in the other
 * direction (read up instead of down).
 *
 * To decode this, we use a simple state machine. Every time the output
 * code changes, it follows state, until finally a full steps worth of
 * code is received (in the correct order). At the final 0-0, it returns
 * a value indicating a step in one direction or the other.
 *
 * It's also possible to use 'half-step' mode. This just emits an event
 * at both the 0-0 and 1-1 positions. This might be useful for some
 * encoders where you want to detect all positions.
 *
 * If an invalid state happens (for example we go from '0-1' straight
 * to '1-0'), the state machine resets to the start until 0-0 and the
 * next valid codes occur.
 *
 * The biggest advantage of using a state machine over other algorithms
 * is that this has inherent debounce built in. Other algorithms emit spurious
 * output with switch bounce, but this one will simply flip between
 * sub-states until the bounce settles, then continue along the state
 * machine.
 * A side effect of debounce is that fast rotations can cause steps to
 * be skipped. By not requiring debounce, fast rotations can be accurately
 * measured.
 * Another advantage is the ability to properly handle bad state, such
 * as due to EMI, etc.
 * It is also a lot simpler than others - a static state table and less
 * than 10 lines of logic.
 */

/*
 * The below state table has, for each state (row), the new state
 * to set based on the next encoder output. From left to right in,
 * the table, the encoder outputs are 00, 01, 10, 11, and the value
 * in that position is the new state to set.
 */

#define R_START 0x0

#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
#  define R_CCW_BEGIN 0x1
#  define R_CW_BEGIN 0x2
#  define R_START_M 0x3
#  define R_CW_BEGIN_M 0x4
#  define R_CCW_BEGIN_M 0x5
const uint8_t ttable[6][4] = {
    // R_START (00)
    {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
    // R_CCW_BEGIN
    {R_START_M | DIR_CCW,  R_START,        R_CCW_BEGIN,  R_START},
    // R_CW_BEGIN
    {R_START_M | DIR_CW,   R_CW_BEGIN,     R_START,      R_START},
    // R_START_M (11)
    {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
    // R_CW_BEGIN_M
    {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
    // R_CCW_BEGIN_M
    {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};
#else
// Use the full-step state table (emits a code at 00 only)
#  define R_CW_FINAL  0x1
#  define R_CW_BEGIN  0x2
#  define R_CW_NEXT   0x3
#  define R_CCW_BEGIN 0x4
#  define R_CCW_FINAL 0x5
#  define R_CCW_NEXT  0x6

const uint8_t ttable[7][4] = {
    // 00        01           10           11                  // BA
    {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},           // R_START
    {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},  // R_CW_FINAL
    {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},           // R_CW_BEGIN
    {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},           // R_CW_NEXT
    {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},           // R_CCW_BEGIN
    {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW}, // R_CCW_FINAL
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},           // R_CCW_NEXT
};
#endif

// GPIO direction and pull-ups should be set externally
static rotary_encoder_t rotary_init(gpio_num_t pin_a, gpio_num_t pin_b)
{
    rotary_encoder_t rotenc;
    rotenc.pin_a = pin_a;
    rotenc.pin_b = pin_b;
    rotenc.state = R_START;
    return rotenc;
}

// handle inverted pins
static int get_pin(gpio_num_t gpio)
{
    return gpio_get_level(gpio);
//    return 1 - gpio_get_level(gpio);
}

uint8_t rotary_process(rotary_encoder_t * rotenc)
{
    uint8_t event = 0;
    if (rotenc != NULL)
    {
        // Get state of input pins.
        uint8_t pin_state = (get_pin(rotenc->pin_b) << 1) | get_pin(rotenc->pin_a);
        // Determine new state from the pins and state table.
        rotenc->state = ttable[rotenc->state & 0xf][pin_state];
        // Return emit bits, i.e. the generated event.
        event = rotenc->state & 0x30;
    }
    return event;
}

typedef struct
{
    rotary_encoder_t rotenc;
    int32_t position;
} knob_with_reset_t;

#ifdef DIRECT
static void isr_rotenc_a(void * args)
{
    gpio_num_t * gpio = (gpio *)args;
    int level = gpio_get_level(*gpio);
    ESP_EARLY_LOGI(TAG, "A%d", level);
}

static void isr_rotenc_b(void * args)
{
    gpio_num_t * gpio = (gpio *)args;
    int level = gpio_get_level(*gpio);
    ESP_EARLY_LOGI(TAG, "B%d", level);
}
#else  // !DIRECT
static QueueHandle_t q1;
static void isr_rotenc_process(void * args)
{
    knob_with_reset_t * knob = (knob_with_reset_t *)args;
    uint8_t event = rotary_process(&knob->rotenc);
    switch (event)
    {
        case DIR_CW:
            ++knob->position;
            //ESP_EARLY_LOGI(TAG, "%d", knob->position);
            xQueueSendToBackFromISR(q1, &knob->position, NULL);
            break;
        case DIR_CCW:
            --knob->position;
            //ESP_EARLY_LOGI(TAG, "%d", knob->position);
            xQueueSendToBackFromISR(q1, &knob->position, NULL);
            break;
        default:
            break;
    }
}
#endif  // DIRECT

static void rotary_encoder_task(void * pvParameter)
{
    assert(pvParameter);
    ESP_LOGI(TAG, "Core ID %d", xPortGetCoreID());

    task_inputs_t * task_inputs = (task_inputs_t *)pvParameter;
    QueueHandle_t input_queue = task_inputs->input_queue;
    gpio_num_t gpio_a = task_inputs->gpio_a;
    gpio_num_t gpio_b = task_inputs->gpio_b;

    gpio_pad_select_gpio(gpio_a);
    gpio_set_pull_mode(gpio_a, GPIO_PULLUP_ONLY);
    gpio_set_direction(gpio_a, GPIO_MODE_INPUT);
    gpio_set_intr_type(gpio_a, GPIO_INTR_ANYEDGE);

    gpio_pad_select_gpio(gpio_b);
    gpio_set_pull_mode(gpio_b, GPIO_PULLUP_ONLY);
    gpio_set_direction(gpio_b, GPIO_MODE_INPUT);
    gpio_set_intr_type(gpio_b, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);
#ifdef DIRECT
    gpio_isr_handler_add(gpio_a, isr_rotenc_a, &gpio_a);
    gpio_isr_handler_add(gpio_b, isr_rotenc_b, &gpio_b);
#else  // DIRECT
    knob_with_reset_t knob = { rotary_init(gpio_a, gpio_b), 0 };
    gpio_isr_handler_add(gpio_a, isr_rotenc_process, &knob);
    gpio_isr_handler_add(gpio_b, isr_rotenc_process, &knob);
#endif  // DIRECT

    int32_t old_value = 0;

    q1 = xQueueCreate(10, sizeof(((knob_with_reset_t *)0)->position));

    while(1)
    {
        int32_t value = 0;
        BaseType_t rc = xQueueReceive(q1, &value, portMAX_DELAY);
        if (rc == pdTRUE)
        {
            ESP_LOGD(TAG, "position %d", value);
            rotary_encoder_event_t event = 0;
            if (value > old_value)
            {
                event = ROTARY_ENCODER_EVENT_CLOCKWISE;
            }
            else if (value < old_value)
            {
                event = ROTARY_ENCODER_EVENT_COUNTER_CLOCKWISE;
            }
            else
            {
                ESP_LOGE(TAG, "encoder position did not change! %d", value);
            }
            old_value = value;

            if (event)
            {
                if (xQueueSendToBack(input_queue, &event, 0) != pdTRUE)
                {
                    ESP_LOGE(TAG, "xQueueSendToBack failed");
                }
            }
        }
    }

    free(task_inputs);
    _task_handle = NULL;
    vTaskDelete(NULL);
}

void rotary_encoder_init(UBaseType_t priority, QueueHandle_t input_queue, gpio_num_t gpio_a, gpio_num_t gpio_b)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // task will take ownership of this struct
    task_inputs_t * task_inputs = malloc(sizeof(*task_inputs));
    if (task_inputs)
    {
        memset(task_inputs, 0, sizeof(*task_inputs));
        task_inputs->input_queue = input_queue;
        task_inputs->gpio_a = gpio_a;
        task_inputs->gpio_b = gpio_b;
        xTaskCreate(&rotary_encoder_task, "rotary_encoder_task", 2048, task_inputs, priority, &_task_handle);
    }
}

void rotary_encoder_delete(void)
{
    if (_task_handle)
        vTaskDelete(_task_handle);
}
