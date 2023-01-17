/* General Purpose Timer example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "timer_interrupt.h"

// std
#include <stdio.h>
#include <string.h>

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// esp
#include "esp_system.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "driver/gpio.h"

// app
#include "define.h"


static const char *TAG = "timer_interrupt";

//  Hardware timer clock divider
static const uint32_t TIMER_DIVIDER = 16;
static const timer_group_t timer_group = TIMER_GROUP_0;
static const timer_idx_t timer_index = TIMER_0;

bool interrupt_init = false;
bool interrupt_start = false;

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    bool high_task_awoken = false;

    if (args != NULL) {
        timer_callback_t callback = (timer_callback_t)args;
        high_task_awoken = callback();
    }

    // return whether we need to yield at the end of ISR
    return high_task_awoken;
}

bool is_timer_interrupt_start()
{
    return interrupt_start;
}

void timer_interrupt_init(timer_callback_t callback, uint32_t frequency)
{
    if (interrupt_init) {
        return;
    }

    uint64_t tick = (float)TIMER_BASE_CLK / (frequency * TIMER_DIVIDER);

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    };

    timer_init(timer_group, timer_index, &config);
    timer_set_counter_value(timer_group, timer_index, 0);
    timer_set_alarm_value(timer_group, timer_index, tick);
    timer_enable_intr(timer_group, timer_index);

    timer_isr_callback_add(timer_group, timer_index, timer_group_isr_callback, callback, 0);

    timer_interrupt_start();
}

void timer_interrupt_start()
{
    if (!interrupt_start) {
        timer_start(timer_group, timer_index);
    }
    interrupt_start = true;
}

void timer_interrupt_stop()
{
    if (interrupt_start) {
        timer_pause(timer_group, timer_index);
    }
    interrupt_start = false;
}

