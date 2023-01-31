#ifndef DEFINE_H__
#define DEFINE_H__

#include "driver/gpio.h"

#define DEBUG_PIN_0 GPIO_NUM_40
#define DEBUG_PIN_1 GPIO_NUM_41
#define DEBUG_PIN_2 GPIO_NUM_42

// delay
#define NOP() asm volatile ("nop")

static uint32_t IRAM_ATTR micros()
{
    return (uint32_t) (esp_timer_get_time());
}

static void IRAM_ATTR delay_us(uint32_t us)
{
    uint32_t m = micros();
    if (us) {
        uint32_t e = (m + us);
        if(m > e) {
            while (micros() > e) {
                NOP();
            }
        }
        while (micros() < e) {
            NOP();
        }
    }
}

#endif // DEFINE_H__
