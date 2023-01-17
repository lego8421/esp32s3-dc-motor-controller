#ifndef _TIMER_H_
#define _TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef bool (*timer_callback_t)(void);

void timer_interrupt_init(timer_callback_t callback, uint32_t frequency);
bool is_timer_interrupt_start();
void timer_interrupt_start();
void timer_interrupt_stop();

#ifdef __cplusplus
}
#endif

#endif /* _TIMER_H_ */
