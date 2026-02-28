#ifndef ARCH_TIMER_H
#define ARCH_TIMER_H

#include <stdint.h>

typedef uint32_t tick_t;
typedef int32_t stick_t;

void delay_us(unsigned int time);
void delay_ms(unsigned int time);
void start_timeout(unsigned int usecs);
unsigned int has_timed_out(void);

#endif
