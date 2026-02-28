#include "config.h"
#include "timer.h"

static volatile tick_t timeout_target;
static volatile uint8_t timeout_active;

void timer_init(void) {
  timeout_active = 0;
}

void delay_us(unsigned int time) {
  volatile unsigned int i;
  for (i = 0; i < (time * 8U); i++) {
    __asm__ volatile("nop");
  }
}

void delay_ms(unsigned int time) {
  while (time--) {
    delay_us(1000);
  }
}

void start_timeout(unsigned int usecs) {
  timeout_target = ticks + (usecs / 10U) + 1U;
  timeout_active = 1;
}

unsigned int has_timed_out(void) {
  if (!timeout_active) {
    return 1;
  }

  if (time_after(ticks, timeout_target)) {
    timeout_active = 0;
    return 1;
  }

  return 0;
}
