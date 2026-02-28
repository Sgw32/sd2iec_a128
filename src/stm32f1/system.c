#include "config.h"
#include "system.h"

static inline void cpu_relax(void) {
  __asm__ volatile("nop");
}

void system_init_early(void) {
}

void system_init_late(void) {
}

void system_sleep(void) {
  cpu_relax();
}

void system_reset(void) {
  while (1) {
    cpu_relax();
  }
}

void disable_interrupts(void) {
  __asm__ volatile("cpsid i" : : : "memory");
}

void enable_interrupts(void) {
  __asm__ volatile("cpsie i" : : : "memory");
}
