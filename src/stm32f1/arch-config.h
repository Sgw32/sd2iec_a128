#ifndef ARCH_CONFIG_H
#define ARCH_CONFIG_H

#include <stdint.h>

#define _BV(x) (1U << (x))

typedef unsigned int rawbutton_t;

#define SYSTEM_TICK_HANDLER void SysTick_Handler(void)

#define HAVE_SD
#define SD_SUPPLY_VOLTAGE (1UL << 21)

#define IEC_SEPARATE_OUT
#define IEC_INPUTS_INVERTED

#define IEC_ATN_HANDLER   void EXTI15_10_IRQHandler(void)
#define IEC_CLOCK_HANDLER void EXTI9_5_IRQHandler(void)
#define IEC_TIMER_HANDLER void TIM2_IRQHandler(void)


extern volatile uint8_t stm32f1_iec_state;

#define IEC_BIT_ATN   (1U << 0)
#define IEC_BIT_CLOCK (1U << 1)
#define IEC_BIT_DATA  (1U << 2)
#define IEC_BIT_SRQ   (1U << 3)
#define IEC_INPUT stm32f1_iec_state

static inline void sdcard_interface_init(void) {
}

static inline void sdcard_set_ss(int state) {
  (void)state;
}

static inline uint8_t sdcard_detect(void) {
  return 1;
}

static inline uint8_t sdcard_wp(void) {
  return 0;
}

static inline void leds_init(void) {
}

static inline void set_busy_led(uint8_t state) {
  (void)state;
}

static inline void set_dirty_led(uint8_t state) {
  (void)state;
}

static inline void toggle_dirty_led(void) {
}

static inline rawbutton_t buttons_read(void) {
  return _BV(0);
}

#define BUTTON_NEXT _BV(0)
#define BUTTON_PREV 0

static inline uint8_t device_hw_address(void) {
  return 8;
}

static inline void iec_interrupts_init(void) {
}

static inline void set_atn(uint8_t state) {
  if (state) {
    stm32f1_iec_state &= (uint8_t)~IEC_BIT_ATN;
  } else {
    stm32f1_iec_state |= IEC_BIT_ATN;
  }
}

static inline void set_clock(uint8_t state) {
  if (state) {
    stm32f1_iec_state &= (uint8_t)~IEC_BIT_CLOCK;
  } else {
    stm32f1_iec_state |= IEC_BIT_CLOCK;
  }
}

static inline void set_data(uint8_t state) {
  if (state) {
    stm32f1_iec_state &= (uint8_t)~IEC_BIT_DATA;
  } else {
    stm32f1_iec_state |= IEC_BIT_DATA;
  }
}

static inline void set_srq(uint8_t state) {
  if (state) {
    stm32f1_iec_state &= (uint8_t)~IEC_BIT_SRQ;
  } else {
    stm32f1_iec_state |= IEC_BIT_SRQ;
  }
}

#endif
