#include "config.h"
#include "bus.h"
#include "iec-bus.h"

volatile uint8_t stm32f1_iec_state = IEC_BIT_ATN | IEC_BIT_CLOCK | IEC_BIT_DATA | IEC_BIT_SRQ;

void iec_interface_init(void) {
  stm32f1_iec_state = IEC_BIT_ATN | IEC_BIT_CLOCK | IEC_BIT_DATA | IEC_BIT_SRQ;
  iec_interrupts_init();
}

void bus_interface_init(void) __attribute__ ((weak, alias("iec_interface_init")));
