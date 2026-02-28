#include "config.h"
#include "uart.h"

void uart_init(void) {
}

unsigned char uart_getc(void) {
  return 0;
}

void uart_putc(char c) {
  (void)c;
}

void uart_puthex(uint8_t num) {
  (void)num;
}

void uart_trace(void *ptr, uint16_t start, uint16_t len) {
  (void)ptr;
  (void)start;
  (void)len;
}

void uart_flush(void) {
}

void uart_puts_P(const char *text) {
  (void)text;
}

void uart_putcrlf(void) {
}
