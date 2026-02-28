/* sd2iec - UART placeholder/stub
 *
 * This file replaces the real UART implementation with no-op functions.
 * Useful to save flash when UART is not needed.
 */

#include <stdint.h>
#include "config.h"
#include "uart.h"

/* If you want uart_getc() to return something predictable, change this. */
#ifndef UART_STUB_GETC_VALUE
#define UART_STUB_GETC_VALUE 0
#endif

void uart_init(void) {
  /* no-op */
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

uint8_t uart_getc(void) {
  /* No real UART. Return a constant to keep callers happy. */
  return (uint8_t)UART_STUB_GETC_VALUE;
}

void uart_flush(void) {
  /* no-op */
}

void uart_puts_P(const char *text) {
  (void)text;
}

void uart_putcrlf(void) {
  /* no-op */
}