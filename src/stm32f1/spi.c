#include <stddef.h>
#include "config.h"
#include "spi.h"

void spi_init(spi_speed_t speed) {
  (void)speed;
  sdcard_interface_init();
}

uint8_t spi_rx_byte(void) {
  return 0xff;
}

void spi_tx_byte(uint8_t data) {
  (void)data;
}

void spi_tx_block(const uint8_t *data, uint32_t length) {
  (void)data;
  (void)length;
}

void spi_rx_block(uint8_t *data, uint32_t length) {
  while (length--) {
    *data++ = 0xff;
  }
}

void spi_set_speed(spi_speed_t speed) {
  (void)speed;
}

void spi_select_device(spi_device_t dev) {
  if (dev == SPIDEV_CARD0 || dev == SPIDEV_ALLCARDS) {
    sdcard_set_ss(0);
  } else {
    sdcard_set_ss(1);
  }
}
