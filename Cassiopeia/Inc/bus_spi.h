#ifndef BUS_SPI_H
#define BUS_SPI_H

#include <stdint.h>

int bus_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);
int bus_spi_write(const uint8_t *tx, uint16_t len);
int bus_spi_baud_slow(void);
int bus_spi_baud_fast(void);

#endif
