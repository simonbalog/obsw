#ifndef SD_SPI_H
#define SD_SPI_H

#include <stdint.h>

int sd_spi_init(void);
int sd_spi_self_test(void);
int sd_spi_read_sector(uint32_t sector, uint8_t *buf);
int sd_spi_write_sector(uint32_t sector, const uint8_t *buf);
int sd_spi_capacity_bytes(uint64_t *bytes);
unsigned int sd_spi_retry_count(void);

#endif
