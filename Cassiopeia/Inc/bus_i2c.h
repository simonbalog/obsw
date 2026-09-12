#ifndef BUS_I2C_H
#define BUS_I2C_H

#include <stdint.h>

int bus_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len);
int bus_i2c_write_reg(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint8_t len);
int bus_i2c_read_reg_short(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len);
int bus_i2c_write_reg_short(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint8_t len);
int bus_i2c_probe(uint8_t dev_addr);
int bus_i2c_busy(void);

#endif
