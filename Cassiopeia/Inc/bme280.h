#ifndef BME280_H
#define BME280_H

#include <stdint.h>

#define BME280_ADDR 0x76
#define BME280_ADDR_ALT 0x77

int bme280_init(void);
int bme280_self_test(void);
int bme280_read(float *temp_c, float *hum_pct, float *press_hpa);

#endif
