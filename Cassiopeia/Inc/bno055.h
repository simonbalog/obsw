#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>

#define BNO055_ADDR 0x28

int bno055_init(void);
int bno055_self_test(void);
int bno055_read(int16_t *acc, int16_t *gyr, int16_t *mag);
int bno055_read_isr(int16_t *acc, int16_t *gyr, int16_t *mag);  /* kratky I2C timeout, jen pro TIM6 ISR */
void bno055_diag(void);
int bno055_calib_status(uint8_t *sys);  /* sys = 0..3 (3 = plne kalibrovano) */

/* Gyro bias (auto-kalibrace pri kazdem bootu, viz orientation.c).
   bno055_read / bno055_read_isr jej automaticky odecitaji. */
void bno055_gyro_bias_reset(void);
void bno055_gyro_bias_set(const int16_t bias[3]);
int  bno055_gyro_bias_get(int16_t bias[3]);  /* 0 = kalibrovano, -1 = ne */

#endif
