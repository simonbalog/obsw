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
int bno055_flight_status(uint8_t *calib_sys, uint8_t *sys_status);
int bno055_flight_status_full(uint8_t *calib_sys, uint8_t *calib_gyr,
                              uint8_t *calib_acc, uint8_t *calib_mag,
                              uint8_t *sys_status);
int bno055_flight_ready(void);
int bno055_calibration_begin(void);
void bno055_calibration_update(void);
int bno055_calibration_active(void);
int bno055_calibration_state(void); /* 0=idle, 1=active, 2=complete, -1=failed */

/* Gyro bias (auto-kalibrace pri kazdem bootu, viz orientation.c).
   bno055_read / bno055_read_isr jej automaticky odecitaji. */
void bno055_gyro_bias_reset(void);
void bno055_gyro_bias_set(const int16_t bias[3]);
int  bno055_gyro_bias_get(int16_t bias[3]);  /* 0 = kalibrovano, -1 = ne */

#endif
