#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>

#define PCA9685_ADDR   0x40
#define PCA9685_NUM_SERVOS 5

/* Servo kanaly dle zapojeni */
#define PCA9685_SERVO_STAB1  0
#define PCA9685_SERVO_STAB2  1
#define PCA9685_SERVO_STAB3  2
#define PCA9685_SERVO_STAB4  3
#define PCA9685_SERVO_PARACHUTE 4

int pca9685_init(void);
int pca9685_self_test(void);
int pca9685_set_servo(uint8_t channel, uint16_t pulse_us);
int pca9685_set_servo_deg(uint8_t channel, int16_t deg);
int pca9685_set_servo_deg_isr(uint8_t channel, int16_t deg);  /* kratky I2C timeout, jen pro TIM6 ISR */
int pca9685_release(uint8_t channel);  /* servo bez pulzu (volne) */
unsigned int pca9685_write_fail_count(void);

#endif
