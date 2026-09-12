#include "pca9685.h"
#include "bus_i2c.h"
#include "stm32h7xx_hal.h"


#define PCA9685_MODE1       0x00
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06

#define PCA9685_OSC_FREQ    25000000UL
#define PCA9685_SERVO_FREQ  50
#define PCA9685_PULSE_RES   4096

#define SERVO_MIN_US  1000
#define SERVO_MAX_US  2000

static int present = 0;
static unsigned int write_fails = 0;   /* pocet selhanych zapisu serv */

int pca9685_init(void)
{
    uint8_t v;

    /* sleep mode */
    v = 0x10;
    if (bus_i2c_write_reg(PCA9685_ADDR, PCA9685_MODE1, &v, 1) != 0)
        return -1;

    /* PCA9685 vyzaduje po nastaveni SLEEP >= 500 us pred zapisem
       PRESCALE (start interniho oscilatoru) */
    HAL_Delay(1);

    /* prescale pro 50 Hz: round(25e6 / (4096 * 50)) - 1 = 121 */
    uint32_t prescale = (PCA9685_OSC_FREQ / (PCA9685_PULSE_RES * PCA9685_SERVO_FREQ)) - 1;
    v = (uint8_t)prescale;
    if (bus_i2c_write_reg(PCA9685_ADDR, PCA9685_PRESCALE, &v, 1) != 0)
        return -1;

    /* auto-increment + restart + normal mode (wake) */
    v = 0x20 | 0x01 | 0x80;
    if (bus_i2c_write_reg(PCA9685_ADDR, PCA9685_MODE1, &v, 1) != 0)
        return -1;

    /* po probuzeni pockat na start oscilatoru, pak lze psat PWM */
    HAL_Delay(1);

    present = 1;

    /* vsechna serva do neutrallu */
    for (uint8_t ch = 0; ch < PCA9685_NUM_SERVOS; ch++)
        pca9685_set_servo(ch, 1500);

    return 0;
}

int pca9685_self_test(void)
{
    if (!present)
        return -1;
    return bus_i2c_probe(PCA9685_ADDR) ? 0 : -1;
}

/* isr != 0 = volano z TIM6 ISR stabilizace -> kratky I2C timeout
   (bus_i2c_write_reg_short), aby se ISR neblokovala na seknute lince. */
static int pca9685_set_servo_internal(uint8_t channel, uint16_t pulse_us, int isr)
{
    if (!present)
        return -1;
    if (channel >= PCA9685_NUM_SERVOS)
        return -1;

    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;

    /* jeden tick = 1 / (25e6/4096) s = 163.84 us / 4096 = 4.096e-6 s
       pulse * 4096 / 1000000 us * 25e6... vzorec: count = pulse_us * 4096 / (1e6 / 50) */
    uint16_t on = 0;
    uint16_t off = (uint16_t)((uint32_t)pulse_us * PCA9685_PULSE_RES * PCA9685_SERVO_FREQ / 1000000UL);

    uint8_t reg = PCA9685_LED0_ON_L + channel * 4;
    uint8_t buf[4];
    buf[0] = on & 0xFF;
    buf[1] = (on >> 8) & 0xFF;
    buf[2] = off & 0xFF;
    buf[3] = (off >> 8) & 0xFF;

    int r = isr ? bus_i2c_write_reg_short(PCA9685_ADDR, reg, buf, 4)
                : bus_i2c_write_reg(PCA9685_ADDR, reg, buf, 4);
    if (r != 0)
    {
        write_fails++;
        return -1;
    }
    return 0;
}

int pca9685_set_servo(uint8_t channel, uint16_t pulse_us)
{
    return pca9685_set_servo_internal(channel, pulse_us, 0);
}

int pca9685_set_servo_deg_isr(uint8_t channel, int16_t deg)
{
    if (deg < -90) deg = -90;
    if (deg > 90) deg = 90;
    uint16_t pulse = (uint16_t)(1500 + (int32_t)deg * 500 / 90);
    return pca9685_set_servo_internal(channel, pulse, 1);
}

int pca9685_release(uint8_t channel)
{
    if (!present)
        return -1;
    if (channel >= PCA9685_NUM_SERVOS)
        return -1;

    /* full-OFF bit (bit4 v OFF_H) = vystup trvale v nule, zadne pulzy.
       Servo je volne (bez holding momentu), dalsi zapis pres
       pca9685_set_servo() bit automaticky smaze. */
    uint8_t reg = PCA9685_LED0_ON_L + channel * 4 + 3; /* OFF_H */
    uint8_t v = 0x10;
    if (bus_i2c_write_reg(PCA9685_ADDR, reg, &v, 1) != 0)
    {
        write_fails++;
        return -1;
    }
    return 0;
}

unsigned int pca9685_write_fail_count(void)
{
    return write_fails;
}

int pca9685_set_servo_deg(uint8_t channel, int16_t deg)
{
    if (deg < -90) deg = -90;
    if (deg > 90) deg = 90;
    uint16_t pulse = (uint16_t)(1500 + (int32_t)deg * 500 / 90);
    return pca9685_set_servo(channel, pulse);
}
