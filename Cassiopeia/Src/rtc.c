#include "rtc.h"
#include "bus_i2c.h"

/*
 * RTC modul: DS1307 na modulu HW-084.
 *
 * I2C adresa 0x68. Casove registry 0x00-0x06 v BCD:
 *   0x00 sekundy  (bit7 CH = clock halt, 1 = osc. zastaven)
 *   0x01 minuty
 *   0x02 hodiny  (bit6 = 12/24h mod, 0 = 24h)
 *   0x03 den v tydnu (1-7)
 *   0x04 den v mesici
 *   0x05 mesic
 *   0x06 rok (0-99, 2000+rok)
 */

#define DS1307_ADDR  0x68
#define DS1307_SEC   0x00
#define DS1307_MIN   0x01
#define DS1307_HOUR  0x02
#define DS1307_MDAY  0x04
#define DS1307_MON   0x05
#define DS1307_YEAR  0x06

static int present = 0;

static uint8_t bcd2dec(uint8_t b)
{
    return (b >> 4) * 10 + (b & 0x0F);
}

static uint8_t dec2bcd(uint8_t d)
{
    return ((d / 10) << 4) | (d % 10);
}

int rtc_init(void)
{
    present = 0;

    if (!bus_i2c_probe(DS1307_ADDR))
        return -1;

    uint8_t sec = 0;
    if (bus_i2c_read_reg(DS1307_ADDR, DS1307_SEC, &sec, 1) != 0)
        return -1;

    /* CH bit: pokud je hodinovy oscilator zastaven (CH=1, typicke po
       vyjimuti baterie), spustit hodiny - jinak cas nebezi */
    if (sec & 0x80)
    {
        sec &= 0x7F;
        if (bus_i2c_write_reg(DS1307_ADDR, DS1307_SEC, &sec, 1) != 0)
            return -1;
    }

    /* vynutit 24h mod (bit6 v hodinach = 0), jinak je AM/PM v bitu5 */
    uint8_t hour = 0;
    if (bus_i2c_read_reg(DS1307_ADDR, DS1307_HOUR, &hour, 1) == 0 &&
        (hour & 0x40))
    {
        hour &= 0x3F; /* 12h -> 24h, zachova hodiny bez AM/PM bitu */
        if (bus_i2c_write_reg(DS1307_ADDR, DS1307_HOUR, &hour, 1) != 0)
            return -1;
    }

    present = 1;
    return 0;
}

int rtc_self_test(void)
{
    if (!present)
        return -1;
    return bus_i2c_probe(DS1307_ADDR) ? 0 : -1;
}

int rtc_get_time(uint8_t *h, uint8_t *m, uint8_t *s)
{
    uint16_t y;
    uint8_t mo, d;
    return rtc_get_datetime(&y, &mo, &d, h, m, s);
}

int rtc_get_datetime(uint16_t *y, uint8_t *mo, uint8_t *d,
                     uint8_t *h, uint8_t *mi, uint8_t *s)
{
    if (!present)
        return -1;

    uint8_t regs[7];
    if (bus_i2c_read_reg(DS1307_ADDR, DS1307_SEC, regs, 7) != 0)
        return -1;

    /* CH=1 znamena, ze hodiny nebezi - data nejsou platna */
    if (regs[0] & 0x80)
        return -1;

    if (s)  *s  = bcd2dec(regs[0] & 0x7F);
    if (mi) *mi = bcd2dec(regs[1]);
    if (h)  *h  = bcd2dec(regs[2] & 0x3F);
    if (d)  *d  = bcd2dec(regs[3 + 1]);
    if (mo) *mo = bcd2dec(regs[3 + 2]);
    if (y)  *y  = 2000 + bcd2dec(regs[3 + 3]);

    return 0;
}

int rtc_set_datetime(uint16_t y, uint8_t mo, uint8_t d,
                     uint8_t h, uint8_t mi, uint8_t s)
{
    if (y < 2000) y = 2000;
    if (y > 2099) y = 2099;

    uint8_t regs[7];
    regs[0] = dec2bcd(s);            /* sekundy, CH=0 -> hodiny bezí */
    regs[1] = dec2bcd(mi);
    regs[2] = dec2bcd(h);            /* 24h mod */
    regs[3] = 1;                     /* den v tydnu (netreba) */
    regs[4] = dec2bcd(d);
    regs[5] = dec2bcd(mo);
    regs[6] = dec2bcd((uint8_t)(y - 2000));

    if (bus_i2c_write_reg(DS1307_ADDR, DS1307_SEC, regs, 7) != 0)
        return -1;
    return 0;
}