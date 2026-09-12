#include "bme280.h"
#include "bus_i2c.h"

#define BME280_CHIP_ID_REG  0xD0
#define BME280_CHIP_ID      0x60
#define BME280_CTRL_MEAS    0xF4
#define BME280_CTRL_HUM     0xF2
#define BME280_CONFIG       0xF5
#define BME280_PRESS_MSB    0xF7
#define BME280_CALIB_START  0x88   /* dig_T1..dig_P9 (24 B) */
#define BME280_CALIB_H1_REG 0xA1   /* dig_H1 (1 B) */
#define BME280_CALIB_H2_REG 0xE1   /* dig_H2..dig_H6 (7 B) */

typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
    int32_t  t_fine;
} Bme280Cal;

static int present = 0;
static uint8_t dev_addr = BME280_ADDR; /* aktivni adresa po auto-detekci */
static Bme280Cal cal;

static int bme280_read_calib(void)
{
    uint8_t c[24];
    if (bus_i2c_read_reg(dev_addr, BME280_CALIB_START, c, 24) != 0)
        return -1;

    cal.dig_T1 = (uint16_t)((uint16_t)(c[1] << 8) | c[0]);
    cal.dig_T2 = (int16_t)((uint16_t)(c[3] << 8) | c[2]);
    cal.dig_T3 = (int16_t)((uint16_t)(c[5] << 8) | c[4]);
    cal.dig_P1 = (uint16_t)((uint16_t)(c[7] << 8) | c[6]);
    cal.dig_P2 = (int16_t)((uint16_t)(c[9] << 8) | c[8]);
    cal.dig_P3 = (int16_t)((uint16_t)(c[11] << 8) | c[10]);
    cal.dig_P4 = (int16_t)((uint16_t)(c[13] << 8) | c[12]);
    cal.dig_P5 = (int16_t)((uint16_t)(c[15] << 8) | c[14]);
    cal.dig_P6 = (int16_t)((uint16_t)(c[17] << 8) | c[16]);
    cal.dig_P7 = (int16_t)((uint16_t)(c[19] << 8) | c[18]);
    cal.dig_P8 = (int16_t)((uint16_t)(c[21] << 8) | c[20]);
    cal.dig_P9 = (int16_t)((uint16_t)(c[23] << 8) | c[22]);

    if (bus_i2c_read_reg(dev_addr, BME280_CALIB_H1_REG, &cal.dig_H1, 1) != 0)
        return -1;

    uint8_t h[7];
    if (bus_i2c_read_reg(dev_addr, BME280_CALIB_H2_REG, h, 7) != 0)
        return -1;

    cal.dig_H2 = (int16_t)((uint16_t)(h[1] << 8) | h[0]);
    cal.dig_H3 = h[2];
    cal.dig_H4 = (int16_t)((uint16_t)(h[3] << 4) | (h[4] & 0x0F));
    if (cal.dig_H4 & 0x0800)
        cal.dig_H4 |= 0xF000;
    cal.dig_H5 = (int16_t)((uint16_t)((h[4] >> 4) << 8) | h[5]);
    if (cal.dig_H5 & 0x0800)
        cal.dig_H5 |= 0xF000;
    cal.dig_H6 = (int8_t)h[6];

    return 0;
}

int bme280_init(void)
{
    uint8_t id = 0;

    /* auto-detekce adresy: 0x76 (SDO=GND) nebo 0x77 (SDO=VCC) */
    if (bus_i2c_read_reg(BME280_ADDR, BME280_CHIP_ID_REG, &id, 1) != 0 || id != BME280_CHIP_ID)
    {
        id = 0;
        if (bus_i2c_read_reg(BME280_ADDR_ALT, BME280_CHIP_ID_REG, &id, 1) != 0 || id != BME280_CHIP_ID)
            return -1;
        dev_addr = BME280_ADDR_ALT;
    }
    else
    {
        dev_addr = BME280_ADDR;
    }

    if (bme280_read_calib() != 0)
        return -1;

    uint8_t cfg[2];
    cfg[0] = 0x01; /* osrs_h = 1 (x1) */
    if (bus_i2c_write_reg(dev_addr, BME280_CTRL_HUM, cfg, 1) != 0)
        return -1;

    cfg[0] = 0xB7; /* osrs_t=1, osrs_p=1, mode=normal */
    if (bus_i2c_write_reg(dev_addr, BME280_CTRL_MEAS, cfg, 1) != 0)
        return -1;

    cfg[0] = 0xA0; /* t_sb=1000ms, filter=off, SPI off */
    if (bus_i2c_write_reg(dev_addr, BME280_CONFIG, cfg, 1) != 0)
        return -1;

    present = 1;
    return 0;
}

int bme280_self_test(void)
{
    if (!present)
        return -1;
    uint8_t id = 0;
    if (bus_i2c_read_reg(dev_addr, BME280_CHIP_ID_REG, &id, 1) != 0)
        return -1;
    return (id == BME280_CHIP_ID) ? 0 : -1;
}

/* Kompenzace dle BME280 datasheetu. */

static int32_t bme280_comp_temp(int32_t adc_t)
{
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)cal.dig_T1 << 1))) * ((int32_t)cal.dig_T2)) >> 11;
    int32_t var2 = (((((adc_t >> 4) - ((int32_t)cal.dig_T1)) * ((adc_t >> 4) - ((int32_t)cal.dig_T1))) >> 12) * ((int32_t)cal.dig_T3)) >> 14;
    cal.t_fine = var1 + var2;
    return (cal.t_fine * 5 + 128) >> 8; /* 0.01 °C */
}

static uint32_t bme280_comp_press(int32_t adc_p)
{
    /* int64 kompenzace dle BME280 datasheetu, vrací Q24.8 (tlak v Pa * 256) */
    int64_t var1, var2, p;

    var1 = ((int64_t)cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)cal.dig_P6;
    var2 = var2 + ((var1 * (int64_t)cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)cal.dig_P3) >> 8) + ((var1 * (int64_t)cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)cal.dig_P1) >> 33;

    if (var1 == 0)
        return 0;

    p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)cal.dig_P7) << 4);

    return (uint32_t)p; /* Q24.8: tlak v Pa = p / 256 */
}

static uint32_t bme280_comp_hum(int32_t adc_h)
{
    int32_t v = (cal.t_fine - ((int32_t)76800));
    v = (((((adc_h << 14) - (((int32_t)cal.dig_H4) << 20) - (((int32_t)cal.dig_H5) * v)) + ((int32_t)16384)) >> 15)
         * (((((((v * ((int32_t)cal.dig_H6)) >> 10) * (((v * ((int32_t)cal.dig_H3)) >> 11) + ((int32_t)32768))) >> 10)
             + ((int32_t)2097152)) * ((int32_t)cal.dig_H2) + 8192) >> 14));
    v = (v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)cal.dig_H1)) >> 4));
    v = (v < 0 ? 0 : v);
    v = (v > 419430400 ? 419430400 : v);
    return (uint32_t)(v >> 12); /* 0.001 %RH */
}

int bme280_read(float *temp_c, float *hum_pct, float *press_hpa)
{
    if (!present)
        return -1;

    uint8_t d[8];
    if (bus_i2c_read_reg(dev_addr, BME280_PRESS_MSB, d, 8) != 0)
        return -1;

    /* Raw 20-bit values */
    int32_t raw_p = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | ((int32_t)d[2] >> 4);
    int32_t raw_t = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | ((int32_t)d[5] >> 4);
    int32_t raw_h = ((int32_t)d[6] << 8) | (int32_t)d[7];

    /* kompenzace teploty vzdy (t_fine se pouziva pro tlak i vlhkost) */
    int32_t t_comp = bme280_comp_temp(raw_t);

    if (temp_c)
        *temp_c = t_comp * 0.01f;
    if (press_hpa)
        *press_hpa = bme280_comp_press(raw_p) / 25600.0f; /* Q24.8 -> Pa (/256) -> hPa (/100) */
    if (hum_pct)
        *hum_pct = bme280_comp_hum(raw_h) * 0.001f;

    return 0;
}