#include "bus_i2c.h"
#include "main.h"
#include "stm32h7xx_hal.h"

/* Normalni transakce z hlavni smycky - staci vyrazne nad max. dobou
   legitimniho cteni (100 kHz I2C: 18B cteni ~2 ms + BNO055 clock stretch). */
#define I2C_TIMEOUT 100

/* Transakce z TIM6 ISR stabilizace: kratky timeout, aby ISR (nejvyssi
   priorita) pri seknute I2C lince (ruiceni od serv) nikdy neblokovala
   system dele nez par ms. 5 ms pokryje legitimni 18B cteni (~2 ms) + rezervu.
   Idealne by ISR nemela na I2C cekat vubec (DMA/IT + buffer), viz TODO. */
#define I2C_TIMEOUT_ISR 5

/* Zamek I2C linky mezi hlavni smyckou a ISR stabilizace (TIM6).
 *
 * Hlavni smycka nastavi i2c_busy=1 kolem kazde transakce. ISR stabilizace
 * na zacatku vzorku kontroluje bus_i2c_busy() a pri obsazene lince vzorek
 * preskoci (dalsi prijde za 20 ms). ISR sama bezi atomicky vuci hlavni
 * smycce - ta bezi na pozadi a nemuze vstoupit doprostred transakce ISR. */
static volatile uint8_t i2c_busy = 0;

int bus_i2c_busy(void)
{
    return i2c_busy;
}

int bus_i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    i2c_busy = 1;
    int r = HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                             data, len, I2C_TIMEOUT);
    i2c_busy = 0;
    return r;
}

int bus_i2c_write_reg(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    i2c_busy = 1;
    int r = HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                              (uint8_t *)data, len, I2C_TIMEOUT);
    i2c_busy = 0;
    return r;
}

/* Varianty pro TIM6 ISR stabilizace: kratky timeout (I2C_TIMEOUT_ISR),
   aby se ISR na seknute lince neblokovala 100 ms. Jen pro tuto vetev -
   hlavni smycka pouziva normalni funkce s I2C_TIMEOUT. */
int bus_i2c_read_reg_short(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    i2c_busy = 1;
    int r = HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                             data, len, I2C_TIMEOUT_ISR);
    i2c_busy = 0;
    return r;
}

int bus_i2c_write_reg_short(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    i2c_busy = 1;
    int r = HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                              (uint8_t *)data, len, I2C_TIMEOUT_ISR);
    i2c_busy = 0;
    return r;
}

int bus_i2c_probe(uint8_t dev_addr)
{
    i2c_busy = 1;
    int r = HAL_I2C_IsDeviceReady(&hi2c1, dev_addr << 1, 5, I2C_TIMEOUT) == HAL_OK;
    i2c_busy = 0;
    return r;
}
