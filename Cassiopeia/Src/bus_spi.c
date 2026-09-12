#include "bus_spi.h"
#include "main.h"
#include "stm32h7xx_hal.h"

/* Kratky timeout na jedno SPI volani: normalni prenos trva jednotky ms
   (512 B @16 MHz ~0.3 ms), pri zaseknutem SPI kernelu nechceme cekat
   cely sekundu - po 100 ms se vraci HAL_TIMEOUT a SD op se da preskocit.
   (500 B pomaly init @504 kHz ~8 ms, i to je daleko pod 100 ms.) */
#define SPI_TIMEOUT 100

/* Na STM32H7 se nesmi michat HAL_SPI_Transmit a HAL_SPI_TransmitReceive
   (RXFIFO state machine se rozejde). Pouzivame vzdy TransmitReceive. */
static uint8_t spi_rx_scratch[512];

int bus_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (len == 0)
        return HAL_OK;
    if (tx == NULL || rx == NULL)
        return HAL_ERROR;
    return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, SPI_TIMEOUT);
}

int bus_spi_write(const uint8_t *tx, uint16_t len)
{
    if (len == 0)
        return HAL_OK;
    if (tx == NULL)
        return HAL_ERROR;
    if (len > sizeof(spi_rx_scratch))
        return HAL_ERROR;
    return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, spi_rx_scratch, len, SPI_TIMEOUT);
}


static int bus_spi_set_baud(uint32_t prescaler)
{
    if (HAL_SPI_DeInit(&hspi1) != HAL_OK)
        return -1;
    hspi1.Init.BaudRatePrescaler = prescaler;
    return (HAL_SPI_Init(&hspi1) == HAL_OK) ? 0 : -1;
}

/* pomaly init SD karty (<400 kHz dle spec) - kernel clock 129 MHz / 256 */
int bus_spi_baud_slow(void)
{
    return bus_spi_set_baud(SPI_BAUDRATEPRESCALER_256);
}

/* rychly provoz po initu (129 MHz / 8 = ~16 MHz) */
int bus_spi_baud_fast(void)
{
    return bus_spi_set_baud(SPI_BAUDRATEPRESCALER_8);
}
