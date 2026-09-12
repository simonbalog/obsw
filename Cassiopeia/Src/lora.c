#include "lora.h"
#include "bus_spi.h"
#include "main.h"
#include "main.h"
#include "watchdog.h"
#include "stm32h7xx_hal.h"

#define LORA_REG_FIFO          0x00
#define LORA_REG_OP_MODE       0x01
#define LORA_REG_FRF_MSB       0x06
#define LORA_REG_FRF_MID       0x07
#define LORA_REG_FRF_LSB       0x08
#define LORA_REG_PA_CONFIG     0x09
#define LORA_REG_OCP           0x0B
#define LORA_REG_LNA           0x0C
#define LORA_REG_FIFO_ADDR_PTR 0x0D
#define LORA_REG_FIFO_TX_BASE  0x0E
#define LORA_REG_FIFO_RX_BASE  0x0F
#define LORA_REG_IRQ_FLAGS     0x12
#define LORA_REG_PACKET_SNR    0x19
#define LORA_REG_PACKET_RSSI   0x1A
#define LORA_REG_SYMB_TIMEOUT  0x1F
#define LORA_REG_MODEM_CONFIG1 0x1D
#define LORA_REG_MODEM_CONFIG2 0x1E
#define LORA_REG_MODEM_CONFIG3 0x26
#define LORA_REG_DETECT_OPT    0x31
#define LORA_REG_DETECT_THRESH 0x37
#define LORA_REG_SYNC_WORD     0x39
#define LORA_REG_PAYLOAD_LEN   0x22
#define LORA_REG_RX_NB_BYTES   0x13
#define LORA_REG_VERSION       0x42

#define LORA_MODE_SLEEP        0x00
#define LORA_MODE_LORA_SLEEP   0x80  /* LoRa + Sleep (bit7 = LongRangeMode) */
#define LORA_MODE_STDBY        0x81  /* LoRa + Standby */
#define LORA_MODE_TX           0x83
#define LORA_MODE_RX_CONT      0x85

#define LORA_FREQ_HZ 433000000UL

static int present = 0;
static int last_rssi = -127;
static unsigned int tx_ok = 0;
static unsigned int tx_fail = 0;
static int spi_error = 0;

static void nss_low(void)
{
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
}

static void nss_high(void)
{
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

static void lora_reset(void)
{
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

static uint8_t lora_reg_read(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0x00 };
    uint8_t rx[2];
    nss_low();
    if (bus_spi_transfer(tx, rx, 2) != HAL_OK)
    {
        spi_error = 1;
        nss_high();
        return 0;
    }
    nss_high();
    return rx[1];
}

static void lora_reg_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), val };
    uint8_t rx[2];
    nss_low();
    if (bus_spi_transfer(tx, rx, 2) != HAL_OK)
        spi_error = 1;
    nss_high();
}

/* Rádio zůstává trvale v RX_CONT (naslouchá i mezi polly), takže paket,
   který dorazí kdykoli, se zachytí a zpracuje při dalším uplink pollu.
   Předtím byl RX jen na kratičké okno a dlouhé LoRa pakety se nechytily. */
static void lora_enter_rx(void)
{
    lora_reg_write(LORA_REG_FIFO_ADDR_PTR, 0x00);
    lora_reg_write(LORA_REG_OP_MODE, LORA_MODE_RX_CONT);
}

static void lora_set_freq(uint32_t freq_hz)
{
    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000UL;
    lora_reg_write(LORA_REG_FRF_MSB, (frf >> 16) & 0xFF);
    lora_reg_write(LORA_REG_FRF_MID, (frf >> 8) & 0xFF);
    lora_reg_write(LORA_REG_FRF_LSB, frf & 0xFF);
}

int lora_init(void)
{
    spi_error = 0;
    lora_reset();
    nss_high();

    if (lora_reg_read(LORA_REG_VERSION) != 0x12)
        return -1;

    /* vstup do LoRa modu: LongRangeMode (bit7) se nastavuje v Sleep,
       teprve pak se prepina do Standby a pisi LoRa konfiguracni registry.
       Jinak zustava cip ve FSK modu a LoRa registry se neaktivuji. */
    lora_reg_write(LORA_REG_OP_MODE, LORA_MODE_SLEEP);
    HAL_Delay(5);
    lora_reg_write(LORA_REG_OP_MODE, LORA_MODE_LORA_SLEEP);
    HAL_Delay(5);
    lora_reg_write(LORA_REG_OP_MODE, LORA_MODE_STDBY);
    HAL_Delay(5);

    lora_set_freq(LORA_FREQ_HZ);

    lora_reg_write(LORA_REG_PA_CONFIG, 0x8C); /* PA_BOOST, +14 dBm */
    lora_reg_write(LORA_REG_OCP, 0x0B);

    lora_reg_write(LORA_REG_LNA, 0x23);

    lora_reg_write(LORA_REG_MODEM_CONFIG1, 0x72); /* BW=125kHz, CR=4/5 */
    lora_reg_write(LORA_REG_MODEM_CONFIG2, 0x84); /* SF=8, CRC on */
    lora_reg_write(LORA_REG_MODEM_CONFIG3, 0x04);

    lora_reg_write(LORA_REG_DETECT_OPT, 0x03);
    lora_reg_write(LORA_REG_DETECT_THRESH, 0x0A);

    lora_reg_write(LORA_REG_SYMB_TIMEOUT, 0x0B);

    lora_reg_write(LORA_REG_FIFO_TX_BASE, 0x00);
    lora_reg_write(LORA_REG_FIFO_RX_BASE, 0x00);

    lora_reg_write(LORA_REG_SYNC_WORD, 0x34);

    lora_enter_rx();
    if (spi_error)
        return -1;
    present = 1;
    return 0;
}

int lora_self_test(void)
{
    if (!present)
        return -1;
    return (lora_reg_read(LORA_REG_VERSION) == 0x12) ? 0 : -1;
}

int lora_send(const uint8_t *data, uint8_t len)
{
    if (!present)
        return -1;
    if (len > 0 && data == NULL)
        return -1;

    spi_error = 0;
    lora_reg_write(LORA_REG_OP_MODE, LORA_MODE_STDBY);

    /* vymazat pripadne stare IRQ flagy (RxDone z RX_CONT apod.) pred TX,
       jinak muze staty flag maskovat TxDone */
    lora_reg_write(LORA_REG_IRQ_FLAGS, 0xFF);

    lora_reg_write(LORA_REG_FIFO_ADDR_PTR, 0x00);
    lora_reg_write(LORA_REG_PAYLOAD_LEN, len);

    for (uint8_t i = 0; i < len; i++)
        lora_reg_write(LORA_REG_FIFO, data[i]);

    if (spi_error)
    {
        lora_enter_rx();
        tx_fail++;
        return -1;
    }

    lora_reg_write(LORA_REG_OP_MODE, LORA_MODE_TX);

    /* Cekani na TxDone. Horni limit 300 ms (> airtime ~100 ms u SF8/125 kHz)
       a refresh watchdogu kazdou iteraci - IWDG ma okno ~0.5 s a zaseknuty
       TX (zadny TxDone) by jinak resetoval MCU prostred pocitani/letu. */
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 300)
    {
        watchdog_refresh();
        if (lora_reg_read(LORA_REG_IRQ_FLAGS) & 0x08) /* TxDone */
        {
            lora_reg_write(LORA_REG_IRQ_FLAGS, 0x08);
            lora_enter_rx();
            if (spi_error)
            {
                tx_fail++;
                return -1;
            }
            tx_ok++;
            return 0;
        }
    }

    lora_enter_rx();
    tx_fail++;
    return -1;
}

int lora_receive(uint8_t *data, uint8_t *len, uint8_t max_len, uint8_t timeout_ms)
{
    if (!present || len == NULL || (max_len > 0 && data == NULL))
        return -1;

    /* rádio zustava trvale v RX_CONT (nastaveno v lora_init / lora_send) */

    spi_error = 0;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < timeout_ms)
    {
        uint8_t irq = lora_reg_read(LORA_REG_IRQ_FLAGS);
        if (spi_error)
            return -1;
        if (irq & 0x40) /* RxDone */
        {
            lora_reg_write(LORA_REG_OP_MODE, LORA_MODE_STDBY);
            if (spi_error)
                return -1;
            uint8_t nb = lora_reg_read(LORA_REG_RX_NB_BYTES);
            if (spi_error)
                return -1;
            if (nb > max_len)
                nb = max_len;
            lora_reg_write(LORA_REG_FIFO_ADDR_PTR, 0x00);
            for (uint8_t i = 0; i < nb; i++)
                data[i] = lora_reg_read(LORA_REG_FIFO);
            if (spi_error)
            {
                lora_enter_rx();
                return -1;
            }
            *len = nb;
            /* RSSI posledniho prijateho paketu (dBm).
               LF pasmo (433 MHz): RSSI = -164 + reg; HF (868 MHz): -157 + reg */
            last_rssi = -164 + (int)lora_reg_read(LORA_REG_PACKET_RSSI);
            lora_reg_write(LORA_REG_IRQ_FLAGS, 0x40);
            lora_enter_rx();
            if (spi_error)
                return -1;
            return 0;
        }
    }

    /* timeout - NEVYPINAM rádio, zustava naslouchat (paket prijde pozdeji) */
    return -1;
}

int lora_last_rssi(void)
{
    return last_rssi;
}

unsigned int lora_tx_ok(void)
{
    return tx_ok;
}

unsigned int lora_tx_fail(void)
{
    return tx_fail;
}
