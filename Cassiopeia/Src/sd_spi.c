#include "sd_spi.h"
#include "bus_spi.h"
#include "main.h"
#include "main.h"
#include "serial_monitor.h"
#include "watchdog.h"
#include "stm32h7xx_hal.h"
#define SD_TIMEOUT_CMD 200

/* kratky wait na data token: karta odpovida na CMD17/24 typicky do ~1 ms,
   zasekla karta se ma vzdat rychle (doporuceny restart jednou smyckou) */
#define SD_TIMEOUT_DATA 5000

#define SD_TIMEOUT_BUSY 1000000

/* cely init karty ma tvrdy casovy strop: kdyz karta neodpovi (mrtva/
   odpojena/studena), init skonci do ~0.5 s a boot pokracuje BEZ SD.
   Bez toho by smycky CMD0 (255x) a ACMD41 (2000x) pres ~504 kHz
   drzely boot vteriny az minuty - presne to, co jsi pozoroval. */
#define SD_INIT_MAX_MS  500

/* docasny strop na programovani po zapisu: hlavni smycka nesmi na
   zamrzle karte zamrznout dele, jinak by letel (kormidla/odpocet) */
#define SD_WRITE_BUSY_MS 100

#define SD_CMD0   0x40
#define SD_CMD8   0x48
#define SD_CMD9   0x49
#define SD_CMD58  0x7A
#define SD_CMD59  0x7B
#define SD_CMD16  0x50
#define SD_CMD17  0x51
#define SD_CMD24  0x58
#define SD_CMD55  0x77
#define SD_ACMD41 0x69

static int present = 0;
static uint8_t sd_type = 0;
static unsigned int retry_total = 0;   /* pocet opakovani sektoru pres retry */

/* deadline pro beznou operaci: 0 = bez limitu (vyuziva se jen pri initu) */
static uint32_t sd_deadline = 0;

static int sd_sector_arg(uint32_t sector, uint32_t *arg)
{
    if (sd_type == 0)
    {
        if (sector > UINT32_MAX / 512U)
            return -1;
        *arg = sector * 512U;
    }
    else
        *arg = sector;
    return 0;
}

static int sd_expired(void)
{
    return sd_deadline != 0 && (int32_t)(HAL_GetTick() - sd_deadline) >= 0;
}

static void cs_high(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

static void cs_low(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void spi_dummy(uint16_t n)
{
    uint8_t ff = 0xFF;
    uint8_t rx;
    while (n--)
    {
        watchdog_refresh();   /* kazdy byte muze na mrtve karte trvat ~100 ms */
        if (bus_spi_transfer(&ff, &rx, 1) != HAL_OK)
            return;
    }
}

static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t tx[6];
    tx[0] = cmd;
    tx[1] = (arg >> 24) & 0xFF;
    tx[2] = (arg >> 16) & 0xFF;
    tx[3] = (arg >> 8) & 0xFF;
    tx[4] = arg & 0xFF;
    /* CRC: jen CMD0 a CMD8 vyzaduji spravny, zbytek 0x01 */
    tx[5] = (cmd == SD_CMD0) ? 0x95 : (cmd == SD_CMD8) ? 0x87 : 0x01;

    bus_spi_write(tx, 6);

    uint8_t r;
    uint32_t i;
    for (i = 0; i < SD_TIMEOUT_CMD && !sd_expired(); i++)
    {
        watchdog_refresh();
        uint8_t ff = 0xFF;
        if (bus_spi_transfer(&ff, &r, 1) != HAL_OK)
            return 0xFF;
        if ((r & 0x80) == 0)
            return r;
    }
    return 0xFF;
}

int sd_spi_init(void)
{
    present = 0;
    /* init musi jet pomalu - SPI kernel = PLL1_Q = 129 MHz / 256 = ~504 kHz */
    if (bus_spi_baud_slow() != 0)
        return -1;

    /* karta potřebuje alespoň 1 ms od stabilizace napájení */
    HAL_Delay(2);

    cs_high();
    spi_dummy(20); /* > 74 clku na start (20 B = 160 clku, vetsi margin), CS HIGH */

    sd_deadline = HAL_GetTick() + SD_INIT_MAX_MS;

    uint8_t r = 0xFF;
    uint32_t tries;
    for (tries = 0; tries < 255 && !sd_expired(); tries++)
    {
        cs_high();
        spi_dummy(1);   /* min. 8 clku s CS HIGH mezi pokusy */
        cs_low();
        spi_dummy(1);   /* 1 dummy byte po CS LOW pred prikazem */
        r = sd_cmd(SD_CMD0, 0);
        if (r == 0x01)
            break;
    }
    if (r != 0x01)
    {
        cs_high();
        goto fail;
    }

    /* CMD8: zjistit SD v2 */
    uint8_t resp8[4];
    r = sd_cmd(SD_CMD8, 0x1AA);
    if (r == 0x01)
    {
        uint8_t ff = 0xFF;
        for (int i = 0; i < 4; i++)
        {
            watchdog_refresh();
            bus_spi_transfer(&ff, &resp8[i], 1);
        }
        sd_type = 2;
    }
    else
    {
        sd_type = 1;
    }

    /* ACMD41 dokola az do pripravenosti karty */
    for (tries = 0; tries < 2000 && !sd_expired(); tries++)
    {
        sd_cmd(SD_CMD55, 0);
        r = sd_cmd(SD_ACMD41, 0x40000000);
        if (r == 0)
            break;
    }
    if (r != 0)
    {
        cs_high();
        goto fail;
    }

    /* CMD58: zjistit SDSC/SDHC */
    if (sd_type == 2)
    {
        r = sd_cmd(SD_CMD58, 0);
        uint8_t ocr[4];
        uint8_t ff = 0xFF;
        for (int i = 0; i < 4; i++)
        {
            watchdog_refresh();
            if (bus_spi_transfer(&ff, &ocr[i], 1) != HAL_OK)
            {
                cs_high();
                goto fail;
            }
        }
        if ((ocr[0] & 0x40) == 0)
            sd_type = 0; /* SDSC */
    }

    r = sd_cmd(SD_CMD16, 512);
    if (r != 0x00)
    {
        cs_high();
        goto fail;
    }
    cs_high();
    spi_dummy(1);

    /* init probehl - muzeme zrychlit na plnou rychlost */
    if (bus_spi_baud_fast() != 0)
        goto fail;

    sd_deadline = 0;
    present = 1;
    return 0;

fail:
    sd_deadline = 0;
    serial_puts("sd: init FAIL (card absent/broken?)\r\n");
    /* i pri neuspechu vratit plnou rychlost, jinak by zustal SPI
       pomaly a zpomalil by LoRa */
    bus_spi_baud_fast();
    return -1;
}

int sd_spi_self_test(void)
{
    return present ? 0 : -1;
}

unsigned int sd_spi_retry_count(void)
{
    return retry_total;
}

int sd_spi_capacity_bytes(uint64_t *bytes)
{
    if (!present)
        return -1;

    /* CMD9 (SEND_CSD) - R1, pak 16 B CSD + 2 B CRC */
    cs_low();
    uint8_t r = sd_cmd(SD_CMD9, 0);
    if (r != 0x00)
    {
        cs_high();
        return -1;
    }

    uint8_t csd[16];
    uint8_t ff = 0xFF;
    for (int i = 0; i < 16; i++)
        bus_spi_transfer(&ff, &csd[i], 1);
    spi_dummy(2); /* CRC */

    cs_high();
    spi_dummy(1);

    if (sd_type == 2)
    {
        /* CSD v2.0 (SDHC/SDXC): C_SIZE = 22 bitu */
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16)
                        | ((uint32_t)csd[8] << 8)
                        | csd[9];
        *bytes = (uint64_t)(c_size + 1) * 512 * 1024;
    }
    else
    {
        /* CSD v1.0 (SDSC): C_SIZE=12b, C_SIZE_MULT=3b, READ_BL_LEN=4b */
        uint32_t c_size     = ((uint32_t)(csd[6] & 0x03) << 10) | ((uint32_t)csd[7] << 2) | ((uint32_t)(csd[8] & 0xC0) >> 6);
        uint32_t c_size_mult = (uint32_t)(csd[9] & 0x03) << 1 | ((uint32_t)(csd[10] & 0x80) >> 7);
        uint32_t read_bl_len = (uint32_t)(csd[5] & 0x0F);
        *bytes = (uint64_t)(c_size + 1) * (uint64_t)(1 << (c_size_mult + 2)) * (uint64_t)(1 << read_bl_len);
    }

    return 0;
}

static uint8_t sd_wait_data(uint32_t timeout)
{
    uint8_t ff = 0xFF, r;
    for (uint32_t i = 0; i < timeout && !sd_expired(); i++)
    {
        watchdog_refresh();
        if (bus_spi_transfer(&ff, &r, 1) != HAL_OK)
            return 0xFF;
        if (r != 0xFF)
            return r;
    }
    return 0xFF;
}

int sd_spi_read_sector(uint32_t sector, uint8_t *buf)
{
    if (!present || !buf)
        return -1;
    uint32_t arg;
    if (sd_sector_arg(sector, &arg) != 0)
        return -1;

    for (int attempt = 0; attempt < 3; attempt++)
    {
        if (attempt > 0)
            retry_total++;
        watchdog_refresh();
        cs_low();
        spi_dummy(1); /* par dummy clku po CS LOW pred prikazem (jako v init) */
        uint8_t r = sd_cmd(SD_CMD17, arg);
        if (r != 0x00)
        {
            cs_high();
            if (attempt == 2)
            {
                serial_puts("sd_err: CMD17 r=");
                print_unsigned(r);
                serial_puts(" sector=");
                print_unsigned(sector);
                serial_puts("\r\n");
            }
            continue;
        }

        r = sd_wait_data(SD_TIMEOUT_DATA);
        if (r != 0xFE)
        {
            cs_high();
            if (attempt == 2)
            {
                serial_puts("sd_err: data token=");
                print_unsigned(r);
                serial_puts(" sector=");
                print_unsigned(sector);
                serial_puts("\r\n");
            }
            continue;
        }

        int ok = 1;
        for (int i = 0; i < 512; i++)
        {
            watchdog_refresh();   /* 512 x az ~100 ms = zablokovany watchdog! */
            uint8_t ff = 0xFF;
            if (bus_spi_transfer(&ff, &buf[i], 1) != HAL_OK)
            {
                ok = 0;
                break;
            }
        }
        if (!ok)
        {
            cs_high();
            continue;
        }

        uint8_t crc[2];
        spi_dummy(2);
        (void)crc;

        cs_high();
        spi_dummy(1);
        return 0;
    }
    return -1;
}

int sd_spi_write_sector(uint32_t sector, const uint8_t *buf)
{
    if (!present || !buf)
        return -1;
    uint32_t arg;
    if (sd_sector_arg(sector, &arg) != 0)
        return -1;

    for (int attempt = 0; attempt < 3; attempt++)
    {
        if (attempt > 0)
            retry_total++;
        watchdog_refresh();
        cs_low();
        spi_dummy(1); /* par dummy clku po CS LOW pred prikazem (jako v init) */
        uint8_t r = sd_cmd(SD_CMD24, arg);
        if (r != 0x00)
        {
            cs_high();
            if (attempt == 2)
            {
                serial_puts("sd_err: CMD24 r=");
                print_unsigned(r);
                serial_puts(" sector=");
                print_unsigned(sector);
                serial_puts("\r\n");
            }
            continue;
        }

        uint8_t token = 0xFE;
        watchdog_refresh();
        if (bus_spi_write(&token, 1) != HAL_OK)
        {
            cs_high();
            continue;
        }
        watchdog_refresh();
        if (bus_spi_write(buf, 512) != HAL_OK)
        {
            cs_high();
            continue;
        }
        watchdog_refresh();
        uint8_t crc[2] = { 0xFF, 0xFF };
        if (bus_spi_write(crc, 2) != HAL_OK)
        {
            cs_high();
            continue;
        }

        r = sd_wait_data(SD_TIMEOUT_DATA); /* data response token: 0x05=prijato */
        if ((r & 0x1F) != 0x05)
        {
            cs_high();
            spi_dummy(1);
            if (attempt == 2)
            {
                serial_puts("sd_err: CMD24 resp=");
                print_unsigned(r);
                serial_puts(" sector=");
                print_unsigned(sector);
                serial_puts("\r\n");
            }
            continue;
        }

        /* pockej az karta dokonci programovani (0xFF = ready).
           CS musi zustat LOW behem busy, jinak se zapis muze prerusit. */
        uint8_t ff = 0xFF;
        int busy = 1;
        uint32_t t0 = HAL_GetTick();
        for (uint32_t i = 0; i < SD_TIMEOUT_BUSY &&
               (int32_t)(HAL_GetTick() - t0) < (int32_t)SD_WRITE_BUSY_MS; i++)
        {
            watchdog_refresh();
            if (bus_spi_transfer(&ff, &ff, 1) != HAL_OK)
                break;
            if (ff == 0xFF)
            {
                busy = 0;
                break;
            }
        }
        if (!busy)
        {
            cs_high();
            spi_dummy(1);
            return 0;
        }
        cs_high();
        spi_dummy(1);
    }
    return -1;
}
