#include "logger.h"
#include "fatfs.h"
#include "sd_spi.h"
#include "serial_monitor.h"
#include "stm32h7xx_hal.h"

/*
 * Logger na SD kartu pres FAT32 (fatfs.c).
 *
 * - kazdy radek je pridan na konec LOG.TXT ve FAT32, takze kartu
 *   lze vlozit do pocitace a log normalne otevrit
 * - logger_init() pripoji kartu a otevre LOG.TXT pri bootu; je casove
 *   ohranicen (fatfs.c, sd_spi.c) a pri neuspechu NEblokuje boot
 * - logger_update() bezi v hlavni smycce a kazde ~2 s zkousi SD znovu
 *   pripojit, takze logging zacne sam, jakmile karta odpovi (bez
 *   restartu a bez cekani "az to po par minutach nabehne")
 * - logger_log() prida radek (API zustava pro telemetrii stejne)
 */

#define LOGGER_RETRY_MS 2000

static int logger_warned = 0;
static unsigned int logger_fails = 0;
static int logger_suspect = 0;      /* karta byla v provozu, ale zapis selhal */
static uint32_t logger_last_try = 0;

static void logger_up(void)
{
    logger_warned = 0;
    logger_suspect = 0;
}

int logger_init(void)
{
    if (fatfs_init() != 0)
    {
        serial_puts("logger: FAT32 init FAIL (funguje to bez SD)\r\n");
        return -1;
    }
    logger_up();
    return 0;
}

int logger_ready(void)
{
    return fatfs_mounted() && !logger_suspect;
}

int logger_update(void)
{
    if (logger_ready())
        return 0;

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - logger_last_try) < (int32_t)LOGGER_RETRY_MS)
        return -1;
    logger_last_try = now;

    /* mrtva/kartu vypnuta karta se doda vcetne opakovaného initu */
    if (sd_spi_init() != 0)
        return -1;
    if (fatfs_init() != 0)
        return -1;

    serial_puts("logger: SD pripojena za behu\r\n");
    logger_up();
    return 0;
}

int logger_log(const char *line)
{
    if (!fatfs_mounted())
        return -1;

    if (fatfs_append_line(line) != 0)
    {
        logger_fails++;
        logger_suspect = 1;
        if (logger_warned == 0)
        {
            logger_warned = 1;
            serial_puts("logger: append FAIL (SD absent/error)\r\n");
        }
        return -1;
    }
    logger_warned = 0;
    logger_suspect = 0;
    return 0;
}

unsigned int logger_fail_count(void)
{
    return logger_fails;
}
