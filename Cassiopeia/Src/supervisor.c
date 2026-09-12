#include "supervisor.h"
#include "serial_monitor.h"
#include "alarm.h"
#include <stddef.h>
#include <string.h>
#include "bme280.h"
#include "bno055.h"
#include "pca9685.h"
#include "sd_spi.h"
#include "fatfs.h"
#include "logger.h"
#include "lora.h"
#include "power.h"
#include "gps.h"
#include "rtc.h"
#include "stm32h7xx_hal.h"
#include "status_report.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static ModuleDef modules[] =
{
    { "bme280",  bme280_init,  bme280_self_test,  MOD_STATUS_OK, 0, MASTER_BME280, 1, 0 },
    { "bno055",  bno055_init,  bno055_self_test,  MOD_STATUS_OK, 0, MASTER_IMU,    1, 0 },
    { "pca9685", pca9685_init, pca9685_self_test, MOD_STATUS_OK, 0, MASTER_SERVO,  1, 0 },
    { "lora",    lora_init,    lora_self_test,    MOD_STATUS_OK, 0, ALARM_LORA,    0, 0 },
    { "power",   power_init,   power_self_test,   MOD_STATUS_OK, 0, ALARM_BATTERY, 0, 0 },
    { "sd",      sd_spi_init,  sd_spi_self_test,  MOD_STATUS_OK, 0, ALARM_SD_CARD, 0, 0 },
    { "gps",     gps_init,     gps_self_test,     MOD_STATUS_OK, 0, -1,            0, 0 },
    { "rtc",     rtc_init,     rtc_self_test,     MOD_STATUS_OK, 0, -1,            0, 0 },
};

static void module_alarm_set(int alarm, int master)
{
    if (master)
        master_alarm_set((MasterAlarmType)alarm);
    else
        alarm_set((AlarmType)alarm);
}

static void module_alarm_clear(int alarm, int master)
{
    if (master)
        master_alarm_clear((MasterAlarmType)alarm);
    else
        alarm_clear((AlarmType)alarm);
}

void supervisor_init(void)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(modules); i++)
    {
        modules[i].status = MOD_STATUS_OK;
        modules[i].present = 0;

        if (modules[i].alarm >= 0)
            module_alarm_clear(modules[i].alarm, modules[i].master);

        if (modules[i].init && modules[i].init() == 0)
        {
            modules[i].present = 1;
        }
        else
        {
            modules[i].status = MOD_STATUS_INIT_ERR;
            if (modules[i].alarm >= 0)
                module_alarm_set(modules[i].alarm, modules[i].master);
        }
    }
}

void supervisor_self_test(void)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(modules); i++)
    {
        if (!modules[i].present)
            continue;

        if (modules[i].self_test && modules[i].self_test() != 0)
        {
            modules[i].status = MOD_STATUS_TEST_ERR;
            if (modules[i].alarm >= 0)
                module_alarm_set(modules[i].alarm, modules[i].master);
        }
    }
}

void supervisor_report(void)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(modules); i++)
    {
        serial_puts("MODULE name=");
        serial_puts(modules[i].name);
        serial_puts(" state=");

        switch (modules[i].status)
        {
        case MOD_STATUS_OK:
            serial_puts(modules[i].optional ? "OPTIONAL" : "OK");
            break;
        case MOD_STATUS_INIT_ERR:
            serial_puts("INIT_ERR");
            break;
        case MOD_STATUS_TEST_ERR:
            serial_puts("TEST_ERR");
            break;
        default:
            serial_puts("UNKNOWN");
            break;
        }

        serial_puts(" code=");
        if (modules[i].alarm >= 0)
            print_unsigned(modules[i].master ? STATUS_CODE_MASTER((unsigned int)modules[i].alarm) :
                           STATUS_CODE_ALARM((unsigned int)modules[i].alarm));
        else
            serial_puts("0");
        if (!modules[i].present) serial_puts(" present=0");
        else serial_puts(" present=1");
        serial_puts("\r\n");
    }
}

uint8_t supervisor_module_status(const char *name)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(modules); i++)
        if (strcmp(modules[i].name, name) == 0)
            return modules[i].status;
    return MOD_STATUS_INIT_ERR;
}

/* Pravidla warningu / alarmu na zaklade provoznich podminek.
 * Vola se periodicky z hlavni smycky; podminky se vyhodnocuji
 * vzdy znovu (latch = drzi dokud stav trva, po odzneni se cisti).
 *
 *   Warning:      neblokuje nic, informace.
 *   Alarm:        subsystem degradovany, start mozny ale rizikovy.
 *   Master alarm: let neni bezpecny (blokuje odpocet).
 */
/* Prahy pro 2S LiPo (7.4 V nominal): varovani 3.5 V/c, master alarm 3.3 V/c.
   S 2S by 7.2 V varovalo hned po pripojeni zateze (3.6 V/c pod sagem), proto 7000. */
#define WRN_BATTERY_LOW_MV   7000
#define MASTER_POWER_CRIT_MV 6600
#define MASTER_POWER_ENABLE  0   /* 0 = testovaci rezim (Nucleo z laptopu, divic nezapojen) */
#define WRN_TEMP_MIN_C       (-10)
#define WRN_TEMP_MAX_C       60
#define WRN_GPS_NOFIX_MS     120000   /* 2 min bez fixu po bootu */
#define WRN_GPS_LOW_SAT      4
#define WRN_LORA_TX_WINDOW   5        /* failu za sledovaci okno */
#define WRN_SD_RETRY_WINDOW  5        /* retry/fat2 failu za okno */
#define WRN_SERVO_FAIL_WINDOW 5       /* pca9685 zapis failu za okno */
#define WRN_UPDATE_PERIOD_MS 1000     /* vyhodnoceni max. 1x za sekundu */

static uint32_t warn_boot_tick = 0;
static unsigned int lora_fail_prev = 0;
static unsigned int lora_warn_count = 0;
static unsigned int sd_retry_prev = 0;
static unsigned int sd_warn_count = 0;
static unsigned int servo_fail_prev = 0;
static unsigned int servo_warn_count = 0;
static uint32_t warn_last_tick = 0;
static int rtc_synced = 0;

/* RTC bylo rucne nastaveno ze zeme (SETTIME) - GPS auto-sync se jiz
   neprepise, cas od stanice ma prednost */
void supervisor_rtc_manual_sync(void)
{
    rtc_synced = 1;
}

void supervisor_warning_update(void)
{
    uint32_t now = HAL_GetTick();
    if (warn_boot_tick == 0)
        warn_boot_tick = now;

    /* vyhodnoceni max. 1x za sekundu (I2C/ADC cteni je pomale) */
    if (now - warn_last_tick < WRN_UPDATE_PERIOD_MS)
        return;
    warn_last_tick = now;

    (void)now;

    /* --- GPS: fix --- */
    if (gps_self_test() == 0)
    {
        if (!gps_valid_fix() && (now - warn_boot_tick) > WRN_GPS_NOFIX_MS)
            warning_set(WRN_GPS_NO_FIX);
        else
            warning_clear(WRN_GPS_NO_FIX);

        uint8_t sats = 0;
        if (gps_valid_fix() && gps_get_satellites(&sats) == 0 && sats < WRN_GPS_LOW_SAT)
            warning_set(WRN_GPS_LOW_SAT);
        else
            warning_clear(WRN_GPS_LOW_SAT);

        /* RTC sync: prvni platny fix s datumem -> nastavit cas z GPS (UTC).
           DS1307 nema casovou zonu, takze se ulozi UTC 1:1 (offline hromadny
           prepočet do lokalniho casu se udela pri zpracovani logu). */
        if (!rtc_synced)
        {
            uint16_t gy = 0;
            uint8_t gmo = 0, gd = 0, gh = 0, gmi = 0, gs = 0;
            if (gps_valid_fix() && gps_get_utc_date(&gy, &gmo, &gd) == 0 &&
                gy >= 2020 && gy <= 2099 && gmo >= 1 && gmo <= 12 && gd >= 1 && gd <= 31)
            {
                gps_get_utc(&gh, &gmi, &gs);
                if (rtc_self_test() == 0 &&
                    rtc_set_datetime(gy, gmo, gd, gh, gmi, gs) == 0)
                {
                    rtc_synced = 1;
                    serial_puts("rtc: synced from GPS UTC ");
                    print_pad4((unsigned int)gy);
                    serial_puts("-");
                    print_pad2((unsigned int)gmo);
                    serial_puts("-");
                    print_pad2((unsigned int)gd);
                    serial_puts(" ");
                    print_pad2((unsigned int)gh);
                    serial_puts(":");
                    print_pad2((unsigned int)gmi);
                    serial_puts(":");
                    print_pad2((unsigned int)gs);
                    serial_puts("\r\n");
                }
            }
        }
    }
    else
    {
        warning_clear(WRN_GPS_NO_FIX);
        warning_clear(WRN_GPS_LOW_SAT);
    }

    /* --- RTC: platnost casu --- */
    uint16_t y;
    uint8_t mo, d, h, mi, s;
    if (rtc_self_test() == 0 && rtc_get_datetime(&y, &mo, &d, &h, &mi, &s) != 0)
        warning_set(WRN_RTC_INVALID);
    else
        warning_clear(WRN_RTC_INVALID);

    /* --- IMU: kalibrace (sys 3 = plne kalibrovano) --- */
    uint8_t csys = 0;
    if (bno055_self_test() == 0 &&
        bno055_calib_status(&csys) == 0 && csys < 3)
        warning_set(WRN_IMU_CAL);
    else
        warning_clear(WRN_IMU_CAL);
    if (!bno055_flight_ready())
        master_alarm_set(MASTER_IMU);
    else
        master_alarm_clear(MASTER_IMU);

    /* --- baterie: warning pod 7.2 V, master alarm pod 6.6 V --- */
    uint16_t mv = 0;
    if (power_read_battery_mv(&mv) == 0 && mv > 0)
    {
        if (mv < WRN_BATTERY_LOW_MV)
            warning_set(WRN_BATTERY_LOW);
        else
            warning_clear(WRN_BATTERY_LOW);

        if (mv < MASTER_POWER_CRIT_MV && MASTER_POWER_ENABLE)
            master_alarm_set(MASTER_POWER);
        else
            master_alarm_clear(MASTER_POWER);
    }

    /* --- teplota --- */
    float t = 0, hh = 0, p = 0;
    if (bme280_read(&t, &hh, &p) == 0)
    {
        if (t < WRN_TEMP_MIN_C || t > WRN_TEMP_MAX_C)
            warning_set(WRN_TEMP_OUT);
        else
            warning_clear(WRN_TEMP_OUT);
    }
    if (bme280_ground_pressure(0) != 0)
        master_alarm_set(MASTER_BME280);

    /* --- LoRa TX: rostouci pocet failu = warning --- */
    unsigned int fail = lora_tx_fail();
    if (fail > lora_fail_prev)
    {
        lora_warn_count++;
        lora_fail_prev = fail;
    }
    if (lora_warn_count > WRN_LORA_TX_WINDOW)
        warning_set(WRN_LORA_TX);
    else
        warning_clear(WRN_LORA_TX);

    /* --- SD: retry nebo FAT2 fallback = warning --- */
    unsigned int retries = sd_spi_retry_count() + fatfs_fat2_fallback_count() + logger_fail_count();
    if (retries > sd_retry_prev)
    {
        sd_warn_count++;
        sd_retry_prev = retries;
    }
    if (sd_warn_count > WRN_SD_RETRY_WINDOW)
        warning_set(WRN_SD_RETRY);
    else
        warning_clear(WRN_SD_RETRY);

    /* --- serva: chybne zapisy polohy = warning --- */
    unsigned int sf = pca9685_write_fail_count();
    if (sf > servo_fail_prev)
    {
        servo_warn_count++;
        servo_fail_prev = sf;
    }
    if (servo_warn_count > WRN_SERVO_FAIL_WINDOW)
        warning_set(WRN_SERVO_ERR);
    else
        warning_clear(WRN_SERVO_ERR);

    alarm_update_leds();
}
