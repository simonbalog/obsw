#include "uplink.h"
#include "lora.h"
#include "flight.h"
#include "countdown.h"
#include "stabilization.h"
#include "pca9685.h"
#include "alarm.h"
#include "status_report.h"
#include "serial_monitor.h"
#include "power.h"
#include "rtc.h"
#include "gps.h"
#include "supervisor.h"
#include "telem_buf.h"
#include "fatfs.h"
#include "orientation.h"
#include "preflight.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define FIRMWARE_VERSION "0.9.2"

/* Uplink - prikazy ze zeme pres LoRa (cyberdeck / RPi).
 *
 *  Kazdych UPLINK_PERIOD_MS se poslechnou pakety a zpracuji textove prikazy:
 *
 *    ABORT   - okamzite prerusit: padak ven + stop odpoctu
 *    CHUTE   - manualni vystreleni padaku
 *    PAUSE   - pozastavit odpoctavek (jen pokud bezi)
 *    RESUME  - pokracovat v odpoctu (jen pokud je paused)
 *    START   - spustit odpoctavek (jen pokud je STOPPED)
 *    TEST    - zapnout/vypnout TEST MODE (stabilizace na zemi)
 *    ADDMIN  - +1 minuta k odpocitavanemu casu
 *    SERVO <ch> <deg>  - natoceni serva (0-4, -90..90 deg) - zemnni test
 *    SOFF    - uvolnit vsechna serva (bez pulzu, volne)
 *    SON     - vsechna serva zpet na 0 deg (drzi)
 *    STABT <ch> - testovaci stabilizace jen na servu <ch> (0-3),
 *              opakovanim/bez argumentu se vypne
 *    GYROON  - zapnout zivy stream orientace (Y;tilt=..;acc=..;dir=..)
 *              kazdych 500 ms pres LoRa (dokud se nevypne GYROOFF)
 *    GYROOFF - vypnout gyro stream (vychozi stav je OFF)
 *    GZERO   - prekalibrovat gyro/bias a referenci "nahoru" za behu
 *    STAT    - okamzite odeslat status report pres LoRa
 *    PING    - odpoved PONG (kontrola spojeni)
 *    LEDS <a> <b> <c>  - rozsvitit/zhasnout LED (0/1) - zemnni test
 *    NEUTRAL - vsechna serva do neutralu (0 deg)
 *    DUMP    - poslat cely telemetricky buffer z RAM pres LoRa
 *    CLEANRAM - vymazat telemetricky buffer v RAM (uvolnit prostor)
 *    BUF     - info o telemetrickem bufferu (pocet zaznamu, naplneni)
 *    LOGS    - vypis logu na SD (aktivni + velikosti L0-L3)
 *    LOGDEL <n> - smazat zalozni log (1=LOG1, 2=LOG2, 3=LOG3)
 *    LOGSEL <n> - ukladat do LOG<n>.TXT (0 = LOG.TXT, 1-3 = zalohy)
 *    POWER   - odeslat napeti (battery, 5V)
 *    ALARMS  - odeslat pocet alarmu a master alarmu
 *    RTC     - odeslat aktualni cas z RTC
 *    GPS     - odeslat pozici z GPS
 *    SETTIME <y> <mo> <d> <h> <mi> <s> - nastavit RTC ze zeme
 *              (POZOR: vzdy posilat UTC! GPS sync take dava UTC,
 *              RTC je tedy konzistentne v UTC)
 *    MODULES - stav modulu
 *    VER     - verze firmwaru
 *    REBOOT  - vzdaleny restart (jen kdyz nebezi odpocet/let)
 *
 *  Kazdy prikaz je potvrzen odpovedi "ACK:<PRIKAZ>" pres LoRa.
 *  Prijimane pakety se porovnavaji po prvnich znacich (case-insensitive).
 */

#define UPLINK_PERIOD_MS  100
#define UPLINK_RX_TIMEOUT 5
#define UPLINK_MAX_LEN    16

static uint32_t next_poll = 0;

void uplink_init(void)
{
    next_poll = HAL_GetTick() + UPLINK_PERIOD_MS;
}

static void uplink_reply(const char *msg)
{
    lora_send((const uint8_t *)msg, (uint8_t)strlen(msg));
}

static void uplink_ack(const char *cmd)
{
    char buf[32];
    int i = 0;
    const char *s = "ACK:";
    while (*s) buf[i++] = *s++;
    while (*cmd && i < 31) buf[i++] = *cmd++;
    buf[i] = 0;
    serial_puts("uplink: reply ");
    serial_puts(buf);
    serial_puts("\r\n");
    uplink_reply(buf);
}

static void uplink_nak(const char *cmd)
{
    char buf[32];
    int i = 0;
    const char *s = "NAK:";
    while (*s) buf[i++] = *s++;
    while (*cmd && i < 31) buf[i++] = *cmd++;
    buf[i] = 0;
    serial_puts("uplink: reply ");
    serial_puts(buf);
    serial_puts("\r\n");
    uplink_reply(buf);
}

static void uplink_abort(void)
{
    serial_puts("uplink: ABORT received\r\n");
    if (flight_state() != FLIGHT_IDLE && flight_state() != FLIGHT_LANDED)
        flight_deploy_chute();
    countdown_abort();
    master_alarm_set(MASTER_LAUNCH);
    alarm_update_leds();
    uplink_ack("ABORT");
}

static void uplink_chute(void)
{
    serial_puts("uplink: CHUTE received\r\n");
    flight_deploy_chute();
    uplink_ack("CHUTE");
}

static void uplink_pause(void)
{
    serial_puts("uplink: PAUSE received\r\n");
    countdown_remote_pause();
    uplink_ack("PAUSE");
}

static void uplink_resume(void)
{
    serial_puts("uplink: RESUME received\r\n");
    countdown_remote_resume();
    uplink_ack("RESUME");
}

static void uplink_start(void)
{
    serial_puts("uplink: START received\r\n");
    countdown_remote_start();
    uplink_ack("START");
}

static void uplink_test(void)
{
    serial_puts("uplink: TEST received\r\n");
    countdown_remote_test_toggle();
    uplink_ack("TEST");
}

static void uplink_addmin(void)
{
    serial_puts("uplink: ADDMIN received\r\n");
    countdown_remote_add_min();
    uplink_ack("ADDMIN");
}

static void uplink_stat(void)
{
    serial_puts("uplink: STAT received\r\n");
    status_report_send_lora();
    uplink_ack("STAT");
}

static void uplink_ping(void)
{
    serial_puts("uplink: PING received\r\n");
    uplink_reply("PONG");
}

/* zapise cislo bez znamenka do buf, vraci pocet znaku */
static int up_put_unsigned(char *buf, unsigned int n)
{
    int i = 0;
    unsigned int mag = 1;
    while (mag <= n / 10) mag *= 10;
    while (mag > 0)
    {
        buf[i++] = (char)('0' + (n / mag) % 10);
        mag /= 10;
    }
    return i;
}

static int up_put_int(char *buf, int n)
{
    int i = 0;
    if (n < 0)
    {
        buf[i++] = '-';
        n = -n;
    }
    i += up_put_unsigned(buf + i, (unsigned int)n);
    return i;
}

/* precte neznamenkove cislo za prefixem (preskoci prefix + mezery),
   vraci 0 a plni val pri spravnem formatu */
static int parse_uint_arg(const uint8_t *d, uint8_t len, const char *prefix, uint32_t *val)
{
    uint8_t i = 0;
    while (prefix[i] && i < len) i++;
    while (i < len && d[i] == ' ') i++;
    uint32_t v = 0;
    uint8_t digits = 0;
    while (i < len && d[i] >= '0' && d[i] <= '9')
    {
        v = v * 10 + (d[i] - '0');
        i++;
        digits++;
    }
    if (digits == 0)
        return -1;
    *val = v;
    return 0;
}

static void uplink_leds(const uint8_t *d, uint8_t len)
{
    uint8_t i = 4; /* "LEDS" */
    while (i < len && d[i] == ' ') i++;
    int vals[3] = { 0, 0, 0 };
    for (int k = 0; k < 3; k++)
    {
        uint32_t v = 0;
        uint8_t digits = 0;
        while (i < len && d[i] >= '0' && d[i] <= '9')
        {
            v = v * 10 + (d[i] - '0');
            i++;
            digits++;
        }
        if (digits == 0)
        {
            serial_puts("uplink: LEDS bad args\r\n");
            uplink_nak("LEDS");
            return;
        }
        vals[k] = (v != 0) ? 1 : 0;
        while (i < len && d[i] == ' ') i++;
    }
    serial_puts("uplink: LEDS received\r\n");
    led_set(vals[0], vals[1], vals[2]);
    uplink_ack("LEDS");
}

static void uplink_neutral(void)
{
    serial_puts("uplink: NEUTRAL received\r\n");
    pca9685_set_servo_deg(PCA9685_SERVO_STAB1, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB2, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB3, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB4, 0);
    uplink_ack("NEUTRAL");
}

/* SOFF - uvolni vsechna serva (zadne pulzy, servo lze otacet rukou) */
static void uplink_servo_off(void)
{
    serial_puts("uplink: SOFF received\r\n");
    stabilization_disengage();
    stabilization_test_channel(-1);
    for (uint8_t ch = 0; ch < PCA9685_NUM_SERVOS; ch++)
        pca9685_release(ch);
    uplink_ack("SOFF");
}

/* SON - vsechna serva opet drzi na 0 deg */
static void uplink_servo_on(void)
{
    serial_puts("uplink: SON received\r\n");
    for (uint8_t ch = 0; ch < PCA9685_NUM_SERVOS; ch++)
        pca9685_set_servo_deg(ch, 0);
    uplink_ack("SON");
}

/* STABT <ch> - testovaci stabilizace jen na jednom servu (kanal 0-3).
   Naklaneni rakety se projevi pouze na vybranem servu. Opakovani
   prikazu (bez argumentu take) test vypne a serva vrati na 0. */
static void stabtest_stop(void)
{
    stabilization_test_channel(-1);
    stabilization_disengage();
    pca9685_set_servo_deg(PCA9685_SERVO_STAB1, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB2, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB3, 0);
    pca9685_set_servo_deg(PCA9685_SERVO_STAB4, 0);
}

static void uplink_stabtest(const uint8_t *d, uint8_t len)
{
    uint8_t i = 0;
    while (i < len && d[i] != ' ') i++;   /* preskoc "STABT" */
    while (i < len && d[i] == ' ') i++;

    if (i >= len)   /* bez kanalu = vypnout */
    {
        serial_puts("uplink: STABT off\r\n");
        stabtest_stop();
        uplink_ack("STABT");
        return;
    }

    uint32_t ch = 0;
    uint8_t digits = 0;
    while (i < len && d[i] >= '0' && d[i] <= '9')
    {
        ch = ch * 10 + (d[i] - '0');
        i++;
        digits++;
    }
    if (digits == 0 || ch >= PCA9685_SERVO_PARACHUTE)
    {
        serial_puts("uplink: STABT bad args (kanal 0-3)\r\n");
        uplink_nak("STABT");
        return;
    }

    if (stabilization_test_channel_get() == (int8_t)ch && stabilization_active())
    {
        serial_puts("uplink: STABT off (was ch");
        print_unsigned(ch);
        serial_puts(")\r\n");
        stabtest_stop();
        uplink_ack("STABT");
        return;
    }

    serial_puts("uplink: STABT on ch");
    print_unsigned(ch);
    serial_puts("\r\n");
    stabilization_test_channel((int8_t)ch);
    stabilization_level_engage();
    uplink_ack("STABT");
}

static void uplink_dump(void)
{
    serial_puts("uplink: DUMP received\r\n");
    telem_buf_dump_start();
    uplink_ack("DUMP");
}

/* GYROON - operator zapne zivy stream orientace (Y; kazdych 500 ms).
   Vychozi stav je OFF, aby Y; neukousala LoRa kanal ostatni telemetrii. */
static void uplink_gyroon(void)
{
    serial_puts("uplink: GYROON received\r\n");
    orientation_gyro_stream_set(1);
    serial_puts("uplink: gyro stream ON (Y; kazdych 500 ms)\r\n");
    uplink_ack("GYROON");
}

static void uplink_gyrooff(void)
{
    serial_puts("uplink: GYROOFF received\r\n");
    orientation_gyro_stream_set(0);
    serial_puts("uplink: gyro stream OFF\r\n");
    uplink_ack("GYROOFF");
}

/* GZERO - re-kalibrace gyra za behu (bez resetu MCU): novy bias gyra +
   nova referencia "nahoru" z aktualni polohy. Raketa musi byt chvili v klidu. */
static void uplink_gzero(void)
{
    serial_puts("uplink: GZERO received\r\n");
    orientation_rezero();
    uplink_ack("GZERO");
}

static void uplink_buf(void)
{
    serial_puts("uplink: BUF received\r\n");
    char buf[64];
    int i = 0;
    const char *s = "BUF:";
    while (*s) buf[i++] = *s++;
    i += up_put_unsigned(buf + i, telem_buf_count());
    s = ";fill=";
    while (*s) buf[i++] = *s++;
    i += up_put_unsigned(buf + i, telem_buf_fill_pct());
    s = "%;ev=";
    while (*s) buf[i++] = *s++;
    i += up_put_unsigned(buf + i, telem_buf_evicted());
    buf[i] = 0;
    uplink_reply(buf);
}

static void uplink_power(void)
{
    serial_puts("uplink: POWER received\r\n");
    uint16_t batt = 0, v5 = 0;
    if (power_read_battery_mv(&batt) != 0)
        batt = 0;
    if (power_read_5v_mv(&v5) != 0)
        v5 = 0;
    char buf[64];
    int i = 0;
    const char *s = "PWR:";
    while (*s) buf[i++] = *s++;
    i += up_put_unsigned(buf + i, batt);
    s = ";5V=";
    while (*s) buf[i++] = *s++;
    i += up_put_unsigned(buf + i, v5);
    buf[i] = 0;
    uplink_reply(buf);
}

static void uplink_alarms(void)
{
    serial_puts("uplink: ALARMS received\r\n");
    char buf[64];
    int i = 0;
    const char *s = "ALM:";
    while (*s) buf[i++] = *s++;
    i += up_put_unsigned(buf + i, (unsigned int)alarm_count());
    s = ";master=";
    while (*s) buf[i++] = *s++;
    i += up_put_unsigned(buf + i, (unsigned int)master_alarm_count());
    buf[i] = 0;
    uplink_reply(buf);
}

static void uplink_rtc(void)
{
    serial_puts("uplink: RTC received\r\n");
    uint16_t y = 0;
    uint8_t mo = 0, d = 0, h = 0, mi = 0, s = 0;
    char buf[40];
    int i = 0;
    const char *p = "RTC:";
    while (*p) buf[i++] = *p++;
    if (rtc_get_datetime(&y, &mo, &d, &h, &mi, &s) == 0)
    {
        i += up_put_unsigned(buf + i, y);
        buf[i++] = '-';
        i += up_put_unsigned(buf + i, mo);
        buf[i++] = '-';
        i += up_put_unsigned(buf + i, d);
        buf[i++] = ' ';
        i += up_put_unsigned(buf + i, h);
        buf[i++] = ':';
        i += up_put_unsigned(buf + i, mi);
        buf[i++] = ':';
        i += up_put_unsigned(buf + i, s);
    }
    else
    {
        const char *na = "NA";
        while (*na) buf[i++] = *na++;
    }
    buf[i] = 0;
    uplink_reply(buf);
}

static void uplink_gps(void)
{
    serial_puts("uplink: GPS received\r\n");
    float lat = 0, lon = 0;
    char buf[40];
    int i = 0;
    const char *p = "GPS:";
    while (*p) buf[i++] = *p++;
    if (gps_get_position(&lat, &lon) == 0)
    {
        int la = (int)lat, lo = (int)lon;
        i += up_put_int(buf + i, la);
        buf[i++] = ';';
        i += up_put_int(buf + i, lo);
    }
    else
    {
        const char *na = "NOFIX";
        while (*na) buf[i++] = *na++;
    }
    buf[i] = 0;
    uplink_reply(buf);
}

/* SETTIME <y> <mo> <d> <h> <mi> <s> - nastavi RTC ze zeme (cas stanice).
   Po manualnim nastaveni se GPS auto-sync uz neprepise. */
static void uplink_settime(const uint8_t *d, uint8_t len)
{
    uint32_t v[6];
    uint8_t i = 0;
    while (i < len && d[i] != ' ') i++;   /* preskoc "SETTIME" */

    for (int k = 0; k < 6; k++)
    {
        while (i < len && d[i] == ' ') i++;
        uint32_t val = 0;
        uint8_t digits = 0;
        while (i < len && d[i] >= '0' && d[i] <= '9')
        {
            val = val * 10 + (d[i] - '0');
            i++;
            digits++;
        }
        if (digits == 0)
        {
            serial_puts("uplink: SETTIME bad args\r\n");
            uplink_nak("SETTIME");
            return;
        }
        v[k] = val;
    }

    if (v[0] < 2020 || v[0] > 2099 || v[1] < 1 || v[1] > 12 ||
        v[2] < 1 || v[2] > 31 || v[3] > 23 || v[4] > 59 || v[5] > 59)
    {
        serial_puts("uplink: SETTIME bad range\r\n");
        uplink_nak("SETTIME");
        return;
    }

    serial_puts("uplink: SETTIME received\r\n");
    if (rtc_set_datetime((uint16_t)v[0], (uint8_t)v[1], (uint8_t)v[2],
                         (uint8_t)v[3], (uint8_t)v[4], (uint8_t)v[5]) != 0)
    {
        uplink_nak("SETTIME");
        return;
    }
    supervisor_rtc_manual_sync();   /* GPS auto-sync uz neprepise */
    uplink_ack("SETTIME");
}

static void uplink_modules(void)
{
    serial_puts("uplink: MODULES received\r\n");
    char buf[128];
    int i = 0;
    const char *s = "MOD:";
    while (*s) buf[i++] = *s++;
    static const char *names[] = { "bme280", "bno055", "pca9685", "lora", "power", "gps", "rtc" };
    for (uint32_t k = 0; k < sizeof(names) / sizeof(names[0]); k++)
    {
        if (k) buf[i++] = ';';
        const char *n = names[k];
        while (*n) buf[i++] = *n++;
        buf[i++] = '=';
        uint8_t st = supervisor_module_status(names[k]);
        buf[i++] = (st == MOD_STATUS_OK) ? 'O' : 'F';
    }
    buf[i] = 0;
    uplink_reply(buf);
}

/* LOGS - vypis logu na SD kartu (aktivni + velikosti) */
static void uplink_logs(void)
{
    serial_puts("uplink: LOGS received\r\n");
    char buf[96];
    int i = 0;
    const char *s = "LOGS:akt=";
    while (*s) buf[i++] = *s++;
    uint8_t akt = fatfs_active_log();
    buf[i++] = '0' + akt;
    s = ";L0=";
    while (*s) buf[i++] = *s++;
    for (uint8_t n = 0; n <= 3; n++)
    {
        uint32_t sz = 0;
        uint8_t ex = 0;
        fatfs_log_info(n, &sz, &ex);
        if (!ex)
        {
            s = "-";
            while (*s) buf[i++] = *s++;
        }
        else
        {
            i += up_put_unsigned(buf + i, sz);
        }
        if (n < 3)
        {
            s = ";L";
            buf[i++] = 'L';
            buf[i++] = '0' + (n + 1);
            buf[i++] = '=';
        }
    }
    buf[i] = 0;
    uplink_reply(buf);
}

/* LOGDEL <n> - smaze zalozni log LOG1/2/3.TXT (n = 1..3) */
static void uplink_logdel(const uint8_t *d, uint8_t len)
{
    uint32_t n = 0;
    if (parse_uint_arg(d, len, "LOGDEL", &n) != 0 || n == 0 || n > 3)
    {
        serial_puts("uplink: LOGDEL bad args (1-3)\r\n");
        uplink_nak("LOGDEL");
        return;
    }
    serial_puts("uplink: LOGDEL received\r\n");
    if (fatfs_delete_log((uint8_t)n) != 0)
    {
        uplink_nak("LOGDEL");
        return;
    }
    uplink_ack("LOGDEL");
}

/* LOGSEL <n> - prene ukladani na LOG<n>.TXT (0 = LOG.TXT, 1-3 = zalohy) */
static void uplink_logsel(const uint8_t *d, uint8_t len)
{
    uint32_t n = 0;
    if (parse_uint_arg(d, len, "LOGSEL", &n) != 0 || n > 3)
    {
        serial_puts("uplink: LOGSEL bad args (0-3)\r\n");
        uplink_nak("LOGSEL");
        return;
    }
    serial_puts("uplink: LOGSEL received\r\n");
    if (fatfs_select_log((uint8_t)n) != 0)
    {
        uplink_nak("LOGSEL");
        return;
    }
    uplink_ack("LOGSEL");
}

/* CLEANRAM - vymaze telemetricky buffer v RAM (prostor pro dalsi let) */
static void uplink_cleanram(void)
{
    serial_puts("uplink: CLEANRAM received\r\n");
    uint32_t lines = telem_buf_count();
    uint8_t fill = (uint8_t)telem_buf_fill_pct();
    telem_buf_clear();
    serial_puts("telem_buf: cleared, was ");
    print_unsigned((unsigned int)lines);
    serial_puts(" lines / ");
    print_unsigned((unsigned int)fill);
    serial_puts(" %\r\n");
    uplink_ack("CLEANRAM");
}

/* VER - verze firmwaru (overeni, co je vlastne nahrano) */
static void uplink_ver(void)
{
    serial_puts("uplink: VER received\r\n");
    uplink_reply("VER:" FIRMWARE_VERSION);
}

/* REBOOT - vzdaleny restart Nuclea. Jen kdyz nic nebezi:
   odpocet zastaveny a let neni v prubehu. */
static void uplink_reboot(void)
{
    serial_puts("uplink: REBOOT received\r\n");

    if (countdown_state() != COUNTDOWN_STOPPED ||
        (flight_state() != FLIGHT_IDLE && flight_state() != FLIGHT_PRE_LAUNCH))
    {
        serial_puts("uplink: REBOOT refused (countdown/flight active)\r\n");
        uplink_nak("REBOOT");
        return;
    }

    uplink_ack("REBOOT");
    /* pockat, az odjde ACK pres LoRa, pak reset */
    HAL_Delay(300);
    NVIC_SystemReset();
}

static int cmd_matches(const uint8_t *d, uint8_t len, const char *cmd)
{
    while (*cmd)
    {
        if (len == 0)
            return 0;
        if (((*d) | 0x20) != ((*cmd) | 0x20))
            return 0;
        d++;
        cmd++;
        len--;
    }
    return 1;
}

/* SERVO <ch> <deg> - vrati 0 a vyplni ch/deg pri spravnem formatu */
static int parse_servo(const uint8_t *d, uint8_t len, int *ch, int *deg)
{
    uint8_t i = 0;

    while (i < len && d[i] != ' ') i++;      /* preskocit "SERVO" */
    while (i < len && d[i] == ' ') i++;      /* mezery */

    int sign = 1;
    if (i < len && (d[i] == '-' || d[i] == '+'))
    {
        if (d[i] == '-') sign = -1;
        i++;
    }
    int c = 0, digits = 0;
    while (i < len && d[i] >= '0' && d[i] <= '9')
    {
        c = c * 10 + (d[i] - '0');
        i++;
        digits++;
    }
    if (digits == 0)
        return -1;
    *ch = sign * c;

    while (i < len && d[i] == ' ') i++;

    sign = 1;
    if (i < len && (d[i] == '-' || d[i] == '+'))
    {
        if (d[i] == '-') sign = -1;
        i++;
    }
    int g = 0;
    digits = 0;
    while (i < len && d[i] >= '0' && d[i] <= '9')
    {
        g = g * 10 + (d[i] - '0');
        i++;
        digits++;
    }
    if (digits == 0)
        return -1;
    *deg = sign * g;
    return 0;
}

static void uplink_handle(const uint8_t *data, uint8_t len)
{
    if (len == 0)
        return;

    if (cmd_matches(data, len, "ABORT"))
        uplink_abort();
    else if (cmd_matches(data, len, "CHUTE"))
        uplink_chute();
    else if (cmd_matches(data, len, "PAUSE"))
        uplink_pause();
    else if (cmd_matches(data, len, "RESUME"))
        uplink_resume();
    else if (cmd_matches(data, len, "START"))
        uplink_start();
    else if (cmd_matches(data, len, "TEST"))
        uplink_test();
    else if (cmd_matches(data, len, "ADDMIN"))
        uplink_addmin();
    else if (cmd_matches(data, len, "STAT"))
        uplink_stat();
    else if (cmd_matches(data, len, "PREFLIGHT"))
    {
        serial_puts("uplink: PREFLIGHT received\r\n");
        preflight_report_lora();
        uplink_ack("PREFLIGHT");
    }
    else if (cmd_matches(data, len, "PING"))
        uplink_ping();
    else if (cmd_matches(data, len, "LEDS"))
        uplink_leds(data, len);
    else if (cmd_matches(data, len, "NEUTRAL"))
        uplink_neutral();
    else if (cmd_matches(data, len, "SOFF"))
        uplink_servo_off();
    else if (cmd_matches(data, len, "SON"))
        uplink_servo_on();
    else if (cmd_matches(data, len, "STABT"))
        uplink_stabtest(data, len);
    else if (cmd_matches(data, len, "GYROON"))
        uplink_gyroon();
    else if (cmd_matches(data, len, "GYROOFF"))
        uplink_gyrooff();
    else if (cmd_matches(data, len, "GZERO"))
        uplink_gzero();
    else if (cmd_matches(data, len, "DUMP"))
        uplink_dump();
    else if (cmd_matches(data, len, "CLEANRAM"))
        uplink_cleanram();
    else if (cmd_matches(data, len, "BUF"))
        uplink_buf();
    else if (cmd_matches(data, len, "LOGS"))
        uplink_logs();
    else if (cmd_matches(data, len, "LOGDEL"))
        uplink_logdel(data, len);
    else if (cmd_matches(data, len, "LOGSEL"))
        uplink_logsel(data, len);
    else if (cmd_matches(data, len, "POWER"))
        uplink_power();
    else if (cmd_matches(data, len, "ALARMS"))
        uplink_alarms();
    else if (cmd_matches(data, len, "RTC"))
        uplink_rtc();
    else if (cmd_matches(data, len, "GPS"))
        uplink_gps();
    else if (cmd_matches(data, len, "SETTIME"))
        uplink_settime(data, len);
    else if (cmd_matches(data, len, "MODULES"))
        uplink_modules();
    else if (cmd_matches(data, len, "VER"))
        uplink_ver();
    else if (cmd_matches(data, len, "REBOOT"))
        uplink_reboot();
    else if (cmd_matches(data, len, "SERVO"))
    {
        int ch = 0, deg = 0;
        if (parse_servo(data, len, &ch, &deg) == 0
            && ch >= 0 && ch < PCA9685_NUM_SERVOS
            && deg >= -90 && deg <= 90)
        {
            serial_puts("uplink: SERVO ");
            print_unsigned((unsigned int)ch);
            serial_puts(" ");
            print_int(deg);
            serial_puts("\r\n");
            pca9685_set_servo_deg((uint8_t)ch, (int16_t)deg);
            uplink_ack("SERVO");
        }
        else
        {
            serial_puts("uplink: SERVO bad args\r\n");
            uplink_nak("SERVO");
        }
    }
    else
    {
        serial_puts("uplink: unknown cmd\r\n");
        uplink_nak("UNKNOWN");
    }
}

void uplink_update(void)
{
    uint32_t now = HAL_GetTick();
    if (now < next_poll)
        return;
    next_poll = now + UPLINK_PERIOD_MS;

    uint8_t buf[UPLINK_MAX_LEN];
    uint8_t len = 0;
    if (serial_command_poll(buf, &len, UPLINK_MAX_LEN) != 0) {
        uplink_handle(buf, len);
        return;
    }
    if (lora_receive(buf, &len, UPLINK_MAX_LEN, UPLINK_RX_TIMEOUT) == 0)
        uplink_handle(buf, len);
}