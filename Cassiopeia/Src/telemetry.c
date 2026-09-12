#include "telemetry.h"
#include "bme280.h"
#include "bno055.h"
#include "orientation.h"
#include "lora.h"
#include "logger.h"
#include "flight.h"
#include "rtc.h"
#include "power.h"
#include "alarm.h"
#include "serial_monitor.h"
#include "telem_buf.h"
#include "gps.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>

/* Telemetrie.
 *
 *  Kazdych TELEMETRY_PERIOD_MS posle pres LoRa slimy radek (jen dulezite
 *  udaje, kratsi vzduchem) a plny radek zapise do SD logu a RAM bufferu
 *  (pro poletovou analyzu / dump).
 *
 *  Slim format pres LoRa (klic=hodnota, kody oddelene carkou):
 *
 *    T;st=<state>;h=<alt_rel_m>;v=<batt_mV>;t=<temp_C>;
 *      master=<kody>;alarm=<kody>;warn=<kody>
 *
 *  Plny format do SD/RAM (vse, vcetne casu z RTC a senzoru):
 *
 *    T;<state>;<tick_ms>;<date>;<time>;<temp_C>;<hum_%>;<press_hPa>;
 *    <alt_rel_m>;<alt_abs_m>;<acc_x>;<acc_y>;<acc_z>;<gyr_x>;<gyr_y>;<gyr_z>;
 *    <rssi>;<master>;<alarm>;<warn>
 *
 *  state = FlightState (0=IDLE,1=PRE_LAUNCH,2=ASCENT,3=APOGEE,4=DESCENT,5=LANDED),
 *  acc_x/y/z = akcelerometr v mg (1000 mg = 1 g), gyr_x/y/z = gyroskop v dps,
 *  rssi = posledni prijaty RSSI v dBm.
 *  date/time z RTC (YYYY-MM-DD, HH:MM:SS); pokud RTC neni k dispozici,
 *  zapise se '0000-00-00' / '00:00:00'.
 *
 *  Ziva orientace pres LoRa kazdych TELEMETRY_GYRO_PERIOD_MS:
 *
 *    Y;tilt=<deg>;acc=<mg>;dir=<UP|DOWN|MISS HH H deg>
 *
 *  tilt = naklon od osy "nahoru" (0 = svisle nahoru), acc = velikost
 *  zrychleni v mg, dir = smer naklonu jako cifernikova hodina videho
 *  z ocasu rakety (12H=nahoru, viz orientation.c).
 *
 *  alt_rel_m = vyska vzhledem k tlaku pri startu (bootu) = 0 m v miste,
 *              kde raketa stoji; ukazuje kolik raketa vyletela.
 *  alt_abs_m = "normalni" absolutni barometricka vyska (ISA, 1013.25 hPa
 *              na hladine more) prepocitana z tlaku.
 */

#define TELEMETRY_PERIOD_MS 1000   /* na zemi / pred startem */
#define TELEMETRY_FAST_MS   250    /* ve letu (ASCENT/APOGEE/DESCENT) */
#define TELEMETRY_MSG_MAX   192    /* rozmer line/full ramcu - max. ~160 znaku pri vsech alarmech */
#define GPS_SERIAL_PERIOD_MS 10000   /* poloha na serial kazdych 10 s */
#define GPS_SD_PERIOD_MS     10000   /* poloha na SD kazdych 10 s */

/* ziva orientace (naklon + zrychleni) pres LoRa. SF8/125 kHz ma airtime
   ~150 ms na paket, proto perioda 500 ms (ne kraticeji). */
#define TELEMETRY_GYRO_PERIOD_MS 500

#define SEA_LEVEL_HPA 1013.25f

static uint32_t next_tick = 0;
static uint32_t gyro_next = 0;
static char line[TELEMETRY_MSG_MAX];
static float ground_press = SEA_LEVEL_HPA;

static uint32_t gps_serial_next = 0;
static uint32_t gps_sd_next = 0;
static char gps_line[96];

static void append_num(char *buf, int *i, int max, int v)
{
    if (*i >= max)
        return;
    int n = v;
    int mag = 1;
    if (n < 0)
    {
        buf[(*i)++] = '-';
        n = -n;
    }
    while (mag <= n / 10) mag *= 10;
    while (mag > 0)
    {
        if (*i >= max)
            return;
        buf[(*i)++] = (char)('0' + (n / mag) % 10);
        mag /= 10;
    }
}

static void append_pad2(char *buf, int *i, int max, int v)
{
    if (*i + 2 > max)
        return;
    if (v < 0) v = 0;
    if (v > 99) v = 99;
    buf[(*i)++] = (char)('0' + (v / 10) % 10);
    buf[(*i)++] = (char)('0' + v % 10);
}

static void append_pad4(char *buf, int *i, int max, int v)
{
    if (*i + 4 > max)
        return;
    if (v < 0) v = 0;
    if (v > 9999) v = 9999;
    buf[(*i)++] = (char)('0' + (v / 1000) % 10);
    buf[(*i)++] = (char)('0' + (v / 100) % 10);
    buf[(*i)++] = (char)('0' + (v / 10) % 10);
    buf[(*i)++] = (char)('0' + v % 10);
}

static void append_datetime(char *buf, int *i, int max)
{
    uint16_t y = 0;
    uint8_t mo = 0, d = 0, h = 0, mi = 0, s = 0;

    if (*i + 19 > max)
        return;

    if (rtc_get_datetime(&y, &mo, &d, &h, &mi, &s) == 0)
    {
        append_pad4(buf, i, max, y);
        buf[(*i)++] = '-';
        append_pad2(buf, i, max, mo);
        buf[(*i)++] = '-';
        append_pad2(buf, i, max, d);
        buf[(*i)++] = ';';
        append_pad2(buf, i, max, h);
        buf[(*i)++] = ':';
        append_pad2(buf, i, max, mi);
        buf[(*i)++] = ':';
        append_pad2(buf, i, max, s);
    }
    else if (*i + 19 <= max)
    {
        memcpy(buf + (*i), "0000-00-00;00:00:00", 19);
        (*i) += 19;
    }
}

/* aktivni kody do "n,n,n" - cislovani podle enumu v alarm.h, aby
   se pri chybe dalo v poli dohledat, co ktere cislo znamena */
static void append_codes(char *buf, int *i, int max, unsigned int mask)
{
    int first = 1;
    unsigned int m = mask;
    while (m)
    {
        unsigned int bit = m & (~m + 1); /* nejnizsi aktivni bit */
        int code = 0;
        while ((bit >> code) != 1)
            code++;
        if (!first)
        {
            if (*i >= max)
                return;
            buf[(*i)++] = ',';
        }
        first = 0;
        append_num(buf, i, max, code);
        m &= ~bit;
    }
    if (first)
    {
        if (*i < max)
            buf[(*i)++] = '0';
    }
}

static float altitude_rel(float press_hpa)
{
    /* vyska vzhledem k tlaku zmerenemu pri startu (0 m = misto bootu) */
    return 44330.0f * (1.0f - (float)pow((double)(press_hpa / ground_press), 0.1903));
}

static float altitude_abs(float press_hpa)
{
    /* absolutni barometricka vyska vuci hladine more (ISA) */
    return 44330.0f * (1.0f - (float)pow((double)(press_hpa / SEA_LEVEL_HPA), 0.1903));
}

/* slim radek pres LoRa / serial - jen dulezite udaje + alarm kody */
static void build_frame_slim(void)
{
    int i = 0;
    line[i++] = 'T';
    line[i++] = ';';
    line[i++] = 's';
    line[i++] = 't';
    line[i++] = '=';
    append_num(line, &i, (int)sizeof(line) - 4, (int)flight_state());
    line[i++] = ';';

    float t = 0, h = 0, p = 0;
    float alt_rel = 0;
    if (bme280_read(&t, &h, &p) != 0)
        t = -273;
    if (p > 100.0f)
        alt_rel = altitude_rel(p);
    else
        alt_rel = -999;

    line[i++] = 'h';
    line[i++] = '=';
    append_num(line, &i, (int)sizeof(line) - 4, (int)alt_rel);
    line[i++] = ';';

    uint16_t mv = 0;
    line[i++] = 'v';
    line[i++] = '=';
    if (power_read_battery_mv(&mv) != 0)
        mv = 0;
    append_num(line, &i, (int)sizeof(line) - 4, (int)mv);
    line[i++] = ';';

    line[i++] = 't';
    line[i++] = '=';
    append_num(line, &i, (int)sizeof(line) - 4, (int)t);
    line[i++] = ';';

    line[i++] = 'm';
    line[i++] = 'a';
    line[i++] = 's';
    line[i++] = 't';
    line[i++] = 'e';
    line[i++] = 'r';
    line[i++] = '=';
    append_codes(line, &i, (int)sizeof(line) - 4, master_alarm_mask());
    line[i++] = ';';

    line[i++] = 'a';
    line[i++] = 'l';
    line[i++] = 'a';
    line[i++] = 'r';
    line[i++] = 'm';
    line[i++] = '=';
    append_codes(line, &i, (int)sizeof(line) - 4, alarm_mask());
    line[i++] = ';';

    line[i++] = 'w';
    line[i++] = 'a';
    line[i++] = 'r';
    line[i++] = 'n';
    line[i++] = '=';
    append_codes(line, &i, (int)sizeof(line) - 4, warning_mask());

    line[i] = 0;
}

/* ziva orientace pres LoRa: naklon + zrychleni z BNO055.
 *   Y;tilt=xx;acc=yyy;dir=UP
 *   Y;tilt=xx;acc=yyy;dir=DOWN
 *   Y;tilt=xx;acc=yyy;dir=MISS 12H xx
 *   (cifernikove hodiny videho z ocasu rakety: 12H=nahoru/STAB2,
 *    3H=vpravo/STAB1, 6H=dolu/STAB4, 9H=vlevo/STAB3; xx = naklon v deg)
 * Posila se JEN kdyz operator zapnul stream (uplink prikaz GYROON);
 * jinak by ukousal LoRa kanal pro ostatni telemetrii. Pri bezici
 * kalibraci (boot / GZERO) = dir=NA. */
static void build_frame_orient(void)
{
    int i = 0;
    orientation_t o;
    int ok = (orientation_sample(&o) == 0);

    line[i++] = 'Y';
    line[i++] = ';';
    {
        const char *s = "tilt=";
        while (*s) line[i++] = *s++;
    }
    append_num(line, &i, (int)sizeof(line) - 4, ok ? o.tilt_deg : -1);
    line[i++] = ';';
    {
        const char *s = "acc=";
        while (*s) line[i++] = *s++;
    }
    append_num(line, &i, (int)sizeof(line) - 4, ok ? o.acc_mg : 0);
    line[i++] = ';';
    {
        const char *s = "dir=";
        while (*s) line[i++] = *s++;
    }

    if (!ok)
    {
        const char *s = "NA";
        while (*s) line[i++] = *s++;
    }
    else if (!o.is_miss)
    {
        const char *s = (o.dir == 'U') ? "UP" : "DOWN";
        while (*s) line[i++] = *s++;
    }
    else
    {
        const char *s = "MISS ";
        while (*s) line[i++] = *s++;
        append_num(line, &i, (int)sizeof(line) - 4, o.clock_h);
        line[i++] = 'H';
        line[i++] = ' ';
        append_num(line, &i, (int)sizeof(line) - 4, o.miss_deg);
    }
    line[i] = 0;
}

/* plny radek pro SD log a RAM buffer - vsechny udaje */
static void build_frame_full(char *buf, int max)
{
    if (max > 4)
        max -= 4;   /* rezerva na oddelovace a NUL */
    int i = 0;
    buf[i++] = 'T';
    buf[i++] = ';';

    append_num(buf, &i, max, (int)flight_state());
    buf[i++] = ';';

    append_num(buf, &i, max, (int)(HAL_GetTick() % 1000000));
    buf[i++] = ';';

    append_datetime(buf, &i, max);
    buf[i++] = ';';

    float t = 0, h = 0, p = 0;
    float alt_rel = 0, alt_abs = 0;
    if (bme280_read(&t, &h, &p) != 0)
    {
        t = -273;
        h = -1;
        p = -1;
    }
    if (p > 100.0f)
    {
        alt_rel = altitude_rel(p);
        alt_abs = altitude_abs(p);
    }
    else
    {
        alt_rel = -999;
        alt_abs = -999;
    }
    append_num(buf, &i, max, (int)t);
    buf[i++] = ';';
    append_num(buf, &i, max, (int)h);
    buf[i++] = ';';
    append_num(buf, &i, max, (int)p);
    buf[i++] = ';';
    append_num(buf, &i, max, (int)alt_rel);
    buf[i++] = ';';
    append_num(buf, &i, max, (int)alt_abs);
    buf[i++] = ';';

    int16_t acc[3], gyr[3];
    if (bno055_read(acc, gyr, 0) != 0)
    {
        acc[0] = acc[1] = acc[2] = -1;
        gyr[0] = gyr[1] = gyr[2] = -1;
    }
    append_num(buf, &i, max, acc[0]);
    buf[i++] = ';';
    append_num(buf, &i, max, acc[1]);
    buf[i++] = ';';
    append_num(buf, &i, max, acc[2]);
    buf[i++] = ';';

    append_num(buf, &i, max, gyr[0]);
    buf[i++] = ';';
    append_num(buf, &i, max, gyr[1]);
    buf[i++] = ';';
    append_num(buf, &i, max, gyr[2]);
    buf[i++] = ';';

    append_num(buf, &i, max, lora_last_rssi());
    buf[i++] = ';';

    append_codes(buf, &i, max, master_alarm_mask());
    buf[i++] = ';';
    append_codes(buf, &i, max, alarm_mask());
    buf[i++] = ';';
    append_codes(buf, &i, max, warning_mask());

    buf[i] = 0;
}

/* GPS pozice jako samostatny radek (kazdych 10 s na serial, 5 s na SD):
 *   G;lat_e6;lon_e6;sats;alt_m;nmea_lines;YYYY-MM-DD;HH:MM:SS
 *   (lat/lon ve stupnich * 1e6, fix = 0.1 m; nmea_lines = pocet prijatych
 *   NMEA vet - debug; datum/cas z RTC) */
static void build_gps_line(void)
{
    int i = 0;
    gps_line[i++] = 'G';
    gps_line[i++] = ';';

    float lat = 0, lon = 0;
    if (gps_get_position(&lat, &lon) != 0)
    {
        memcpy(gps_line + i, "0;0;0;0", 7);
        i += 7;
    }
    else
    {
        append_num(gps_line, &i, (int)sizeof(gps_line) - 4, (int)(lat * 1000000.0f));
        gps_line[i++] = ';';
        append_num(gps_line, &i, (int)sizeof(gps_line) - 4, (int)(lon * 1000000.0f));
        gps_line[i++] = ';';
        uint8_t sats = 0;
        if (gps_get_satellites(&sats) != 0)
            sats = 0;
        append_num(gps_line, &i, (int)sizeof(gps_line) - 4, (int)sats);
        gps_line[i++] = ';';
        int16_t alt = 0;
        if (gps_get_altitude(&alt) != 0)
            alt = 0;
        append_num(gps_line, &i, (int)sizeof(gps_line) - 4, (int)alt);
    }

    gps_line[i++] = ';';
    append_num(gps_line, &i, (int)sizeof(gps_line) - 4, (int)gps_nmea_lines());

    gps_line[i++] = ';';
    append_num(gps_line, &i, (int)sizeof(gps_line) - 4, (int)gps_rx_bytes());

    gps_line[i++] = ';';
    append_datetime(gps_line, &i, (int)sizeof(gps_line) - 4);

    gps_line[i] = 0;
}

void telemetry_init(void)
{
    uint32_t now = HAL_GetTick();
    next_tick = now + TELEMETRY_PERIOD_MS;
    gyro_next = now + TELEMETRY_GYRO_PERIOD_MS;
    gps_serial_next = now + GPS_SERIAL_PERIOD_MS;
    gps_sd_next = now + GPS_SD_PERIOD_MS;
    telem_buf_init();

    /* zachytit tlak v miste startu -> alt_rel bude 0 m */
    float p = 0;
    if (bme280_read(0, 0, &p) == 0 && p > 100.0f)
        ground_press = p;
}

/* perioda telemetrie podle faze letu - ve vzduchu rychleji */
static uint32_t telemetry_period(void)
{
    FlightState st = flight_state();
    if (st == FLIGHT_ASCENT || st == FLIGHT_APOGEE || st == FLIGHT_DESCENT)
        return TELEMETRY_FAST_MS;
    return TELEMETRY_PERIOD_MS;
}

void telemetry_update(void)
{
    uint32_t now = HAL_GetTick();

    /* dump bufferu pres LoRa bezi po castich - posila se 1 zaznam
       na volani, aby watchdog (0.5 s) stihal refresh mezi pakety */
    if (telem_buf_dump_active())
    {
        telem_buf_dump_step();
        return;
    }

    if (now < next_tick)
        return;
    next_tick = now + telemetry_period();

    /* GPS pozice: na serial + LoRa kazdych 10 s (recovery rakety) */
    if (now >= gps_serial_next)
    {
        gps_serial_next = now + GPS_SERIAL_PERIOD_MS;
        build_gps_line();
        serial_puts(gps_line);
        serial_puts("\r\n");
        lora_send((const uint8_t *)gps_line, (uint8_t)strlen(gps_line));
    }

    /* GPS pozice: na SD kazdych 5 s */
    if (now >= gps_sd_next)
    {
        gps_sd_next = now + GPS_SD_PERIOD_MS;
        build_gps_line();
        logger_log(gps_line);
    }

    /* ziva orientace (naklon + zrychleni) pres LoRa + serial.
       Jen kdyz operator zapnul stream GYROON, jinak by ukousal kanal. */
    if (orientation_gyro_stream_get())
    {
        if (now >= gyro_next)
        {
            gyro_next = now + TELEMETRY_GYRO_PERIOD_MS;
            build_frame_orient();
            serial_puts(line);
            serial_puts("\r\n");
            lora_send((const uint8_t *)line, (uint8_t)strlen(line));
        }
    }

    /* plny radek do SD + RAM buffer (analyza po letu) */
    char full[TELEMETRY_MSG_MAX];
    build_frame_full(full, (int)sizeof(full));
    logger_log(full);
    telem_buf_store(full);

    /* slim radek pres LoRa + serial monitor */
    build_frame_slim();
    serial_puts(line);
    serial_puts("\r\n");
    lora_send((const uint8_t *)line, (uint8_t)strlen(line));
}
