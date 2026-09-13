#include "status_report.h"
#include "sd_spi.h"
#include "logger.h"
#include "serial_monitor.h"
#include "countdown.h"
#include "flight.h"
#include "alarm.h"
#include "safety.h"
#include "power.h"
#include "lora.h"
#include "telem_buf.h"
#include "main.h"
#include "bme280.h"
#include "bno055.h"
#include "orientation.h"
#include "stabilization.h"
#include "pca9685.h"
#include "preflight.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <string.h>

#define STATUS_REPORT_PERIOD_MS 2000U
#define STATUS_LINE_MAX 240U

static uint32_t next_tick;
static char sline[STATUS_LINE_MAX];

static void putc_bounded(unsigned int *pos, char c)
{
    if (*pos + 1U < STATUS_LINE_MAX)
        sline[(*pos)++] = c;
}

static void puts_bounded(unsigned int *pos, const char *s)
{
    if (!s) return;
    while (*s) putc_bounded(pos, *s++);
}

static void uint_bounded(unsigned int *pos, unsigned int n)
{
    char b[11];
    unsigned int i = sizeof(b);
    do { b[--i] = (char)('0' + n % 10U); n /= 10U; } while (n);
    while (i < sizeof(b)) putc_bounded(pos, b[i++]);
}

static void int_bounded(unsigned int *pos, int n)
{
    if (n < 0) { putc_bounded(pos, '-'); n = -n; }
    uint_bounded(pos, (unsigned int)n);
}

static const char *flight_name(FlightState s)
{
    switch (s) {
    case FLIGHT_IDLE: return "IDLE"; case FLIGHT_PRE_LAUNCH: return "PRE";
    case FLIGHT_ASCENT: return "ASCENT"; case FLIGHT_APOGEE: return "APOGEE";
    case FLIGHT_DESCENT: return "DESCENT"; case FLIGHT_LANDED: return "LANDED";
    case FLIGHT_BOOT_LEVEL: return "BOOT_LEVEL"; case FLIGHT_BOOT_HOLD: return "BOOT_HOLD";
    default: return "UNKNOWN";
    }
}

static const char *count_name(CountdownState s)
{
    return s == COUNTDOWN_RUNNING ? "RUNNING" :
           s == COUNTDOWN_ARMED ? "ARMED" : "STOPPED";
}

static void append_codes(unsigned int *pos)
{
    unsigned int i, mask;
    int first = 1;
    mask = warning_mask();
    for (i = 0; i < WRN_COUNT; i++) if (mask & (1U << i)) {
        if (!first) putc_bounded(pos, ','); first = 0;
        uint_bounded(pos, STATUS_CODE_WRN(i));
    }
    mask = alarm_mask();
    for (i = 0; i < ALARM_COUNT; i++) if (mask & (1U << i)) {
        if (!first) putc_bounded(pos, ','); first = 0;
        uint_bounded(pos, STATUS_CODE_ALARM(i));
    }
    mask = master_alarm_mask();
    for (i = 0; i < MASTER_COUNT; i++) if (mask & (1U << i)) {
        if (!first) putc_bounded(pos, ','); first = 0;
        uint_bounded(pos, STATUS_CODE_MASTER(i));
    }
    if (first) puts_bounded(pos, "NONE");
}

static unsigned int altitude_cm(float pressure, float ground)
{
    float altitude = 44330.0f * (1.0f - powf(pressure / ground, 0.1903f));
    if (altitude <= 0.0f) return 0U;
    if (altitude >= 42949672.0f) return 0xFFFFFFFFU;
    return (unsigned int)(altitude * 100.0f);
}

static void status_build_line(void)
{
    float press = 0.0f, ground = 0.0f;
    int16_t gyr[3] = {0};
    orientation_t orient = {0};
    unsigned int pos = 0, i;
    uint16_t batt = 0, v5 = 0;

    (void)bme280_read(0, 0, &press);
    (void)bme280_ground_pressure(&ground);
    (void)bno055_read(0, gyr, 0);
    (void)orientation_sample(&orient);
    (void)power_read_battery_mv(&batt);
    (void)power_read_5v_mv(&v5);

    puts_bounded(&pos, "STAT v=1 flight=");
    puts_bounded(&pos, flight_name(flight_state()));
    puts_bounded(&pos, " count=");
    puts_bounded(&pos, count_name(countdown_state()));
    puts_bounded(&pos, " warn=");
    uint_bounded(&pos, (unsigned int)warning_count());
    puts_bounded(&pos, " alarm=");
    uint_bounded(&pos, (unsigned int)alarm_count());
    puts_bounded(&pos, " master=");
    uint_bounded(&pos, (unsigned int)master_alarm_count());
    puts_bounded(&pos, " codes=");
    append_codes(&pos);
    puts_bounded(&pos, " pressure_hpa=");
    int_bounded(&pos, (int)(press * 100.0f));
    puts_bounded(&pos, " altitude_cm=");
    uint_bounded(&pos, ground > 0.0f ? altitude_cm(press, ground) : 0U);
    puts_bounded(&pos, " gyro_dps=");
    for (i = 0; i < 3; i++) { if (i) putc_bounded(&pos, ','); int_bounded(&pos, gyr[i]); }
    puts_bounded(&pos, " nose=");
    if (!orientation_ready()) puts_bounded(&pos, "CALIBRATING");
    else if (orient.dir == 'U') puts_bounded(&pos, "UP");
    else if (orient.dir == 'D') puts_bounded(&pos, "DOWN");
    else { uint_bounded(&pos, (unsigned int)orient.clock_h); puts_bounded(&pos, "H/"); uint_bounded(&pos, (unsigned int)orient.miss_deg); puts_bounded(&pos, "deg"); }
    puts_bounded(&pos, " stab=");
    for (i = 0; i < 4; i++) {
        int16_t deg = 0;
        (void)stabilization_command_get((uint8_t)i, &deg);
        if (i) putc_bounded(&pos, ',');
        uint_bounded(&pos, i); putc_bounded(&pos, ':'); int_bounded(&pos, deg); putc_bounded(&pos, 'd');
    }
    puts_bounded(&pos, " batt_mv="); uint_bounded(&pos, batt);
    puts_bounded(&pos, " 5v_mv="); uint_bounded(&pos, v5);
    puts_bounded(&pos, " sd_transport=");
    puts_bounded(&pos, sd_spi_self_test() == 0 ? "OK" : "FAIL");
    puts_bounded(&pos, " logger_fs=");
    puts_bounded(&pos, logger_ready() ? "OK" : "FAIL");
    puts_bounded(&pos, " safety="); puts_bounded(&pos, safety_triggered() ? "ON" : "OFF");
    puts_bounded(&pos, " preflight=");
    if (!preflight_has_result()) puts_bounded(&pos, "UNKNOWN");
    else puts_bounded(&pos, preflight_passed() ? "PASS" : "FAIL");
    sline[pos] = '\0';
}

void status_report_init(void) { next_tick = HAL_GetTick() + STATUS_REPORT_PERIOD_MS; }

void status_report_update(void)
{
    uint32_t now = HAL_GetTick();
    if (now < next_tick) return;
    next_tick = now + STATUS_REPORT_PERIOD_MS;
    status_build_line();
    serial_puts(sline);
    serial_puts("\r\n");
}

void status_report_send_lora(void)
{
    status_build_line();
    lora_send((const uint8_t *)sline, (uint8_t)strlen(sline));
}

void status_report_event(const char *level, unsigned int code, const char *text)
{
    serial_puts("EVENT level=");
    serial_puts(level ? level : "INFO");
    serial_puts(" code=");
    print_unsigned(code);
    serial_puts(" text=");
    serial_puts(text ? text : "NONE");
    serial_puts("\r\n");
}
