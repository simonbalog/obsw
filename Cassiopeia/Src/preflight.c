#include "preflight.h"
#include "alarm.h"
#include "bme280.h"
#include "bno055.h"
#include "power.h"
#include "status_report.h"
#include "serial_monitor.h"
#include "lora.h"
#include "pca9685.h"
#include "sd_spi.h"
#include "logger.h"
#include "watchdog.h"
#include "flight.h"
#include "orientation.h"
#include <string.h>

#define PREFLIGHT_CHECKS 11U
#define PREFLIGHT_CODE_BASE 400U

static const char *names[PREFLIGHT_CHECKS] =
    { "BME280", "BNO055", "POWER", "SERVOS", "LORA", "SD", "LOGGER",
      "WATCHDOG", "FLIGHT", "CALIBRATION", "ALARMS" };
static uint16_t results[PREFLIGHT_CHECKS];
static int overall_pass;
static int has_result;

static void append_uint(char *out, unsigned int *pos, unsigned int value)
{
    char b[11];
    unsigned int i = sizeof(b);
    do { b[--i] = (char)('0' + value % 10U); value /= 10U; } while (value && i);
    while (i < sizeof(b) && *pos < 220U) out[(*pos)++] = b[i++];
}

int preflight_run(void)
{
    float t = 0.0f, h = 0.0f, p = 0.0f;
    int16_t a[3], g[3], m[3];
    uint16_t batt = 0, v5 = 0;
    unsigned int i;

    for (i = 0; i < PREFLIGHT_CHECKS; i++) results[i] = 0;
    /* Codes are stable and per-module: 0 means pass, 1.. are defined by the
       module implementation and are also emitted as EVENT records. */
    if (bme280_self_test() != 0 || bme280_read(&t, &h, &p) != 0)
        results[0] = PREFLIGHT_CODE_BASE + 1U;
    if (bno055_self_test() != 0 || bno055_read(a, g, m) != 0)
        results[1] = PREFLIGHT_CODE_BASE + 2U;
    if (power_read_battery_mv(&batt) != 0 || power_read_5v_mv(&v5) != 0)
        results[2] = PREFLIGHT_CODE_BASE + 3U;
    if (pca9685_self_test() != 0)
        results[3] = PREFLIGHT_CODE_BASE + 4U;
    if (lora_self_test() != 0)
        results[4] = PREFLIGHT_CODE_BASE + 5U;
    if (sd_spi_self_test() != 0)
        results[5] = PREFLIGHT_CODE_BASE + 6U;
    if (!logger_ready())
        results[6] = PREFLIGHT_CODE_BASE + 7U;
    if (watchdog_self_test() != 0)
        results[7] = PREFLIGHT_CODE_BASE + 8U;
    if (flight_state() != FLIGHT_IDLE || bno055_flight_ready() == 0 ||
        bme280_ground_pressure(0) != 0)
        results[8] = PREFLIGHT_CODE_BASE + 9U;
    if (!orientation_ready())
        results[9] = PREFLIGHT_CODE_BASE + 10U;

    /* An already active alarm is never hidden by a diagnostic run. */
    if (master_alarm_count() != 0 || alarm_count() != 0)
        results[10] = PREFLIGHT_CODE_BASE + 11U;

    overall_pass = 1;
    for (i = 0; i < PREFLIGHT_CHECKS; i++) {
        if (results[i] != 0) {
            overall_pass = 0;
            status_report_event("PREFLIGHT", results[i], names[i]);
        }
    }
    has_result = 1;

    status_report_event(overall_pass ? "PREFLIGHT" : "MASTER",
                        overall_pass ? 0U : PREFLIGHT_CODE_BASE,
                        overall_pass ? "PASS" : "FAIL");
    alarm_update_leds();
    return overall_pass ? 0 : -1;
}

int preflight_has_result(void) { return has_result; }
int preflight_passed(void) { return has_result && overall_pass; }

void preflight_report_lora(void)
{
    char out[240];
    unsigned int pos = 0, i;
    (void)preflight_run();
    const char *prefix = "PREFLIGHT result=";
    while (*prefix && pos < sizeof(out) - 1U) out[pos++] = *prefix++;
    const char *state = overall_pass ? "PASS" : "FAIL";
    while (*state && pos < sizeof(out) - 1U) out[pos++] = *state++;
    for (i = 0; i < PREFLIGHT_CHECKS && pos < sizeof(out) - 1U; i++) {
        const char *name = names[i];
        out[pos++] = ' '; while (*name && pos < sizeof(out) - 1U) out[pos++] = *name++;
        out[pos++] = '='; append_uint(out, &pos, results[i]);
    }
    out[pos] = '\0';
    serial_puts(out);
    serial_puts("\r\n");
    lora_send((const uint8_t *)out, (uint8_t)strlen(out));
}
