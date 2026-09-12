#include "status_report.h"
#include "serial_monitor.h"
#include "countdown.h"
#include "flight.h"
#include "alarm.h"
#include "safety.h"
#include "power.h"
#include "lora.h"
#include "telem_buf.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/*
 * Status report - kompletni prehled systemu na serial monitor.
 *
 * Periodicky (STATUS_REPORT_PERIOD_MS) vypise radek se vsemi
 * dulezitymi informacemi, takze pri pripojeni kabelem (rampa / po
 * pristani) je hned videt stav systemu bez hledani v telemetrii.
 *
 * Format (CSV, bez mezer - snadny parsing i lidske cteni):
 *
 *   S;<flight>;<count_state>;<phase>;<T-rem>;<alarms>;<master>;<safety>;<batt_mV>;<5V_mV>;<rssi>
 */

#define STATUS_REPORT_PERIOD_MS 2000
#define STATUS_LINE_MAX 160

static uint32_t next_tick = 0;
static char sline[STATUS_LINE_MAX];

static int print_unsigned_to(char *buf, unsigned int n)
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

static int print_int_to(char *buf, int n)
{
    int i = 0;
    if (n < 0)
    {
        buf[i++] = '-';
        n = -n;
    }
    i += print_unsigned_to(buf + i, (unsigned int)n);
    return i;
}

static const char *flight_name(FlightState s)
{
    switch (s)
    {
    case FLIGHT_IDLE:       return "IDLE";
    case FLIGHT_PRE_LAUNCH: return "PRE_LAUNCH";
    case FLIGHT_ASCENT:     return "ASCENT";
    case FLIGHT_APOGEE:     return "APOGEE";
    case FLIGHT_DESCENT:    return "DESCENT";
    case FLIGHT_LANDED:     return "LANDED";
    case FLIGHT_BOOT_LEVEL: return "BOOT_LEVEL";
    case FLIGHT_BOOT_HOLD:  return "BOOT_HOLD";
    default:                return "?";
    }
}

static const char *count_state_name(CountdownState s)
{
    switch (s)
    {
    case COUNTDOWN_STOPPED: return "STOPPED";
    case COUNTDOWN_RUNNING: return "RUNNING";
    case COUNTDOWN_ARMED:   return "ARMED";
    default:                return "?";
    }
}

static const char *count_phase_name(CountdownPhase p)
{
    switch (p)
    {
    case COUNTDOWN_PHASE_IDLE:    return "IDLE";
    case COUNTDOWN_PHASE_CHECK:   return "CHECK";
    case COUNTDOWN_PHASE_WAVE:    return "WAVE";
    case COUNTDOWN_PHASE_WARNING: return "WARNING";
    default:                      return "?";
    }
}

void status_report_init(void)
{
    next_tick = HAL_GetTick() + STATUS_REPORT_PERIOD_MS;
}

static void status_build_line(void)
{
    uint32_t rem = countdown_remaining_s();

    uint16_t batt = 0, v5 = 0;
    if (power_read_battery_mv(&batt) != 0)
        batt = 0;
    if (power_read_5v_mv(&v5) != 0)
        v5 = 0;

    int i = 0;
    sline[i++] = 'S';
    sline[i++] = ';';
    {
        const char *s = flight_name(flight_state());
        while (*s) sline[i++] = *s++;
    }
    sline[i++] = ';';
    {
        const char *s = count_state_name(countdown_state());
        while (*s) sline[i++] = *s++;
    }
    sline[i++] = ';';
    {
        const char *s = count_phase_name(countdown_phase());
        while (*s) sline[i++] = *s++;
    }
    sline[i++] = ';';
    sline[i++] = 'T';
    sline[i++] = '-';
    i += print_unsigned_to(sline + i, rem / 60);
    sline[i++] = ':';
    if (rem % 60 < 10) sline[i++] = '0';
    i += print_unsigned_to(sline + i, rem % 60);
    sline[i++] = ';';
    {
        const char *s = "alarms=";
        while (*s) sline[i++] = *s++;
    }
    i += print_unsigned_to(sline + i, (unsigned int)alarm_count());
    sline[i++] = ';';
    {
        const char *s = "master=";
        while (*s) sline[i++] = *s++;
    }
    i += print_unsigned_to(sline + i, (unsigned int)master_alarm_count());
    sline[i++] = ';';
    {
        const char *s = "warn=";
        while (*s) sline[i++] = *s++;
    }
    i += print_unsigned_to(sline + i, (unsigned int)warning_count());
    sline[i++] = ';';
    {
        const char *s = "safety=";
        while (*s) sline[i++] = *s++;
    }
    {
        const char *s = safety_triggered() ? "ON" : "OFF";
        while (*s) sline[i++] = *s++;
    }
    sline[i++] = ';';
    {
        const char *s = "batt=";
        while (*s) sline[i++] = *s++;
    }
    i += print_unsigned_to(sline + i, (unsigned int)batt);
    {
        const char *s = "mV;5V=";
        while (*s) sline[i++] = *s++;
    }
    i += print_unsigned_to(sline + i, (unsigned int)v5);
    {
        const char *s = "mV;rssi=";
        while (*s) sline[i++] = *s++;
    }
    i += print_int_to(sline + i, lora_last_rssi());
    {
        const char *s = "dBm;b1=";
        while (*s) sline[i++] = *s++;
    }
    i += print_unsigned_to(sline + i, (unsigned int)(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET ? 0 : 1));
    sline[i++] = ';';
    {
        const char *s = "buf=";
        while (*s) sline[i++] = *s++;
    }
    i += print_unsigned_to(sline + i, (unsigned int)telem_buf_fill_pct());
    sline[i++] = '%';
    sline[i++] = ';';
    {
        const char *s = "ev=";
        while (*s) sline[i++] = *s++;
    }
    i += print_unsigned_to(sline + i, (unsigned int)telem_buf_evicted());
    sline[i++] = ';';
    {
        const char *s = "tx=";
        while (*s) sline[i++] = *s++;
    }
    i += print_unsigned_to(sline + i, lora_tx_ok());
    sline[i++] = ';';
    {
        const char *s = "txfail=";
        while (*s) sline[i++] = *s++;
    }
    i += print_unsigned_to(sline + i, lora_tx_fail());
    sline[i] = 0;
}

void status_report_update(void)
{
    uint32_t now = HAL_GetTick();
    if (now < next_tick)
        return;
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
