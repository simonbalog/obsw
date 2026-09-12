#ifndef RTC_H
#define RTC_H

#include <stdint.h>

/*
 * RTC modul - DS1307 (HW-084) na I2C (adresa 0x68).
 *
 *   rtc_init()     - init RTC, spusti hodiny (CH bit), vynuti 24h mod
 *   rtc_self_test  - overi pritomnost na I2C
 *   rtc_get_time() - vraceni casu (HH:MM:SS)
 *   rtc_get_datetime() - plne datum a cas (YYYY-MO-DD HH:MM:SS)
 *   rtc_set_datetime() - nastaveni casu (po vyjimuti baterie, prvni start)
 *
 * rtc_get_datetime() vraci 0 pokud je cas platny, jinak -1.
 * Telemetrie pouzije datum kdyz je k dispozici, jinak ---.
 */

int rtc_init(void);
int rtc_self_test(void);
int rtc_get_time(uint8_t *h, uint8_t *m, uint8_t *s);
int rtc_get_datetime(uint16_t *y, uint8_t *mo, uint8_t *d,
                     uint8_t *h, uint8_t *mi, uint8_t *s);
int rtc_set_datetime(uint16_t y, uint8_t mo, uint8_t d,
                     uint8_t h, uint8_t mi, uint8_t s);

#endif
