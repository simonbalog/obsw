#ifndef GPS_H
#define GPS_H

#include <stdint.h>

/*
 * GPS modul - Waveshare LC76G (Multi-GNSS) na USART1 (9600 baud, NMEA).
 *
 *   gps_init()        - init UART1 + RX interrupt
 *   gps_self_test     - 0 kdyz je UART inicializovan
 *   gps_update()      - zpracovat NMEA vety (volat z hlavni smycky)
 *   gps_get_position  - vratit lat/lon (stupne) pokud je platny fix
 *   gps_valid_fix     - 1 kdyz mame aktualni platny fix
 *   gps_get_altitude  - nadmorska vyska (m)
 *   gps_get_satellites- pocet viditelnych satelitu
 *   gps_get_utc       - UTC cas z posledni vety
 *
 * gps_get_position vraci 0 jen kdyz je fix platny, jinak -1.
 */

int gps_init(void);
int gps_self_test(void);
void gps_update(void);
void gps_uart_irq(void);
int gps_get_position(float *lat, float *lon);
int gps_valid_fix(void);
unsigned int gps_nmea_lines(void);  /* pocet prijatych NMEA vet (debug) */
unsigned int gps_rx_bytes(void);    /* pocet surovych bajtu z UARTu (debug) */
int gps_get_altitude(int16_t *alt);
int gps_get_satellites(uint8_t *n);
int gps_get_fix_quality(uint8_t *q);
void gps_get_utc(uint8_t *h, uint8_t *m, uint8_t *s);
int gps_get_utc_date(uint16_t *y, uint8_t *mo, uint8_t *d); /* 0 = platne datum z fixu */

#endif