#include "gps.h"
#include "serial_monitor.h"
#include "stm32h7xx_hal.h"

/*
 * GPS modul: Waveshare LC76G (Multi-GNSS: GPS+BDS+GLONASS+Galileo+QZSS).
 *
 * Komunikace UART 9600 baud 8N1 (default dle Waveshare wiki pro tento
 * modul), NMEA0183 vety (GGA, RMC, ...). Pripojeno na USART1:
 *   GPS TX  -> PA10 (USART1_RX)      GPS RX -> PA9 (USART1_TX)
 *   GPS VCC -> 3V3                    GPS GND -> GND
 *
 * Prijem pres RXNE interrupt do kruhoveho bufferu, zpracovani vet
 * v gps_update() volanem z hlavni smycky. Parsuje GGA (pozice, fix,
 * pocet satelitu, vyska) a RMC (validita, UTC cas/datum).
 */

#define GPS_RX_BUF_SIZE 256
#define GPS_LINE_MAX    96

/* LC76G vystup: 115200 baud (APB2 = 64 MHz -> BRR 556). Pokud modul
   vysila na 9600 (BRR 6667), data budou zmetky - viz $PAIR864. */
#define GPS_BRR_115200  556
#define GPS_BRR_9600    6667
#define GPS_BAUD_DEFAULT GPS_BRR_115200

/* USART1 na APB2 (64 MHz), base = 0x40011000 */
#define USART1_CR1 (*(volatile unsigned int*)(0x40011000))
#define USART1_BRR (*(volatile unsigned int*)(0x4001100C))
#define USART1_ISR (*(volatile unsigned int*)(0x4001101C))
#define USART1_RDR (*(volatile unsigned int*)(0x40011024))

/* RCC APB2ENR = RCC_BASE(0x58024400) + 0xF0, USART1EN = bit4 */
#define RCC_APB2ENR (*(volatile unsigned int*)(0x580244F0))
/* RCC AHB4ENR = 0x580244E0, GPIOAEN = bit0 */
#define RCC_AHB4ENR (*(volatile unsigned int*)(0x580244E0))
/* GPIOA base = 0x58020000 */
#define GPIOA_MODER (*(volatile unsigned int*)(0x58020000))
#define GPIOA_AFRH  (*(volatile unsigned int*)(0x58020024))

#define USART1_CR1_UE      (1U << 0)
#define USART1_CR1_RE      (1U << 2)
#define USART1_CR1_TE      (1U << 3)
#define USART1_CR1_RXNEIE  (1U << 5)

#define USART1_ISR_RXNE    (1U << 5)
#define USART1_ISR_ORE     (1U << 3)

static volatile uint8_t rx_buf[GPS_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;

static int present = 0;
static int gps_valid = 0;      /* posledni fix je platny */
static unsigned int nmea_lines = 0;  /* pocet prijatych NMEA vet (debug) */
static volatile unsigned int rx_bytes = 0;  /* pocet surovych bajtu z UARTu (debug) */
static int32_t lat_e6 = 0;     /* stupne * 1e6, kladny = sever */
static int32_t lon_e6 = 0;     /* stupne * 1e6, kladny = vychod */
static uint8_t fix_quality = 0;
static uint8_t sats = 0;
static int16_t alt_m = 0;
static uint8_t utc_h = 0, utc_m = 0, utc_s = 0;
static uint8_t utc_d = 0, utc_mo = 0;
static uint16_t utc_y = 0;

/* NMEA parser stavy */
#define PARSE_IDLE   0
#define PARSE_LINE   1
static uint8_t parse_state = PARSE_IDLE;
static char line[GPS_LINE_MAX];
static uint8_t line_len = 0;

static void uart_init(uint32_t brr)
{
    RCC_AHB4ENR |= (1U << 0); /* GPIOA clock */

    /* PA9, PA10 -> AF7 (USART1) */
    GPIOA_MODER = (GPIOA_MODER & ~((3U << 18) | (3U << 20)))
                | (2U << 18) | (2U << 20);
    GPIOA_AFRH = (GPIOA_AFRH & ~(0xFFU << 4)) | (0x77U << 4);

    RCC_APB2ENR |= (1U << 4); /* USART1 clock */

    USART1_CR1 = 0;
    USART1_BRR = brr;
    USART1_CR1 = USART1_CR1_UE | USART1_CR1_RE | USART1_CR1_TE | USART1_CR1_RXNEIE;
}

void gps_uart_irq(void)
{
    uint32_t isr = USART1_ISR;
    if (isr & USART1_ISR_ORE)
        USART1_ISR = USART1_ISR_ORE; /* vycistit overrun */
    if (isr & USART1_ISR_RXNE)
    {
        uint8_t c = (uint8_t)(USART1_RDR & 0xFF);
        rx_bytes++;
        uint16_t next = (uint16_t)((rx_head + 1) % GPS_RX_BUF_SIZE);
        if (next != rx_tail)
        {
            rx_buf[rx_head] = c;
            rx_head = next;
        }
    }
}

static int rx_available(void)
{
    return (int)((rx_head - rx_tail + GPS_RX_BUF_SIZE) % GPS_RX_BUF_SIZE);
}

static uint8_t rx_get(void)
{
    uint8_t c = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) % GPS_RX_BUF_SIZE);
    return c;
}

/* konverze NMEA pozice "ddmm.mmmm" (resp. "dddmm.mmmm") na stupne*1e6 */
static int32_t nmea_to_e6(const char *s, int len)
{
    int32_t deg = 0, frac = 0;
    int i = 0;
    while (i < len && s[i] >= '0' && s[i] <= '9')
    {
        deg = deg * 10 + (s[i] - '0');
        i++;
    }
    /* stupne maji 2 (lat) nebo 3 (lon) cifry pred teckou; zbytek = minuty */
    int32_t minutes = 0, scale = 1;
    if (i < len && s[i] == '.')
    {
        i++;
        int digits = 0;
        while (i < len && digits < 6 && s[i] >= '0' && s[i] <= '9')
        {
            minutes = minutes * 10 + (s[i] - '0');
            scale *= 10;
            digits++;
            i++;
        }
    }
    /* NMEA "ddmm.mmmm"/"dddmm.mmmm": minuty = posledni 2 cifry cele casti */
    int32_t deg_part = deg / 100;
    int32_t min_part = deg % 100;
    /* minute jako desetinne: minutes/scale.
       minutes * 1e6 preteka int32 (minutes > 2147), proto int64! */
    int64_t deg_e6 = (int64_t)deg_part * 1000000;
    int64_t min_e6 = ((int64_t)min_part * 1000000 + ((int64_t)minutes * 1000000) / scale) / 60;
    frac = (int32_t)(deg_e6 + min_e6);
    return frac;
}

static int field(const char *l, int n, const char **start, int *len)
{
    int i = 0, f = 0;
    while (l[i] && f < n)
    {
        if (l[i] == ',')
            f++;
        i++;
    }
    *start = &l[i];
    *len = 0;
    while (l[i] && l[i] != ',')
    {
        (*len)++;
        i++;
    }
    return (*len > 0) ? 0 : -1;
}

static int digits(const char *s, int len, int *val)
{
    *val = 0;
    if (len <= 0)
        return -1;
    for (int i = 0; i < len; i++)
    {
        if (s[i] < '0' || s[i] > '9')
            return -1;
        *val = *val * 10 + (s[i] - '0');
    }
    return 0;
}

static void parse_line(const char *l)
{
    /* podporuje GGA i RMC (GPS i GNGNS prefixy); po '$' nasleduje
       2-znakove talker ID (GP, GN, ...) a typ vety */
    if (l[0] != '$')
        return;
    nmea_lines++;

    const char *type = l + 3;   /* "$GPGGA" -> "GGA" */
    if (type[0] == 'G' && type[1] == 'G' && type[2] == 'A')
    {
        /* $..GGA,hhmmss,lat,N,lon,E,fix,sats,hdop,alt,M,... */
        const char *s;
        int len;

        /* cas hhmmss */
        if (field(l, 1, &s, &len) == 0 && len >= 6)
        {
            int v;
            if (digits(s, 2, &v) == 0) utc_h = (uint8_t)v;
            if (digits(s + 2, 2, &v) == 0) utc_m = (uint8_t)v;
            if (digits(s + 4, 2, &v) == 0) utc_s = (uint8_t)v;
        }

        int32_t lat = 0, lon = 0;
        char ns = 0, ew = 0;
        if (field(l, 2, &s, &len) == 0)
            lat = nmea_to_e6(s, len);
        if (field(l, 3, &s, &len) == 0)
            ns = (len > 0) ? s[0] : 0;
        if (field(l, 4, &s, &len) == 0)
            lon = nmea_to_e6(s, len);
        if (field(l, 5, &s, &len) == 0)
            ew = (len > 0) ? s[0] : 0;

        int fix = -1;
        if (field(l, 6, &s, &len) == 0)
            digits(s, len, &fix);

        int sat = -1;
        if (field(l, 7, &s, &len) == 0)
            digits(s, len, &sat);

        int alt = -1;
        if (field(l, 9, &s, &len) == 0)
            digits(s, len, &alt);

        fix_quality = (uint8_t)((fix > 0) ? fix : 0);
        sats = (uint8_t)((sat > 0) ? sat : 0);
        if (alt >= 0)
            alt_m = (int16_t)alt;

        if (fix > 0)
        {
            lat_e6 = (ns == 'S') ? -lat : lat;
            lon_e6 = (ew == 'W') ? -lon : lon;
            gps_valid = 1;
        }
        else
        {
            gps_valid = 0;
        }
    }
    else if (type[0] == 'R' && type[1] == 'M' && type[2] == 'C')
    {
        /* $..RMC,hhmmss,A,lat,N,lon,E,spd,trk,ddmmyy,... */
        const char *s;
        int len;

        if (field(l, 2, &s, &len) == 0 && len >= 1 && s[0] == 'A')
        {
            int32_t lat = 0, lon = 0;
            char ns = 0, ew = 0;
            if (field(l, 3, &s, &len) == 0)
                lat = nmea_to_e6(s, len);
            if (field(l, 4, &s, &len) == 0)
                ns = (len > 0) ? s[0] : 0;
            if (field(l, 5, &s, &len) == 0)
                lon = nmea_to_e6(s, len);
            if (field(l, 6, &s, &len) == 0)
                ew = (len > 0) ? s[0] : 0;

            if (lat != 0 || lon != 0)
            {
                lat_e6 = (ns == 'S') ? -lat : lat;
                lon_e6 = (ew == 'W') ? -lon : lon;
                gps_valid = 1;
                if (fix_quality == 0)
                    fix_quality = 1;
            }
        }

        /* datum ddmmyy */
        if (field(l, 9, &s, &len) == 0 && len == 6)
        {
            int v;
            if (digits(s, 2, &v) == 0) utc_d = (uint8_t)v;
            if (digits(s + 2, 2, &v) == 0) utc_mo = (uint8_t)v;
            if (digits(s + 4, 2, &v) == 0) utc_y = (uint16_t)(2000 + v);
        }
    }
}

static void process_char(uint8_t c)
{
    if (parse_state == PARSE_IDLE)
    {
        if (c == '$')
        {
            parse_state = PARSE_LINE;
            line[0] = '$';
            line_len = 1;
        }
    }
    else
    {
        if (c == '\r' || c == '\n')
        {
            parse_state = PARSE_IDLE;
            line[line_len] = 0;
            /* minimalni NMEA veta je aspon "$XXGGA" (6 znaku); kratsi
               (sum/zmetky) se ignoruji, aby parse_line necetla stale bajty */
            if (line_len >= 6)
                parse_line(line);
            return;
        }
        if (line_len < GPS_LINE_MAX - 1)
            line[line_len++] = (char)c;
    }
}

int gps_init(void)
{
    /* LC76G: 115200 baud (APB2 = 64 MHz -> BRR 556).
       Pokud modul vysila na 9600, zmen na GPS_BRR_9600. */
    uart_init(GPS_BAUD_DEFAULT);

    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_EnableIRQ(USART1_IRQn);

    present = 1;
    return 0;
}

int gps_self_test(void)
{
    /* GPS nelze spolehlive detekovat v okamziku bootu (fix trva desitky
       sekund), takze UART inicializovany = pritomen. Skutecna validita
       se kontroluje pres gps_get_position() az po prvnim fixu. */
    return present ? 0 : -1;
}

void gps_update(void)
{
    while (rx_available() > 0)
        process_char(rx_get());
}

int gps_get_position(float *lat, float *lon)
{
    if (!gps_valid)
        return -1;
    if (lat) *lat = (float)lat_e6 / 1000000.0f;
    if (lon) *lon = (float)lon_e6 / 1000000.0f;
    return 0;
}

int gps_valid_fix(void)
{
    return gps_valid;
}

unsigned int gps_nmea_lines(void)
{
    return nmea_lines;
}

unsigned int gps_rx_bytes(void)
{
    return rx_bytes;
}

int gps_get_altitude(int16_t *alt)
{
    if (!gps_valid)
        return -1;
    if (alt) *alt = alt_m;
    return 0;
}

int gps_get_satellites(uint8_t *n)
{
    if (!gps_valid)
        return -1;
    if (n) *n = sats;
    return 0;
}

int gps_get_fix_quality(uint8_t *q)
{
    if (q) *q = fix_quality;
    return 0;
}

void gps_get_utc(uint8_t *h, uint8_t *m, uint8_t *s)
{
    if (h) *h = utc_h;
    if (m) *m = utc_m;
    if (s) *s = utc_s;
}

int gps_get_utc_date(uint16_t *y, uint8_t *mo, uint8_t *d)
{
    if (!gps_valid || utc_y == 0)
        return -1;
    if (y) *y = utc_y;
    if (mo) *mo = utc_mo;
    if (d) *d = utc_d;
    return 0;
}