#include "telem_buf.h"
#include "lora.h"
#include "serial_monitor.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* Telemetrický kruhový buffer v RAM.
 *
 *  Záznamy se ukládají jako [1 B délka][data bez '\0']. Každý řádek má
 *  délkový prefix, takže lze procházet od nejstaršího k nejnovějšímu
 *  a pak poslat vše přes LoRa (dump po letu / ABORTu).
 *
 *  Kapacita je fixní (TELEM_BUF_SIZE). Když dojde místo, NEpřeteče -
 *  nejstarší záznam se vyhodí (ring buffer), aby se vešel nový. Tím se
 *  chrání paměť: nikdy nečte/píše mimo buffer, nikdy nepropadne
 *  do stacku/heapu. Vyhozené záznamy se počítají (telem_buf_evicted),
 *  takže víme, kolik dat se ztratilo.
 *
 *  Naplnění se hlídá tak, že dump nikdy nesmí běžet déle, než trvá
 *  watchdog timeout - posílá se max 1 záznam na jedno volání
 *  (telem_buf_dump_step), zbytek hlavní smyčka dokrmuje watchdog.
 */

#define TELEM_BUF_SIZE (192U * 1024U) /* 192 KB telemetrie = ~40-45 min letu */

static uint8_t buf[TELEM_BUF_SIZE];
static uint32_t head = 0;   /* zapisovaci pozice */
static uint32_t tail = 0;   /* nejstarsi zaznam */
static uint32_t bytes_used = 0;
static uint32_t line_count = 0;
static uint32_t evicted_count = 0;

static uint32_t dump_pos = 0;
static uint32_t dump_left = 0;
static uint32_t dump_ok = 0;
static uint32_t dump_fail = 0;
static uint8_t dump_active = 0;

void telem_buf_init(void)
{
    head = 0;
    tail = 0;
    bytes_used = 0;
    line_count = 0;
    evicted_count = 0;
    dump_pos = 0;
    dump_left = 0;
    dump_ok = 0;
    dump_fail = 0;
    dump_active = 0;
}

/* vymaze cely obsah bufferu (uvolni prostor pro dalsi let).
   Bezici dump se prerusi - po vymazani nema co posilat. */
void telem_buf_clear(void)
{
    telem_buf_init();
}

static void evict_oldest(void)
{
    if (line_count == 0)
        return;
    uint32_t len = buf[tail];
    uint32_t rec = 1 + len;
    tail = (tail + rec) % TELEM_BUF_SIZE;
    bytes_used -= rec;
    line_count--;
    evicted_count++;
}

int telem_buf_store(const char *line)
{
    uint32_t len = strlen(line);
    uint32_t need = 1 + len;
    if (len == 0 || need > TELEM_BUF_SIZE)
        return -1;

    while (bytes_used + need > TELEM_BUF_SIZE)
        evict_oldest();

    buf[head] = (uint8_t)len;
    for (uint32_t i = 0; i < len; i++)
        buf[(head + 1 + i) % TELEM_BUF_SIZE] = (uint8_t)line[i];

    head = (head + need) % TELEM_BUF_SIZE;
    bytes_used += need;
    line_count++;
    return 0;
}

uint32_t telem_buf_count(void)
{
    return line_count;
}

uint32_t telem_buf_evicted(void)
{
    return evicted_count;
}

uint32_t telem_buf_fill_pct(void)
{
    return (uint32_t)(((uint64_t)bytes_used * 100) / TELEM_BUF_SIZE);
}

void telem_buf_dump_start(void)
{
    if (line_count == 0)
    {
        serial_puts("telem_buf: dump - nothing to send\r\n");
        dump_active = 0;
        return;
    }
    dump_pos = tail;
    dump_left = line_count;
    dump_ok = 0;
    dump_fail = 0;
    dump_active = 1;

    serial_puts("telem_buf: dump over LoRa, lines=");
    print_unsigned(line_count);
    serial_puts("\r\n");
}

int telem_buf_dump_active(void)
{
    return dump_active;
}

int telem_buf_dump_step(void)
{
    if (!dump_active)
        return 1;
    if (dump_left == 0)
    {
        dump_active = 0;
        serial_puts("telem_buf: dump done, sent=");
        print_unsigned(dump_ok);
        serial_puts(" fail=");
        print_unsigned(dump_fail);
        serial_puts("\r\n");
        return 1;
    }

    uint32_t len = buf[dump_pos];
    uint8_t out[256];
    out[len] = 0;
    for (uint32_t i = 0; i < len; i++)
        out[i] = buf[(dump_pos + 1 + i) % TELEM_BUF_SIZE];

    if (lora_send(out, (uint8_t)len) == 0)
        dump_ok++;
    else
        dump_fail++;

    dump_pos = (dump_pos + 1 + len) % TELEM_BUF_SIZE;
    dump_left--;
    return dump_left == 0;
}