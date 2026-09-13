#include "fatfs.h"
#include "sd_spi.h"
#include "serial_monitor.h"
#include "watchdog.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* Minimalni FAT32 - jen pro logger jedne slozky LOG.TXT.
 *
 *  fatfs_init()  - pripoji existujici FAT32 nebo zformátuje kartu
 *  fatfs_append_line() - prida radek na konec LOG.TXT
 *
 *  Cilem je, aby se karta dala vlozit do pocitace a LOG.TXT
 *  normalne otevrit. Nepodporuje slozky, mazani ani LFN.
 */

#define BPS 512

/* BPB offsety */
#define BPB_BYTES_PER_SEC    11
#define BPB_SPC              13
#define BPB_RESERVED         14
#define BPB_NUM_FATS         16
#define BPB_FAT16_SEC        22
#define BPB_TOTAL_SEC32      32
#define BPB_FAT_SEC32        36
#define BPB_ROOT_CLUSTER     44

#define FNAME_SIZE 8
#define FEXT_SIZE  3

#define FAT_EOC  0x0FFFFFFF
#define FAT_FREE 0x00000000

#define LOG_NAME "LOG"
#define LOG_EXT  "TXT"

typedef struct
{
    uint16_t bps;
    uint8_t  spc;
    uint16_t reserved;
    uint8_t  nfats;
    uint32_t fat_sectors;
    uint32_t root_cluster;
    uint32_t data_start;    /* prvni sektor clusteru 2 */
    uint32_t total_sectors;
    uint32_t volume_start;
    uint32_t volume_sectors;
    uint32_t total_clusters;
    uint32_t next_free;
} FatFs;

static FatFs fs;
static unsigned int fat2_fallback = 0;   /* pocet FAT2 copy failu */
static int mounted = 0;

/* tvrdy casovy strop pro fatfs_init: mount/rotace/otevreni LOG.TXT musi
   skoncit do ~0.4 s, jinak boot pokracuje bez SD (petry z main loop). */
#define FATFS_INIT_MAX_MS 400
/* tvrdy strop na jeden pripsany radek (hlavni smycka nesmi zustat viset) */
#define FATFS_LINE_MAX_MS 120
static uint32_t fs_deadline = 0;   /* 0 = bez limitu (normalni operace) */

static int fs_dead_expired(void)
{
    return fs_deadline != 0 && (int32_t)(HAL_GetTick() - fs_deadline) >= 0;
}

/* stav otevreneho souboru */
static uint32_t f_first_cluster;
static uint32_t f_size;
static uint32_t cur_cluster;
static uint32_t cur_byte_in_cluster;   /* 0..spc*BPS */
static uint32_t f_ent_sector;          /* sektor root dir zaznamu */
static uint16_t f_ent_off;             /* offset 32B zaznamu */

static uint8_t  sec_buf[BPS];          /* persistentni sektorovy buffer */
static uint32_t  sec_sector;           /* absolutni sektor bufferu */
static uint16_t  sec_fill;             /* platnych bajtu v bufferu (0..BPS) */

/* odlozene propojeni clusteru FAT[stary]=novy (kvuli poradi data->FAT) */
static uint32_t fk_pending_prev;
static uint32_t fk_pending_nc;

static int fs_read_sector(uint32_t sec, uint8_t *buf)
{
    if (fs_dead_expired() || (fs.total_sectors != 0 && sec >= fs.total_sectors))
        return -1;
    watchdog_refresh();
    return sd_spi_read_sector(sec, buf);
}

static int update_dir_size(void);

static int fs_write_sector(uint32_t sec, const uint8_t *buf)
{
    if (fs_dead_expired() || sec >= fs.total_sectors)
        return -1;
    watchdog_refresh();
    return sd_spi_write_sector(sec, buf);
}

static uint32_t cluster_to_sector(uint32_t c)
{
    if (c < 2 || c > fs.total_clusters + 1 || fs.spc == 0)
        return UINT32_MAX;
    uint64_t sec = (uint64_t)fs.data_start + (uint64_t)(c - 2) * fs.spc;
    if (sec >= fs.total_sectors || sec > UINT32_MAX ||
        sec + fs.spc > fs.total_sectors)
        return UINT32_MAX;
    return (uint32_t)sec;
}

static uint32_t fat_entry_sector(uint32_t cluster)
{
    if (cluster < 2 || cluster > fs.total_clusters + 1)
        return UINT32_MAX;
    uint64_t sec = (uint64_t)fs.volume_start + fs.reserved
                 + ((uint64_t)cluster * 4U) / BPS;
    return (sec < fs.data_start && sec < UINT32_MAX) ? (uint32_t)sec : UINT32_MAX;
}

static uint16_t fat_entry_off(uint32_t cluster)
{
    return (uint16_t)((cluster * 4) % BPS);
}

static uint32_t fat_read_entry(uint32_t cluster)
{
    if (cluster < 2 || cluster > fs.total_clusters + 1)
        return FAT_EOC;
    uint8_t buf[BPS];
    uint32_t sec = fat_entry_sector(cluster);
    if (sec == UINT32_MAX || fs_read_sector(sec, buf) != 0)
        return FAT_EOC;
    uint16_t o = fat_entry_off(cluster);
    return (uint32_t)buf[o] | ((uint32_t)buf[o + 1] << 8)
         | ((uint32_t)buf[o + 2] << 16) | ((uint32_t)buf[o + 3] << 24);
}

static int fat_write_entry(uint32_t cluster, uint32_t value)
{
    uint32_t sec = fat_entry_sector(cluster);
    if (sec == UINT32_MAX)
        return -1;
    uint8_t buf[BPS];
    if (fs_read_sector(sec, buf) != 0)
        return -1;
    uint16_t o = fat_entry_off(cluster);
    buf[o]     = value & 0xFF;
    buf[o + 1] = (value >> 8) & 0xFF;
    buf[o + 2] = (value >> 16) & 0xFF;
    buf[o + 3] = (value >> 24) & 0xFF;

    /* FAT1 = hlavni kopie, ta se vzdy cte */
    if (fs_write_sector(sec, buf) != 0)
        return -1;

    /* FAT2 = zrcadlo. Nektere karty maji v teto oblasti vadny blok;
       FAT2 neni pro nas funkci nutna (cteme vzdy FAT1). Selhani
       pouze ohlasi a pokracuje, jinak by vadny blok FAT2 blokoval
       veskere alokace clusteru. */
    if (fs.nfats > 1)
    {
        if (fs_write_sector(sec + fs.fat_sectors, buf) != 0)
        {
            static int fat2_warned = 0;
            fat2_fallback++;
            if (!fat2_warned)
            {
                fat2_warned = 1;
                serial_puts("fatfs: WARN: FAT2 copy write FAIL - continuing with FAT1\r\n");
            }
        }
    }
    return 0;
}

static uint32_t fat_alloc_cluster(void)
{
    uint32_t c = fs.next_free;
    uint32_t max_c = fs.total_clusters + 1;
    if (c < 2 || c > max_c)
        c = 2;

    uint32_t start = c;
    do
    {
        watchdog_refresh();
        if (fat_read_entry(c) == FAT_FREE)
        {
            if (fat_write_entry(c, FAT_EOC) != 0)
                return 0;
            fs.next_free = (c + 1 > max_c) ? 2 : c + 1;
            return c;
        }
        c = (c + 1 > max_c) ? 2 : c + 1;
    } while (c != start);

    return 0;
}

static uint32_t fat_next(uint32_t cluster)
{
    if (cluster < 2 || cluster > fs.total_clusters + 1)
        return 0;
    uint32_t v = fat_read_entry(cluster);
    if (v >= FAT_EOC)
        return 0;
    if (v < 2 || v > fs.total_clusters + 1)
        return 0;
    return v;
}

/* ---------------- mount / format ---------------- */

static void parse_bpb(const uint8_t *boot, uint32_t volume_start)
{
    fs.bps           = boot[BPB_BYTES_PER_SEC] | (boot[BPB_BYTES_PER_SEC + 1] << 8);
    fs.spc           = boot[BPB_SPC];
    fs.reserved      = boot[BPB_RESERVED] | (boot[BPB_RESERVED + 1] << 8);
    fs.nfats         = boot[BPB_NUM_FATS];
    fs.fat_sectors   = (uint32_t)boot[BPB_FAT_SEC32] | ((uint32_t)boot[BPB_FAT_SEC32 + 1] << 8)
                     | ((uint32_t)boot[BPB_FAT_SEC32 + 2] << 16) | ((uint32_t)boot[BPB_FAT_SEC32 + 3] << 24);
    fs.root_cluster  = (uint32_t)boot[BPB_ROOT_CLUSTER] | ((uint32_t)boot[BPB_ROOT_CLUSTER + 1] << 8)
                     | ((uint32_t)boot[BPB_ROOT_CLUSTER + 2] << 16) | ((uint32_t)boot[BPB_ROOT_CLUSTER + 3] << 24);
    fs.volume_start  = volume_start;
    fs.data_start    = volume_start + fs.reserved + fs.nfats * fs.fat_sectors;
    fs.next_free     = 2;
    uint32_t total_sec = (uint32_t)boot[BPB_TOTAL_SEC32] | ((uint32_t)boot[BPB_TOTAL_SEC32 + 1] << 8)
                       | ((uint32_t)boot[BPB_TOTAL_SEC32 + 2] << 16) | ((uint32_t)boot[BPB_TOTAL_SEC32 + 3] << 24);
    fs.volume_sectors = total_sec;
    fs.total_sectors = volume_start + total_sec;
    if (total_sec > fs.reserved + fs.nfats * fs.fat_sectors && fs.spc != 0)
        fs.total_clusters = (total_sec - fs.reserved - fs.nfats * fs.fat_sectors) / fs.spc;
    else
        fs.total_clusters = 0;
}

static int bpb_is_fat32(const uint8_t *boot)
{
    if (boot[510] != 0x55 || boot[511] != 0xAA)
        return 0;
    if (fs.bps != BPS || fs.spc == 0 || fs.nfats == 0)
        return 0;
    uint32_t fat16 = (uint32_t)boot[BPB_FAT16_SEC] | ((uint32_t)boot[BPB_FAT16_SEC + 1] << 8);
    if (fat16 != 0)
        return 0; /* FAT16/12 */
    if (fs.fat_sectors == 0 || fs.root_cluster < 2 ||
        fs.total_sectors <= fs.data_start || fs.total_clusters == 0 ||
        fs.root_cluster > fs.total_clusters + 1)
        return 0;
    if ((uint64_t)fs.data_start + (uint64_t)fs.total_clusters * fs.spc > fs.total_sectors)
        return 0;
    return 1;
}

/* je na karte jiny souborovy system s daty (FAT16/12, exFAT)?
   Takovou kartu NEformátujeme - mohli bychom smazat data. */
static int bpb_other_fs(const uint8_t *boot)
{
    if (boot[510] != 0x55 || boot[511] != 0xAA)
        return 0; /* neni ani FAT - prazdna/nepodporovana karta */
    if (memcmp(&boot[3], "EXFAT   ", 8) == 0)
        return 1;
    if (fs.bps == BPS && fs.spc != 0 && fs.nfats != 0)
    {
        uint32_t fat16 = (uint32_t)boot[BPB_FAT16_SEC] | ((uint32_t)boot[BPB_FAT16_SEC + 1] << 8);
        if (fat16 != 0)
            return 1; /* FAT16/FAT12 */
    }
    return 0;
}

/* Prochazeni root retezce musi byt omezeno poctem hopu: kdyz SPI cteni
   z polozene/burajici karty vrati chybny zaznam FAT (cyklus, ne-EOC),
   smycka by jinak bezela vecne pred watchdogem. Root adresar realne
   trva par clusteru - 4096 hopu je vice nez stesti. */
#define ROOT_CHAIN_GUARD 4096

/* najde 32B zaznam LOG.TXT v root dir.
   Vrati 1 pokud zaznam existuje (a naplni entry_data), 0 jinak.
   entry_data musi mit alespon 32 B. */
static int root_find_log(uint8_t *entry_data, uint32_t *ent_sec, uint16_t *ent_off)
{
    uint32_t c = fs.root_cluster;
    uint8_t buf[BPS];
    uint32_t guard = 0;

    while (c != 0 && guard++ <= ROOT_CHAIN_GUARD)
    {
        for (uint32_t s = 0; s < fs.spc; s++)
        {
            if (fs_read_sector(cluster_to_sector(c) + s, buf) != 0)
                return 0;
            for (int e = 0; e < BPS / 32; e++)
            {
                uint8_t *en = &buf[e * 32];
                if (en[0] == 0x00) /* konec adresare */
                {
                    if (entry_data)
                    {
                        memcpy(entry_data, buf, BPS);
                        *ent_sec = cluster_to_sector(c) + s;
                        *ent_off = (uint16_t)(e * 32);
                    }
                    return 0;
                }
                if (en[0] == 0xE5)
                    continue;
                if (memcmp(en, "LOG     ", FNAME_SIZE) == 0 && memcmp(&en[8], "TXT", FEXT_SIZE) == 0)
                {
                    if (entry_data)
                    {
                        memcpy(entry_data, buf, BPS);
                        *ent_sec = cluster_to_sector(c) + s;
                        *ent_off = (uint16_t)(e * 32);
                    }
                    return 1;
                }
            }
        }
        c = fat_next(c);
    }
    return 0;
}

/* --- obecne prace s root adresarem (rotace logu) -------------------- */

/* najde 32B zaznam souboru name/ext v root dir. Vrati 1 = nalezeno. */
static int root_find_entry(const char *name, const char *ext,
                           uint8_t *entry_data, uint32_t *ent_sec, uint16_t *ent_off)
{
    uint32_t c = fs.root_cluster;
    uint8_t buf[BPS];
    uint32_t guard = 0;

    while (c != 0 && guard++ <= ROOT_CHAIN_GUARD)
    {
        for (uint32_t s = 0; s < fs.spc; s++)
        {
            if (fs_read_sector(cluster_to_sector(c) + s, buf) != 0)
                return 0;
            for (int e = 0; e < BPS / 32; e++)
            {
                uint8_t *en = &buf[e * 32];
                if (en[0] == 0x00)
                    return 0;
                if (en[0] == 0xE5)
                    continue;
                if (memcmp(en, name, FNAME_SIZE) == 0 &&
                    memcmp(&en[8], ext, FEXT_SIZE) == 0)
                {
                    if (entry_data)
                    {
                        memcpy(entry_data, buf, BPS);
                        *ent_sec = cluster_to_sector(c) + s;
                        *ent_off = (uint16_t)(e * 32);
                    }
                    return 1;
                }
            }
        }
        c = fat_next(c);
    }
    return 0;
}

/* najde volny slot v root dir (konec adresare nebo smazany zaznam) */
static int root_find_free(uint8_t *entry_data, uint32_t *ent_sec, uint16_t *ent_off)
{
    uint32_t c = fs.root_cluster;
    uint8_t buf[BPS];
    uint32_t guard = 0;

    while (c != 0 && guard++ <= ROOT_CHAIN_GUARD)
    {
        for (uint32_t s = 0; s < fs.spc; s++)
        {
            if (fs_read_sector(cluster_to_sector(c) + s, buf) != 0)
                return 0;
            for (int e = 0; e < BPS / 32; e++)
            {
                uint8_t *en = &buf[e * 32];
                if (en[0] == 0x00 || en[0] == 0xE5)
                {
                    memcpy(entry_data, buf, BPS);
                    *ent_sec = cluster_to_sector(c) + s;
                    *ent_off = (uint16_t)(e * 32);
                    return 1;
                }
            }
        }
        c = fat_next(c);
    }
    return 0;
}

/* uvolni retezec clusteru (nastavi FAT zaznamy na 0) */
static void free_chain(uint32_t first)
{
    uint32_t c = first;
    uint32_t guard = 0;
    while (c >= 2 && guard++ <= fs.total_clusters)
    {
        uint32_t next = fat_next(c);
        fat_write_entry(c, 0);
        if (next == 0 || next < 2)
            break;
        c = next;
    }
}

/* smaze soubor z root dir (dir zaznam + data na karte) */
static int root_delete_file(const char *name, const char *ext)
{
    uint8_t buf[BPS];
    uint32_t sec = 0;
    uint16_t off = 0;
    if (!root_find_entry(name, ext, buf, &sec, &off))
        return -1;

    uint8_t *en = &buf[off];
    uint32_t first = ((uint32_t)en[21] << 24) | ((uint32_t)en[20] << 16)
                   | ((uint32_t)en[27] << 8) | (uint32_t)en[26];
    if (first >= 2)
        free_chain(first);

    buf[off] = 0xE5;
    return fs_write_sector(sec, buf);
}

/* prejmenovani/rotace logu bylo zruseno: pri bootu se do karty jen cte
   (a pri prvnim zapisu se LOG.TXT pripadne vytvori). Tim se vyrazne snizi
   pocet zapisu pri startu (mene rizika resetu/brownoutu) a chovani je
   jednodussi - LOG.TXT se proste porad pripisuje. Stare logy LOG1-3.TXT
   zustavaji na karte a daji se mazat/prepínat pres uplink. */

int fatfs_init(void)
{
    mounted = 0;
    memset(&fs, 0, sizeof(fs));
    fs_deadline = HAL_GetTick() + FATFS_INIT_MAX_MS;

    uint8_t boot[BPS];
    if (fs_read_sector(0, boot) != 0)
    {
        fs_deadline = 0;
        serial_puts("fatfs: read sector 0 FAIL\r\n");
        return -1;
    }

    uint32_t volume_start = 0;
    parse_bpb(boot, volume_start);
    if (!bpb_is_fat32(boot) && boot[510] == 0x55 && boot[511] == 0xAA)
    {
        /* Windows normally creates an MBR partition.  The FAT volume
           begins at the partition LBA, not at physical sector zero. */
        for (uint32_t i = 0; i < 4; i++)
        {
            const uint8_t *part = &boot[446 + i * 16];
            uint8_t type = part[4];
            uint32_t start = (uint32_t)part[8] | ((uint32_t)part[9] << 8)
                           | ((uint32_t)part[10] << 16) | ((uint32_t)part[11] << 24);
            uint32_t length = (uint32_t)part[12] | ((uint32_t)part[13] << 8)
                            | ((uint32_t)part[14] << 16) | ((uint32_t)part[15] << 24);
            if ((type == 0x0B || type == 0x0C) && start != 0 && length != 0)
            {
                if (sd_spi_read_sector(start, boot) != 0)
                    break;
                parse_bpb(boot, start);
                if (fs.volume_sectors > length ||
                    start > UINT32_MAX - fs.volume_sectors)
                {
                    fs.total_clusters = 0;
                    break;
                }
                volume_start = start;
                break;
            }
        }
    }
    if (!bpb_is_fat32(boot))
    {
        if (bpb_other_fs(boot))
        {
            fs_deadline = 0;
            serial_puts("fatfs: UNSUPPORTED filesystem - NOT formatting (preserve data!)\r\n");
            serial_puts("fatfs: vloz FAT32 kartu nebo ji naformatuj v pocitaci\r\n");
            return -1;
        }
        fs_deadline = 0;
        /* karta se ve firmware NIKDY neformatuje: pri startu rakety nechceme
           polovinu formatovani a polomrtvou kartu; kartu pripravime na PC */
        serial_puts("fatfs: not FAT32 - mount skipped (format card on PC)\r\n");
        return -1;
    }

    serial_puts("fatfs: FAT32 mounted\r\n");

    if (fatfs_open_log() != 0)
    {
        fs_deadline = 0;
        return -1;
    }

    /* mounted az po uspesnem otevreni LOG.TXT - jinak by telemetrie
       zapisovala s neinicializovanym stavem souboru (cur_cluster=0)
       do FAT/katalogu a znicila karty */
    fs_deadline = 0;
    mounted = 1;
    return 0;
}

int fatfs_mounted(void)
{
    return mounted;
}

int fatfs_open_log(void)
{
    uint8_t entbuf[BPS];
    uint32_t ent_sec = 0;
    uint16_t ent_off = 0;
    uint32_t first = 0;

    int rfl = root_find_log(entbuf, &ent_sec, &ent_off);

    if (rfl)
    {
        uint8_t *en = &entbuf[ent_off];
        first = ((uint32_t)en[21] << 24) | ((uint32_t)en[20] << 16)
              | ((uint32_t)en[27] << 8) | (uint32_t)en[26];
    }

    if (first == 0)
    {
        /* vytvorit novy soubor. Prvni volani root_find_log uz vyplnilo
           ent_sec/ent_off pozici volneho slotu (0x00 = konec adresare).
           Nekolikat ji znovu - vratila by zase 0 (soubor stale neexistuje). */
        if (ent_sec == 0)
        {
            serial_puts("fatfs: adresar plny, neni kam zapsat\r\n");
            return -1; /* adresar je plny, nemame kam zapsat */
        }

        uint32_t c = fat_alloc_cluster();
        if (c == 0)
        {
            serial_puts("fatfs: no free cluster\r\n");
            return -1;
        }

        uint8_t *en = &entbuf[ent_off];
        memset(en, 0, 32);
        memset(en, ' ', FNAME_SIZE);
        memset(&en[8], ' ', FEXT_SIZE);
        memcpy(en, LOG_NAME, strlen(LOG_NAME));
        memcpy(&en[8], LOG_EXT, strlen(LOG_EXT));
        en[11] = 0x20; /* archive */
        en[20] = (c >> 16) & 0xFF;
        en[21] = (c >> 24) & 0xFF;
        en[26] = c & 0xFF;
        en[27] = (c >> 8) & 0xFF;
        /* size = 0 */
        if (fs_write_sector(ent_sec, entbuf) != 0)
            return -1;
        first = c;
    }

    f_first_cluster = first;
    f_ent_sector = ent_sec;
    f_ent_off = ent_off;
    f_size = 0;
    cur_cluster = first;
    cur_byte_in_cluster = 0;
    sec_fill = 0;
    sec_sector = 0;

    /* velikost z dir zaznamu */
    {
        uint8_t tbuf[BPS];
        if (fs_read_sector(ent_sec, tbuf) == 0)
        {
            uint8_t *en = &tbuf[ent_off];
            f_size = (uint32_t)en[28] | ((uint32_t)en[29] << 8)
                   | ((uint32_t)en[30] << 16) | ((uint32_t)en[31] << 24);
        }
    }

    /* projiti na konec retezce a pozici v poslednim clusteru */
    uint32_t c = first;
    uint32_t bytes_in_chain = 0;
    uint32_t last_cluster = first;
    uint32_t walk_guard = 0;
    uint32_t chain_bad = 0;
    while (c != 0)
    {
        uint32_t next = fat_next(c);
        if (next == 0)
        {
            last_cluster = c;
            break;
        }
        if (++walk_guard > fs.total_clusters)
        {
            last_cluster = c;
            chain_bad = 1;
            break;
        }
        bytes_in_chain += fs.spc * BPS;
        c = next;
    }

    cur_cluster = last_cluster;
    if (f_size < bytes_in_chain)
        chain_bad = 1;
    cur_byte_in_cluster = (f_size >= bytes_in_chain) ? f_size - bytes_in_chain : 0;

    /* OCHRANA PROTI POSKOZENEMU RETEZCI:
       Pokud dir zaznam tvrdi vice dat, nez retezec fyzicky pokryva
       (napr. po prerusenem zapisu / vypadku napajeni), nebo se retezec
       zacyklil (chain_bad), je soubor poskozen. Retezec nelze bezpecne
       opravit, proto soubor smazeme a vytvorime novy (ztrata logu je
       lepsi nez zapis do neplatnych sektoru a dalsi niceni FAT). */
    if (chain_bad || cur_byte_in_cluster >= (uint32_t)fs.spc * BPS)
    {
        serial_puts("fatfs: WARN: LOG.TXT chain corrupt - recreating file\r\n");

        uint8_t dbuf[BPS];
        if (fs_read_sector(ent_sec, dbuf) != 0)
            return -1;
        /* smazat stary dir zaznam */
        dbuf[ent_off] = 0xE5;
        if (fs_write_sector(ent_sec, dbuf) != 0)
            return -1;

        /* vytvorit novy soubor */
        uint32_t c = fat_alloc_cluster();
        if (c == 0)
        {
            serial_puts("fatfs: no free cluster\r\n");
            return -1;
        }
        uint8_t *en = &dbuf[ent_off];
        memset(en, 0, 32);
        memset(en, ' ', FNAME_SIZE);
        memset(&en[8], ' ', FEXT_SIZE);
        memcpy(en, LOG_NAME, strlen(LOG_NAME));
        memcpy(&en[8], LOG_EXT, strlen(LOG_EXT));
        en[11] = 0x20; /* archive */
        en[20] = (c >> 16) & 0xFF;
        en[21] = (c >> 24) & 0xFF;
        en[26] = c & 0xFF;
        en[27] = (c >> 8) & 0xFF;
        if (fs_write_sector(ent_sec, dbuf) != 0)
            return -1;

        f_first_cluster = c;
        f_ent_sector = ent_sec;
        f_ent_off = ent_off;
        f_size = 0;
        cur_cluster = c;
        cur_byte_in_cluster = 0;
        sec_fill = 0;
        sec_sector = 0;

        serial_puts("fatfs: LOG.TXT recreated, new first=");
        print_unsigned((unsigned int)c);
        serial_puts("\r\n");
        return 0;
    }

    serial_puts("fatfs: LOG.TXT open, size=");
    print_unsigned((unsigned int)f_size);
    serial_puts(" first=");
    print_unsigned((unsigned int)first);
    serial_puts(" last=");
    print_unsigned((unsigned int)last_cluster);
    serial_puts(" chainB=");
    print_unsigned((unsigned int)bytes_in_chain);
    serial_puts(" posInLast=");
    print_unsigned((unsigned int)cur_byte_in_cluster);
    serial_puts("\r\n");
    serial_puts("fatfs: dataStart=");
    print_unsigned((unsigned int)fs.data_start);
    serial_puts(" spc=");
    print_unsigned((unsigned int)fs.spc);
    serial_puts(" totClu=");
    print_unsigned((unsigned int)fs.total_clusters);
    serial_puts("\r\n");
    return 0;
}

static int update_dir_size(void)
{
    uint8_t buf[BPS];
    if (fs_read_sector(f_ent_sector, buf) != 0)
        return -1;
    uint8_t *en = &buf[f_ent_off];
    en[28] = f_size & 0xFF;
    en[29] = (f_size >> 8) & 0xFF;
    en[30] = (f_size >> 16) & 0xFF;
    en[31] = (f_size >> 24) & 0xFF;
    return fs_write_sector(f_ent_sector, buf);
}

/* zapise jeden bajt na pozici cur_byte_in_cluster, posune stav */
static int append_byte(uint8_t b)
{
    if (cur_cluster < 2 || cur_cluster > fs.total_clusters + 1 ||
        cur_byte_in_cluster >= (uint32_t)fs.spc * BPS)
        return -1;
    uint32_t sector_idx = cur_byte_in_cluster / BPS;
    uint32_t abs_sec = cluster_to_sector(cur_cluster) + sector_idx;
    if (cluster_to_sector(cur_cluster) == UINT32_MAX ||
        abs_sec >= fs.total_sectors)
        return -1;
    uint16_t off = cur_byte_in_cluster % BPS;

    /* buffer musi odpovidat aktualnimu sektoru */
    if (sec_fill == 0 || sec_sector != abs_sec)
    {
        if (off != 0)
        {
            /* pokracujeme v existujicim sektoru - nacti ho */
            if (fs_read_sector(abs_sec, sec_buf) != 0)
                return -1;
        }
        else
        {
            memset(sec_buf, 0, BPS);
        }
        sec_sector = abs_sec;
    }

    sec_buf[off] = b;
    f_size++;
    cur_byte_in_cluster++;
    sec_fill = off + 1;

    if (sec_fill == BPS)
    {
        if (fs_write_sector(sec_sector, sec_buf) != 0)
            return -1;
        sec_fill = 0;
    }

    if (cur_byte_in_cluster == (uint32_t)fs.spc * BPS)
    {
        /* Retezec dosel na konec clusteru - jeho data jsou v tento
           okamzik jiz zapsana (posledni sektor se vyplnil a zapsal).
           Poradi pro bezpecny (vytrzeny zapis odolny) zapis:
             1) data noveho clusteru se zapisi vzdy DRIVE,
             2) FAT[novy] = EOC pri alokaci,
             3) FAT[stary] = novy AZ PO zapsani dat noveho clusteru
                (odlozeno a doreseno pri dalsim preteceni / na konci radku).
           Pri vypadku mezi 2 a 3 zustane jen neodkazovany cluster
           (preteceni) a retezec zustane platny - nikdy zlomeny. */
        if (fk_pending_prev)
        {
            if (fat_write_entry(fk_pending_prev, fk_pending_nc) != 0)
                return -1;
            fk_pending_prev = 0;
        }

        uint32_t nc = fat_alloc_cluster();
        if (nc == 0)
            return -1;
        fk_pending_prev = cur_cluster;
        fk_pending_nc = nc;
        cur_cluster = nc;
        cur_byte_in_cluster = 0;
        sec_fill = 0;
        sec_sector = 0;
    }
    return 0;
}

int fatfs_append_line(const char *line)
{
    if (!mounted || f_first_cluster == 0)
        return -1;

    /* jeden radek ma tvrdy strop: hlavni smycka nesmi na zaseknute
       karte uviset dele, jinak by let (odpocet/rizeni) stal */
    fs_deadline = HAL_GetTick() + FATFS_LINE_MAX_MS;
    watchdog_refresh();

    fk_pending_prev = 0;

    int rc = 0;
    while (*line)
    {
        if (append_byte((uint8_t)*line) != 0)
        {
            rc = -1;
            goto done;
        }
        line++;
    }
    if (append_byte((uint8_t)'\n') != 0)
    {
        rc = -1;
        goto done;
    }

    /* docist posledni parcialni sektor */
    if (sec_fill != 0)
    {
        if (fs_write_sector(sec_sector, sec_buf) != 0)
        {
            rc = -1;
            goto done;
        }
        sec_fill = 0;
    }

    /* az TEPRVE teď, kdy jsou data noveho clusteru fyzicky zapsana,
       propojit FAT[stary] = novy - vypadek mezi zapisem dat a spojem
       ponecha retezec platny (stary konec souboru bez noveho kusu) */
    if (fk_pending_prev)
    {
        if (fat_write_entry(fk_pending_prev, fk_pending_nc) != 0)
        {
            rc = -1;
            goto done;
        }
        fk_pending_prev = 0;
    }

    if (update_dir_size() != 0)
        rc = -1;

done:
    fs_deadline = 0;
    return rc;
}

int fatfs_sync(void)
{
    if (!mounted)
        return -1;
    fs_deadline = HAL_GetTick() + FATFS_LINE_MAX_MS;
    if (sec_fill != 0 && fs_write_sector(sec_sector, sec_buf) != 0)
    {
        fs_deadline = 0;
        return -1;
    }
    sec_fill = 0;
    if (fk_pending_prev != 0)
    {
        if (fat_write_entry(fk_pending_prev, fk_pending_nc) != 0)
        {
            fs_deadline = 0;
            return -1;
        }
        fk_pending_prev = 0;
    }
    int rc = update_dir_size();
    fs_deadline = 0;
    return rc;
}

unsigned int fatfs_fat2_fallback_count(void)
{
    return fat2_fallback;
}

/* --- vyber / mazani logu pres uplink (LOG.TXT, LOG1-3.TXT) ---------- */

static uint8_t active_log_num = 0;   /* kam se prave uklada (0-3) */

static const char *log_name_for(uint8_t n)
{
    static const char names[4][FNAME_SIZE + 1] = {
        "LOG     ", "LOG1    ", "LOG2    ", "LOG3    "
    };
    return (n <= 3) ? names[n] : names[0];
}

uint8_t fatfs_active_log(void)
{
    return active_log_num;
}

/* prepne aktivni log na LOG<n>.TXT - dalsi radky se budou zapisovat tam.
   Existujici soubor se pokracuje (pripise na konec), neexistujici se
   vytvori. Volat jen mezi radky (append_line vzal vypne buffer). */
int fatfs_select_log(uint8_t n)
{
    if (!mounted || n > 3)
        return -1;

    const char *name = log_name_for(n);
    uint8_t buf[BPS];
    uint32_t sec = 0;
    uint16_t off = 0;
    uint32_t first = 0;

    if (!root_find_entry(name, LOG_EXT, buf, &sec, &off))
    {
        /* neexistuje -> vytvorit prazdny soubor */
        if (!root_find_free(buf, &sec, &off))
            return -1;
        uint32_t c = fat_alloc_cluster();
        if (c == 0)
            return -1;
        uint8_t *en = &buf[off];
        memset(en, 0, 32);
        memset(en, ' ', FNAME_SIZE);
        memset(&en[8], ' ', FEXT_SIZE);
        memcpy(en, name, strlen(name));
        memcpy(&en[8], LOG_EXT, strlen(LOG_EXT));
        en[11] = 0x20; /* archive */
        en[20] = (c >> 16) & 0xFF;
        en[21] = (c >> 24) & 0xFF;
        en[26] = c & 0xFF;
        en[27] = (c >> 8) & 0xFF;
        if (fs_write_sector(sec, buf) != 0)
            return -1;

        f_first_cluster = c;
        f_ent_sector = sec;
        f_ent_off = off;
        f_size = 0;
        cur_cluster = c;
        cur_byte_in_cluster = 0;
    }
    else
    {
        /* existuje -> najit konec retezce a pripisovat */
        uint8_t *en = &buf[off];
        first = ((uint32_t)en[21] << 24) | ((uint32_t)en[20] << 16)
              | ((uint32_t)en[27] << 8) | (uint32_t)en[26];
        if (first < 2)
            return -1;
        f_size = (uint32_t)en[28] | ((uint32_t)en[29] << 8)
               | ((uint32_t)en[30] << 16) | ((uint32_t)en[31] << 24);

        uint32_t c = first;
        uint32_t bytes_in_chain = 0;
        uint32_t last = first;
        uint32_t guard = 0;
        int at_end = 0;
        while (c != 0 && guard++ <= fs.total_clusters)
        {
            uint32_t next = fat_next(c);
            if (next == 0)
            {
                last = c;
                at_end = 1;
                break;
            }
            bytes_in_chain += fs.spc * BPS;
            c = next;
        }
        if (!at_end)
            return -1;   /* zacykleny retezec - nemenime */

        cur_byte_in_cluster = f_size - bytes_in_chain;
        if (cur_byte_in_cluster >= (uint32_t)fs.spc * BPS)
            return -1;   /* poskozeny retezec - nemenime */

        f_first_cluster = first;
        f_ent_sector = sec;
        f_ent_off = off;
        cur_cluster = last;
    }

    sec_fill = 0;
    sec_sector = 0;
    active_log_num = n;
    return 0;
}

/* smaze zalohni log LOG<n>.TXT (n = 1..3). Kdyz je prave aktivni,
   nejdrive se prepnulo na LOG.TXT, aby stav nezustaval na smazanych
   clusterech. */
int fatfs_delete_log(uint8_t n)
{
    if (!mounted || n == 0 || n > 3)
        return -1;
    if (active_log_num == n)
        fatfs_select_log(0);
    return root_delete_file(log_name_for(n), LOG_EXT);
}

/* velikost a existence LOG<n>.TXT bez zmeny stavu (pro LOGS vypis) */
int fatfs_log_info(uint8_t n, uint32_t *size, uint8_t *exists)
{
    if (size)
        *size = 0;
    if (exists)
        *exists = 0;
    if (!mounted || n > 3)
        return -1;

    uint8_t buf[BPS];
    uint32_t sec = 0;
    uint16_t off = 0;
    if (!root_find_entry(log_name_for(n), LOG_EXT, buf, &sec, &off))
        return 0;

    uint8_t *en = &buf[off];
    if (exists)
        *exists = 1;
    if (size)
        *size = (uint32_t)en[28] | ((uint32_t)en[29] << 8)
              | ((uint32_t)en[30] << 16) | ((uint32_t)en[31] << 24);
    return 0;
}
