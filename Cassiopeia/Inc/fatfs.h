#ifndef FATFS_H
#define FATFS_H

#include <stdint.h>

/* logy na karte: 0 = LOG.TXT (aktivni), 1-3 = LOG1/2/3.TXT (zalohy) */

int fatfs_init(void);
int fatfs_open_log(void);
int fatfs_append_line(const char *line);
unsigned int fatfs_fat2_fallback_count(void);
int fatfs_mounted(void);                /* 1 = mount probehl, log je otevreny */

int fatfs_select_log(uint8_t n);   /* prepne ukladani na LOG<n>.TXT */
int fatfs_delete_log(uint8_t n);   /* smaze LOG<n>.TXT (n = 1..3) */
int fatfs_log_info(uint8_t n, uint32_t *size, uint8_t *exists);
uint8_t fatfs_active_log(void);

#endif
