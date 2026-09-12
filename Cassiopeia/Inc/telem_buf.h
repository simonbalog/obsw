#ifndef TELEM_BUF_H
#define TELEM_BUF_H

#include <stdint.h>

void     telem_buf_init(void);
void     telem_buf_clear(void);
int      telem_buf_store(const char *line);
uint32_t telem_buf_count(void);
uint32_t telem_buf_evicted(void);
uint32_t telem_buf_fill_pct(void);
void     telem_buf_dump_start(void);
int      telem_buf_dump_step(void);
int      telem_buf_dump_active(void);

#endif