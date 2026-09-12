#ifndef LOGGER_H
#define LOGGER_H

int logger_init(void);
int logger_log(const char *line);
unsigned int logger_fail_count(void);
int logger_ready(void);         /* 1 = log jde zapisovat (SD pripojena) */
int logger_update(void);        /* retry SD pripojeni z hlavni smycky */

#endif
