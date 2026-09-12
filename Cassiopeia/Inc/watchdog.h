#ifndef WATCHDOG_H
#define WATCHDOG_H

void watchdog_init(void);
void watchdog_refresh(void);
int watchdog_self_test(void);

#endif
