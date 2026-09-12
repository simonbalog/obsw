#ifndef POWER_H
#define POWER_H

#include <stdint.h>

int power_init(void);
int power_self_test(void);
int power_read_battery_mv(uint16_t *mv);
int power_read_5v_mv(uint16_t *mv);

#endif
