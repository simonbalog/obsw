#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

void safety_init(void);
void safety_update(void);
uint8_t safety_triggered(void);

#endif
