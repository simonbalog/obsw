#ifndef STABILIZATION_H
#define STABILIZATION_H

#include <stdint.h>

/* perioda regulaeni smycky stabilizace - TIM6 tick (50 Hz) */
#define STAB_LOOP_MS 20

void stabilization_init(void);
void stabilization_update(void);
void stabilization_engage(void);
void stabilization_level_engage(void);
void stabilization_disengage(void);
uint8_t stabilization_active(void);
void stabilization_test_channel(int8_t ch);   /* ch<0 = vsechna serva */
int8_t stabilization_test_channel_get(void);
int stabilization_command_get(uint8_t channel, int16_t *degrees);

#endif
