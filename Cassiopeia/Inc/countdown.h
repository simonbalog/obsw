#ifndef COUNTDOWN_H
#define COUNTDOWN_H

#include <stdint.h>

#define COUNTDOWN_DEFAULT_MIN 5

typedef enum {
    COUNTDOWN_STOPPED,
    COUNTDOWN_RUNNING,   /* bezi odpoctavek (check / wave / warning) */
    COUNTDOWN_ARMED      /* T-0 dosazeno */
} CountdownState;

typedef enum {
    COUNTDOWN_PHASE_IDLE,
    COUNTDOWN_PHASE_CHECK,   /* prvnich 30 s - kontrola systemu, LED alarmy */
    COUNTDOWN_PHASE_WAVE,    /* priprava - LED vlnou */
    COUNTDOWN_PHASE_WARNING  /* poslednich 30 s - vyrazne blikani */
} CountdownPhase;

void countdown_init(void);
void countdown_update(void);
void countdown_remote_pause(void);
void countdown_remote_resume(void);
void countdown_remote_start(void);
void countdown_remote_add_min(void);
void countdown_remote_test_toggle(void);
void countdown_abort(void);
CountdownState countdown_state(void);
CountdownPhase countdown_phase(void);
uint32_t countdown_remaining_s(void);
uint8_t countdown_test_mode(void);

#endif
