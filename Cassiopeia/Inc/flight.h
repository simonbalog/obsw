#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdint.h>

typedef enum {
    FLIGHT_IDLE = 0,     /* mimo let */
    FLIGHT_PRE_LAUNCH,   /* armed, ceka na trhnuti (liftoff) */
    FLIGHT_ASCENT,       /* vzestup, stabilizace aktivni */
    FLIGHT_APOGEE,       /* apogeum - padak ven */
    FLIGHT_DESCENT,      /* sestup */
    FLIGHT_LANDED,       /* pristani */
    FLIGHT_BOOT_LEVEL,   /* boot: prvnich 15 s - vyrovnavani na rampe */
    FLIGHT_BOOT_HOLD     /* boot: 15-30 s - klid, raketa stoji na klapkach */
} FlightState;

void flight_init(void);
void flight_start(void);
void flight_update(void);
void flight_abort(void);
void flight_deploy_chute(void);
FlightState flight_state(void);

#endif
