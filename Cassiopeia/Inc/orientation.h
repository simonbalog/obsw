#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <stdint.h>

/* Orientace podle BNO055 (akcelerometr = gravitacni vektor).
 *
 *  - pri kazdem bootu se IMU "nuluje": prvnich ~1.5 s se predpoklada,
 *    ze raketa stoji rovne (nosem nahoru) a je v klidu. Z akcelerometru
 *    se ulozi referencni vektor "nahoru" a z gyroskopu se odebere bias
 *    (bno055_gyro_bias_set), takze klidove gyro je presne 0.
 *  - tilt = naklon od osy "nahoru" v stupnich (0 = svisle nahoru,
 *    90 = vodorovne, 180 = hlavou dolu).
 *  - smer = na kterou stranu raketa nakloni, vyjadreny cifernikovou
 *    hodinou videho z ocasu rakety: UP / DOWN, jinak MISS <HH>H <deg>.
 *    12H = nahoru (servo STAB2), 3H = vpravo (STAB1), 6H = dolu (STAB4),
 *    9H = vlevo (STAB3); ostatni hodiny = mezi-tele serv.
 */

typedef struct {
    int   tilt_deg;     /* naklon od "nahoru", 0..180 */
    int   acc_mg;       /* velikost zrychleni v mg (1000 = 1 g) */
    char  dir;          /* 'U'=UP, 'D'=DOWN, 0 = MISS */
    int   is_miss;      /* 1 = naklonena, 0 = UP/DOWN */
    int   clock_h;      /* hodina 1..12 pro MISS (12H = nahoru / STAB2) */
    int   miss_deg;     /* naklon pro MISS (UP/DOWN = 0) */
} orientation_t;

void orientation_init(void);
int  orientation_ready(void);            /* 0 = jeste bezim kalibrace */
void orientation_update(void);           /* vola se z hlavni smycky */
int  orientation_sample(orientation_t *o);   /* aktualni orientace, -1 = neni */

/* zivy stream Y;... pres LoRa se posila JEN kdyz je zapnuty (GYROON).
   Jiny skutek by ukousl LoRa kanal pro ostatni telemetrii. */
void orientation_gyro_stream_set(int on);
int  orientation_gyro_stream_get(void);

/* prekalibrace za behu (GZERO): NOVY bias gyra + nova referencia
   "nahoru" z aktualni polohy. ~1.5 s treba nehybat raketou. */
void orientation_rezero(void);

#endif