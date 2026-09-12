#include "orientation.h"
#include "bno055.h"
#include "serial_monitor.h"
#include "stm32h7xx_hal.h"
#include <math.h>

/* Orientace a kalibrace zaroveni pri kazdem bootu.
 *
 *  Po resetu se chvili predpoklada klid: z akcelerometru (gravitacni vektor)
 *  se ulozi referencni smer "nahoru" a z gyroskopu bias (bno055_gyro_bias_set,
 *  ktery pak odecitaji vsechna cteni gyra vcetne ISR stabilizace).
 *  Od te chvile se kazde okno hlavni smycky pocita:
 *    tilt_deg = uhel mezi aktualnim zrychlenim a referencni "svislici"
 *               (0 = svisle nahoru, 180 = svisle dolu),
 *    acc_mg   = velikost zrychleni (1000 = 1 g),
 *    smer     = UP / DOWN / MISS <HH>H <deg>, cifernikove hodiny
 *               videho z ocasu rakety (12H = nahoru).
 *
 *  Cifernik: 12H = horni servo (STAB2), 3H = prave (STAB1),
 *  6H = dolni (STAB4), 9H = leve (STAB3).
 *
 *  Montaz senzoru: BNO055 sedi v raketach obvykle podelnou osou Z.
 *  Smer "12H" (vychozi: +h2 osa, tj. v typicke montazi +Y senzoru)
 *  nemusi sedet na tvoji desku. Po prvnim testu na rampe nastav
 *  ORIENT_CLOCK_DEG_OFFSET / ORIENT_CLOCK_FLIP tak, aby MISS <HH>H
 *  ukazovalo na servo, kterym zpusobem se raketa skutecne naklani.
 */

#define ORIENT_CALIB_MS     1500   /* doba sberu vzorku po bootu */
#define ORIENT_LEVEL_DEG     8     /* <= = "rovne" -> UP/DOWN */
#define ORIENT_MIN_SAMPLES   5

/* nulove nastaveni = vychozi montaz (viz komentar vyse) */
#define ORIENT_CLOCK_DEG_OFFSET   0
#define ORIENT_CLOCK_FLIP         0

static int calib_done = 0;
static uint32_t calib_start = 0;
static int calib_cnt = 0;
static int32_t sum_acc[3] = { 0, 0, 0 };
static int32_t sum_gyr[3] = { 0, 0, 0 };

static int gyro_stream = 0;   /* 0 = OFF (vychozi), 1 = ON pres GYROON */

static float up[3];   /* referencni "svislice" (smer nahoru v tele) */
static float h1[3];   /* rovinna osa 1 (kolmo na up) */
static float h2[3];   /* rovinna osa 2 (up x h1) */

static orientation_t cur;

static void vec_norm(const float v[3], float out[3])
{
    float l = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (l < 1e-6f)
    {
        out[0] = 0; out[1] = 0; out[2] = 1;
        return;
    }
    out[0] = v[0] / l;
    out[1] = v[1] / l;
    out[2] = v[2] / l;
}

static void vec_cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static float vec_dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* Smer naklonu rakety jako cifernikova hodina (1..12).
 *  p1/p2 = horizontalni projekce gravitace na osy h1/h2. Raketa se
 *  naklani opacnym smerem, nez kam ukazuje horizontalni gravitace.
 *  Uhel se pocita od +h2 (12H) po smeru hodin k +h1 (3H) - viz
 *  ORIENT_CLOCK_DEG_OFFSET / ORIENT_CLOCK_FLIP pro montazni kalibraci. */
static int clock_hour(float p1, float p2)
{
    float ax = -p1;
    float ay = -p2;
    float ang = (float)atan2((double)ax, (double)ay) * 180.0f / 3.14159265f;
    ang += (float)ORIENT_CLOCK_DEG_OFFSET;
    if (ORIENT_CLOCK_FLIP)
        ang = 360.0f - ang;
    if (ang < 0.0f)
        ang += 360.0f;
    if (ang >= 360.0f)
        ang -= 360.0f;
    int h = (int)((ang + 15.0f) / 30.0f) % 12;  /* 0..11 */
    return (h == 0) ? 12 : h;
}

void orientation_init(void)
{
    calib_done = 0;
    calib_cnt = 0;
    calib_start = HAL_GetTick();
    sum_acc[0] = sum_acc[1] = sum_acc[2] = 0;
    sum_gyr[0] = sum_gyr[1] = sum_gyr[2] = 0;
    bno055_gyro_bias_reset();
}

void orientation_update(void)
{
    int16_t acc[3], gyr[3];

    if (!calib_done)
    {
        if (bno055_read(acc, gyr, 0) != 0)
            return;

        sum_acc[0] += acc[0]; sum_acc[1] += acc[1]; sum_acc[2] += acc[2];
        sum_gyr[0] += gyr[0]; sum_gyr[1] += gyr[1]; sum_gyr[2] += gyr[2];
        calib_cnt++;

        if ((HAL_GetTick() - calib_start < ORIENT_CALIB_MS) ||
            calib_cnt < ORIENT_MIN_SAMPLES)
            return;

        /* gyro bias = prumer z klidoveho gyra */
        int16_t bias[3];
        bias[0] = (int16_t)((sum_gyr[0] + calib_cnt / 2) / calib_cnt);
        bias[1] = (int16_t)((sum_gyr[1] + calib_cnt / 2) / calib_cnt);
        bias[2] = (int16_t)((sum_gyr[2] + calib_cnt / 2) / calib_cnt);
        bno055_gyro_bias_set(bias);

        /* referencni svislice z prumeru akcelerometru (raketa stoji nahoru) */
        float a[3];
        a[0] = (float)(sum_acc[0] / calib_cnt);
        a[1] = (float)(sum_acc[1] / calib_cnt);
        a[2] = (float)(sum_acc[2] / calib_cnt);
        vec_norm(a, up);

        /* rovinne osy pro smer naklonu: h1 = osa X projekci (neni-li
           rovnobezna s up), h2 = up x h1 */
        float ax[3] = { 1.0f, 0.0f, 0.0f };
        float dd = vec_dot(up, ax);
        if (dd < -0.9f || dd > 0.9f)
        {
            ax[0] = 0.0f; ax[1] = 1.0f; ax[2] = 0.0f;
        }
        float proj[3];
        proj[0] = ax[0] - up[0] * dd;
        proj[1] = ax[1] - up[1] * dd;
        proj[2] = ax[2] - up[2] * dd;
        vec_norm(proj, h1);
        vec_cross(up, h1, h2);

        calib_done = 1;

        serial_puts("orientation: calib OK (gyr bias=");
        print_int(bias[0]); serial_puts(","); print_int(bias[1]); serial_puts(","); print_int(bias[2]);
        serial_puts("; up=");
        print_int((int)(up[0] * 1000.0f)); serial_puts(",");
        print_int((int)(up[1] * 1000.0f)); serial_puts(",");
        print_int((int)(up[2] * 1000.0f));
        serial_puts(")\r\n");
        return;
    }

    /* ziva orientace: staci akcelerometr */
    if (bno055_read(acc, 0, 0) != 0)
        return;

    float v[3];
    v[0] = (float)acc[0];
    v[1] = (float)acc[1];
    v[2] = (float)acc[2];
    float al = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (al < 1e-3f)
        return;

    /* naklon vuci referencni svislici */
    float d = (v[0] * up[0] + v[1] * up[1] + v[2] * up[2]) / al;
    if (d < -1.0f) d = -1.0f;
    if (d >  1.0f) d =  1.0f;
    float tilt = (float)acos((double)d) * 180.0f / 3.14159265f;

    cur.acc_mg = (int)(al + 0.5f);
    cur.tilt_deg = (int)(tilt + 0.5f);

    if (tilt <= ORIENT_LEVEL_DEG)
    {
        cur.dir = 'U';
        cur.is_miss = 0;
        cur.clock_h = 0;
        cur.miss_deg = 0;
    }
    else if (tilt >= 180.0f - ORIENT_LEVEL_DEG)
    {
        cur.dir = 'D';
        cur.is_miss = 0;
        cur.clock_h = 0;
        cur.miss_deg = 0;
    }
    else
    {
        /* horizontalni projekce (kolmo na svislici) -> smer naklonu */
        float hp[3];
        hp[0] = v[0] - up[0] * d * al;
        hp[1] = v[1] - up[1] * d * al;
        hp[2] = v[2] - up[2] * d * al;
        float p1 = hp[0] * h1[0] + hp[1] * h1[1] + hp[2] * h1[2];
        float p2 = hp[0] * h2[0] + hp[1] * h2[1] + hp[2] * h2[2];
        cur.dir = 0;
        cur.is_miss = 1;
        cur.clock_h = clock_hour(p1, p2);
        cur.miss_deg = cur.tilt_deg;
    }
}

int orientation_ready(void)
{
    return calib_done;
}

int orientation_sample(orientation_t *o)
{
    if (!calib_done)
        return -1;
    if (o)
        *o = cur;
    return 0;
}

void orientation_gyro_stream_set(int on)
{
    gyro_stream = on ? 1 : 0;
}

int orientation_gyro_stream_get(void)
{
    return gyro_stream;
}

void orientation_rezero(void)
{
    calib_done = 0;
    calib_cnt = 0;
    calib_start = HAL_GetTick();
    sum_acc[0] = sum_acc[1] = sum_acc[2] = 0;
    sum_gyr[0] = sum_gyr[1] = sum_gyr[2] = 0;
    bno055_gyro_bias_reset();
    serial_puts("orientation: rezero started (drz let rakety ~1.5 s)\r\n");
}