#include "stabilization.h"
#include "pid.h"
#include "bno055.h"
#include "pca9685.h"
#include "bus_i2c.h"
#include "stm32h7xx_hal.h"
#include <math.h>

/* Stabilizace pomoci 4 serv (aerodynamicke plochy / kridelka).
 *
 *  - cte uhlove rychlosti z BNO055 (gyro, dps)
 *  - PID snazi se drzet 0 deg/s na kazde ose
 *  - rozlozeni ploch (pohled od ocasu rakety):
 *        STAB1 (ch0): vpravo   ->  +roll  +yaw
 *        STAB2 (ch1): nahore   ->  +roll  +pitch
 *        STAB3 (ch2): vlevo    ->  -roll  -yaw
 *        STAB4 (ch3): dole     ->  -roll  -pitch
 *
 *    Roll: vsechny 4 plochy diferenciane (max. moment proti rotaci,
 *          aby kamera netocila stale dokola).
 *    Pitch (naklon): horni/dolni par STAB2/STAB4.
 *    Yaw:            pravy/levy par STAB1/STAB3.
 *
 *  Znamenka odpovidaji obvykle zakrytovane geometrii (plocha se
 *  vychyli proti smeru rotace). Pri letovem testu, kdyz by PID
 *  stabilizaci rozkymital (kladna zpetna vazba), znamenko otoct.
 *
 *  LEVELING MODE (na zemi, boot):
 *    - misto gyro rychlosti se vyrovnava podle smeru gravitace
 *      z akcelerometru (raketa stoji na rampe)
 *    - serva reaguji na naklon -> na zemi je videt, ze raketa
 *      nestoji uplne rovne (plochy se vychyli proti naklonu)
 *    - po 15 s se vypne (flight.c -> BOOT_HOLD)
 *
 *  REAL-TIME PRIORITA:
 *    stabilization_update() se vola z TIM6 preruseni (50 Hz, nejvyssi
 *    priorita), NIC ji v hlavni smycce neblokuje. Jen pokud hlavni
 *    smycka prave drzi I2C linku, se vzorek preskoci (bus_i2c_busy()).
 *
 *  TODO: bez realneho letoveho testu jsou konstanty ladici - upravit
 *        az budou zname tvary ploch a vykon serv.
 */

#define STAB_MAX_DEG   30

#define LEVEL_MAX_DEG  15   /* vyrovnavani na zemi: mala vychylka */

/* clamp dt - kdyz vzorek preskoci (hlavni smycka drzi I2C nebo vetsi zatizeni),
   dt se spocita z HAL_GetTick a omezuje, aby PID nekopal odhad rozdilu */
#define STAB_DT_MIN_S  0.005f
#define STAB_DT_MAX_S  0.050f

static volatile uint8_t active = 0;
static volatile uint8_t leveling = 0;
static volatile int8_t test_channel = -1;   /* >=0 = vystup stabilizace jen na toto servo */
static uint32_t last_tick = 0;

static PidCtrl pid_roll;
static PidCtrl pid_pitch;
static PidCtrl pid_yaw;

static PidCtrl pid_level_pitch;
static PidCtrl pid_level_roll;

static int16_t clamp_srv(float v)
{
    if (v > STAB_MAX_DEG) v = STAB_MAX_DEG;
    if (v < -STAB_MAX_DEG) v = -STAB_MAX_DEG;
    return (int16_t)v;
}

static int16_t clamp_level(float v)
{
    if (v > LEVEL_MAX_DEG) v = LEVEL_MAX_DEG;
    if (v < -LEVEL_MAX_DEG) v = -LEVEL_MAX_DEG;
    return (int16_t)v;
}

void stabilization_init(void)
{
    pid_init(&pid_roll,  0.8f, 0.05f, 0.2f, -STAB_MAX_DEG, STAB_MAX_DEG);
    pid_init(&pid_pitch, 0.8f, 0.05f, 0.2f, -STAB_MAX_DEG, STAB_MAX_DEG);
    pid_init(&pid_yaw,   0.8f, 0.05f, 0.2f, -STAB_MAX_DEG, STAB_MAX_DEG);
    pid_init(&pid_level_pitch, 1.5f, 0.0f, 0.0f, -LEVEL_MAX_DEG, LEVEL_MAX_DEG);
    pid_init(&pid_level_roll,  1.5f, 0.0f, 0.0f, -LEVEL_MAX_DEG, LEVEL_MAX_DEG);
    active = 0;
    leveling = 0;
    last_tick = HAL_GetTick();
}

void stabilization_engage(void)
{
    /* vynulovat stav PID - po disengage/znovu-engage by stara integral/deriv
       zpravila skok na servech */
    pid_reset(&pid_roll);
    pid_reset(&pid_pitch);
    pid_reset(&pid_yaw);
    active = 1;
    leveling = 0;
    last_tick = HAL_GetTick();
}

void stabilization_level_engage(void)
{
    pid_reset(&pid_level_pitch);
    pid_reset(&pid_level_roll);
    active = 1;
    leveling = 1;
    last_tick = HAL_GetTick();
}

void stabilization_disengage(void)
{
    active = 0;
    leveling = 0;
}

uint8_t stabilization_active(void)
{
    return active;
}

/* Testovaci rezim: vystup stabilizace jen na jeden kanal (0-4).
   ch < 0 = normalni provoz (vsechna serva). */
void stabilization_test_channel(int8_t ch)
{
    test_channel = (ch < PCA9685_NUM_SERVOS) ? ch : -1;
}

int8_t stabilization_test_channel_get(void)
{
    return test_channel;
}

/* zapis na servo s ohledem na testovaci kanal.
   _isr varianta = kratky I2C timeout (viz bus_i2c.c), tady se vzdy
   vola z TIM6 ISR, kde se nesmi cekat na linku 100 ms */
static void srv_out(uint8_t ch, int16_t deg)
{
    if (test_channel >= 0 && (uint8_t)test_channel != ch)
        return;
    pca9685_set_servo_deg_isr(ch, deg);
}

static void stabilization_level_update(float dt)
{
    if (bus_i2c_busy())
        return;

    int16_t acc[3];
    if (bno055_read_isr(acc, 0, 0) != 0)
        return;

    /* akcelerometr v mg (1000 = 1 g). Z gravitacniho vektoru se
       urci naklon rakety vuci svislika. */
    float ax = acc[0] / 1000.0f;
    float ay = acc[1] / 1000.0f;
    float az = acc[2] / 1000.0f;

    float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (mag < 0.5f)
        return;

    /* naklon v stupnich kolem os X a Y (raketa stoji na Z) */
    float tilt_pitch = asinf(ax / mag) * 180.0f / 3.14159265f;
    float tilt_roll  = asinf(ay / mag) * 180.0f / 3.14159265f;

    float deg_pitch = pid_update_dt(&pid_level_pitch, 0, tilt_pitch, dt);
    float deg_roll  = pid_update_dt(&pid_level_roll,  0, tilt_roll,  dt);

    /* vychyleni proti naklonu - znamenko je treba pripadne otocit */
    srv_out(PCA9685_SERVO_STAB1, clamp_level(-deg_roll));
    srv_out(PCA9685_SERVO_STAB2, clamp_level(-deg_pitch));
    srv_out(PCA9685_SERVO_STAB3, clamp_level( deg_roll));
    srv_out(PCA9685_SERVO_STAB4, clamp_level( deg_pitch));
}

/* Regulacni smycka - vola se z TIM6 preruseni (kazdych STAB_LOOP_MS).
 * Timer je casova zakladna, dt (pro PID) se meri pres HAL_GetTick. */
void stabilization_update(void)
{
    if (!active)
        return;

    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_tick) * 0.001f;
    if (dt < STAB_DT_MIN_S) dt = STAB_DT_MIN_S;
    if (dt > STAB_DT_MAX_S) dt = STAB_DT_MAX_S;
    last_tick = now;

    if (leveling)
    {
        stabilization_level_update(dt);
        return;
    }

    /* hlavni smycka prave pouziva I2C linku - preskoc tento vzorek,
       dalsi prijde za STAB_LOOP_MS */
    if (bus_i2c_busy())
        return;

    int16_t gyr[3];
    if (bno055_read_isr(0, gyr, 0) != 0)
        return;

    float roll_rate  = gyr[0];   /* dps, osa X */
    float pitch_rate = gyr[1];   /* dps, osa Y */
    float yaw_rate   = gyr[2];   /* dps, osa Z */

    float deg_roll  = pid_update_dt(&pid_roll,  0, roll_rate,  dt);
    float deg_pitch = pid_update_dt(&pid_pitch, 0, pitch_rate, dt);
    float deg_yaw   = pid_update_dt(&pid_yaw,   0, yaw_rate,   dt);

    srv_out(PCA9685_SERVO_STAB1, clamp_srv( deg_roll + deg_yaw));
    srv_out(PCA9685_SERVO_STAB2, clamp_srv( deg_roll + deg_pitch));
    srv_out(PCA9685_SERVO_STAB3, clamp_srv(-deg_roll - deg_yaw));
    srv_out(PCA9685_SERVO_STAB4, clamp_srv(-deg_roll - deg_pitch));
}

/* HAL callback for the TIM6 stabilization interrupt. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
        stabilization_update();
}
