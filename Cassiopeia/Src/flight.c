#include "flight.h"
#include "bme280.h"
#include "bno055.h"
#include "pca9685.h"
#include "stabilization.h"
#include "serial_monitor.h"
#include "lora.h"
#include "alarm.h"
#include "status_report.h"
#include "stm32h7xx_hal.h"
#include <math.h>

/* Letovy FSM.
 *
 *  IDLE        - mimo let
 *  BOOT_LEVEL/BOOT_HOLD are retained for compatibility but are not entered
 *  automatically.  PRE_LAUNCH is entered only by an explicit flight_start()
 *  authorization.
 *                (liftoff trhnuti BNO055), pak ASCENT
 *  ASCENT      - vzestup: stabilizace aktivni, sleduje apogeum
 *  APOGEE      - padak vystrelen (kanal 4 = PCA9685_SERVO_PARACHUTE)
 *  DESCENT     - sestup
 *  LANDED      - pristani
 *
 *  Apogeum (primarni): pokles vysky od maxima o APOGEE_DROP_M.
 *  Apogeum (fallback BNO055):
 *    - raketa se preklopi (tilt > APOGEE_TILT_DEG)
 *
 *  Po detekci apogea zustava stav APOGEE po dobu APOGEE_HOLD_MS
 *  (aby ho zaznamenala telemetrie), pak prechazi na DESCENT.
 *
 *  Liftoff: |acc| > LIFTOFF_G po dobu LIFTOFF_SAMPLES vzorku.
 */

#define LIFTOFF_G          3.0f
#define LIFTOFF_SAMPLES    3
#define APOGEE_DROP_M      5.0f
#define APOGEE_TILT_DEG    70.0f
#define APOGEE_CHUTE_DEG   60
#define APOGEE_HOLD_MS     2000
#define LANDED_ALT_M       10.0f
#define LANDED_SETTLE_MS   3000

#define BOOT_LEVEL_MS      15000   /* prvnich 15 s: vyrovnavani na rampe */
#define BOOT_HOLD_MS       30000   /* do 30 s: klid, raketa stoji na klapkach */

#define PRE_LAUNCH_SAMPLE_MS 5
#define ASCENT_SAMPLE_MS     20

static FlightState state = FLIGHT_IDLE;
static float ground_press = 1013.25f;
static float max_alt = 0;
static uint32_t last_sample = 0;
static uint32_t landed_tick = 0;
static uint32_t apogee_tick = 0;
static uint8_t liftoff_count = 0;
static uint32_t boot_tick = 0;

static float altitude_from_press(float press_hpa)
{
    return 44330.0f * (1.0f - (float)pow((double)(press_hpa / ground_press), 0.1903));
}

static float acc_magnitude(const int16_t *acc)
{
    float ax = acc[0] / 1000.0f;
    float ay = acc[1] / 1000.0f;
    float az = acc[2] / 1000.0f;
    return (float)sqrt(ax * ax + ay * ay + az * az);
}

void flight_init(void)
{
    state = FLIGHT_IDLE;
    max_alt = 0;
    last_sample = 0;
    landed_tick = 0;
    liftoff_count = 0;
    ground_press = 0.0f;
    boot_tick = 0;

    if (bme280_capture_ground_pressure() == 0)
        (void)bme280_ground_pressure(&ground_press);
    else
        master_alarm_set(MASTER_BME280);

    stabilization_disengage();
    serial_puts("flight: IDLE - launch authorization required\r\n");
}

void flight_start(void)
{
    if (state != FLIGHT_IDLE || master_alarm_count() > 0 || !bno055_flight_ready() ||
        bme280_ground_pressure(&ground_press) != 0)
    {
        master_alarm_set(!bno055_flight_ready() ? MASTER_IMU : MASTER_BME280);
        status_report_event("MASTER", STATUS_CODE_MASTER(!bno055_flight_ready() ? MASTER_IMU : MASTER_BME280), "flight-start-blocked");
        return;
    }
    state = FLIGHT_PRE_LAUNCH;
    max_alt = 0;
    last_sample = HAL_GetTick();
    landed_tick = 0;
    liftoff_count = 0;
    boot_tick = 0;

    float p = 0;
    if (bme280_read(0, 0, &p) == 0 && p > 100.0f)
        ground_press = p;

    stabilization_disengage();
    serial_puts("flight: PRE_LAUNCH authorized - waiting for liftoff\r\n");
}

void flight_abort(void)
{
    state = FLIGHT_IDLE;
    max_alt = 0;
    last_sample = 0;
    landed_tick = 0;
    liftoff_count = 0;
    boot_tick = 0;
    stabilization_disengage();
    serial_puts("flight: ABORT - reset to IDLE\r\n");
}

FlightState flight_state(void)
{
    return state;
}

static void flight_deploy(void)
{
    serial_puts("flight: APOGEE - parachute deploy\r\n");
    pca9685_set_servo_deg(PCA9685_SERVO_PARACHUTE, APOGEE_CHUTE_DEG);
    stabilization_disengage();
    lora_send((const uint8_t *)"APOGEE", 6);   /* event hned na zemi */
    state = FLIGHT_APOGEE;
    apogee_tick = HAL_GetTick();
}

void flight_deploy_chute(void)
{
    if (state == FLIGHT_IDLE || state == FLIGHT_LANDED)
        return;
    if (state == FLIGHT_BOOT_LEVEL || state == FLIGHT_BOOT_HOLD)
        return;
    if (state != FLIGHT_APOGEE)
        flight_deploy();
}

static void flight_pre_launch(uint32_t now)
{
    int16_t acc[3];
    if (bno055_read(acc, 0, 0) != 0)
        return;

    float mag = acc_magnitude(acc);
    if (mag > LIFTOFF_G)
    {
        if (++liftoff_count >= LIFTOFF_SAMPLES)
        {
            state = FLIGHT_ASCENT;
            liftoff_count = 0;
            serial_puts("flight: ASCENT (liftoff) - stabilization on\r\n");
            lora_send((const uint8_t *)"LIFTOFF", 7);   /* event hned na zemi */
            stabilization_engage();
        }
    }
    else
    {
        liftoff_count = 0;
    }
}

static void flight_ascent(uint32_t now, float alt)
{
    int16_t acc[3];
    float mag = 0;

    if (bno055_read(acc, 0, 0) == 0)
    {
        mag = acc_magnitude(acc);
    }

    if (mag > LIFTOFF_G)
    {
        /* stale pod vyraznym tahem - nehlede apogeum */
        max_alt = alt;
        return;
    }

    /* primarni apogeum: pokles vysky */
    if (max_alt > 0 && alt < max_alt - APOGEE_DROP_M)
    {
        serial_puts("flight: APOGEE (pressure) @ ");
        print_unsigned((unsigned int)max_alt);
        serial_puts(" m\r\n");
        flight_deploy();
        return;
    }

    /* fallback BNO055: raketa se preklopila (tilt) */
    if (mag > 0.8f && mag < 1.5f)
    {
        float c = (acc[2] / 1000.0f) / mag;
        if (c > 1.0f) c = 1.0f;
        if (c < -1.0f) c = -1.0f;
        float tilt = (float)acos((double)c) * 180.0f / 3.14159265f;
        if (tilt > APOGEE_TILT_DEG)
        {
            serial_puts("flight: APOGEE (overturn)\r\n");
            flight_deploy();
            return;
        }
    }

    /* POZN.: fallback "prudke zpomaleni" (|acc| < APOGEE_DECEL_G) byl
       odstranen - po vyhoreni motoru klesne zrychleni okamzite na ~0 g
       jeste v letu pred apogeem, takze by padak odpalil pri vyhoreni,
       ne na apogeu. Spolehliva detekce apogea je pokles vysky vyse. */
}

static void flight_descent(uint32_t now, float alt)
{
    if (alt <= LANDED_ALT_M)
    {
        if (landed_tick == 0)
            landed_tick = now;
        else if (now - landed_tick >= LANDED_SETTLE_MS)
        {
            state = FLIGHT_LANDED;
            serial_puts("flight: LANDED\r\n");
            lora_send((const uint8_t *)"LANDED", 6);   /* event hned na zemi */
        }
    }
    else
    {
        landed_tick = 0;
    }
}

void flight_update(void)
{
    uint32_t now = HAL_GetTick();

    if (state == FLIGHT_BOOT_LEVEL)
    {
        if (now - boot_tick >= BOOT_LEVEL_MS)
        {
            state = FLIGHT_BOOT_HOLD;
            stabilization_disengage();
            pca9685_set_servo_deg(PCA9685_SERVO_STAB1, 0);
            pca9685_set_servo_deg(PCA9685_SERVO_STAB2, 0);
            pca9685_set_servo_deg(PCA9685_SERVO_STAB3, 0);
            pca9685_set_servo_deg(PCA9685_SERVO_STAB4, 0);
            pca9685_set_servo_deg(PCA9685_SERVO_PARACHUTE, 0);
            serial_puts("flight: BOOT_HOLD - 15s klid, vsechna serva na 0 (ready)\r\n");
        }
        return;
    }

    if (state == FLIGHT_BOOT_HOLD)
    {
        if (now - boot_tick >= BOOT_HOLD_MS)
        {
            state = FLIGHT_PRE_LAUNCH;
            last_sample = now;
            liftoff_count = 0;
            serial_puts("flight: LAUNCH READY - waiting for first movement\r\n");
        }
        return;
    }

    if (state == FLIGHT_IDLE || state == FLIGHT_LANDED)
        return;

    uint32_t period = (state == FLIGHT_PRE_LAUNCH) ? PRE_LAUNCH_SAMPLE_MS : ASCENT_SAMPLE_MS;
    if (now - last_sample < period)
        return;
    last_sample = now;

    if (state == FLIGHT_PRE_LAUNCH)
    {
        flight_pre_launch(now);
        return;
    }

    float press = 0;
    if (bme280_read(0, 0, &press) != 0 || press <= 100.0f)
        return;

    float alt = altitude_from_press(press);
    if (alt > max_alt)
        max_alt = alt;

    switch (state)
    {
    case FLIGHT_ASCENT:
        flight_ascent(now, alt);
        break;
    case FLIGHT_APOGEE:
        if (now - apogee_tick >= APOGEE_HOLD_MS)
        {
            state = FLIGHT_DESCENT;
            serial_puts("flight: DESCENT\r\n");
        }
        break;
    case FLIGHT_DESCENT:
        flight_descent(now, alt);
        break;
    default:
        break;
    }
}
