#include "safety.h"
#include "serial_monitor.h"
#include "alarm.h"
#include "bme280.h"
#include "bno055.h"
#include "pca9685.h"
#include "stm32h7xx_hal.h"

/* Safety watch: monitoruje tlak a IMU pro predcasny odpal.
 *
 *  - prudky pokles tlaku  (spoustaci sekvence)
 *  - prudke zrychleni      (IMU vysoke G)
 *
 * Pokud nektery prekroci threshold, nastavi MASTER_LAUNCH,
 * rozsviti LED a spusti stabilizaci (serva).
 */

#define SAFETY_SAMPLE_MS      50
#define SAFETY_PRESS_DROP_HPA 10.0f   /* hPa za vzorek -> vypusteni tlaku */
#define SAFETY_ACCEL_G        2.5f    /* vetsi nez 2.5g -> odpali */

static uint8_t triggered = 0;
static uint32_t last_sample = 0;
static float last_press = 0;
static uint8_t have_press = 0;

static void safety_trigger(const char *why)
{
    triggered = 1;
    master_alarm_set(MASTER_LAUNCH);
    serial_puts("SAFETY: trigger - ");
    serial_puts(why);
    serial_puts("\r\n");
    alarm_update_leds();
}

void safety_init(void)
{
    triggered = 0;
    last_sample = 0;
    have_press = 0;
}

uint8_t safety_triggered(void)
{
    return triggered;
}

void safety_update(void)
{
    if (triggered)
        return;

    uint32_t now = HAL_GetTick();
    if (now - last_sample < SAFETY_SAMPLE_MS)
        return;
    last_sample = now;

    /* tlak */
    float press = 0;
    if (bme280_read(0, 0, &press) == 0)
    {
        if (have_press)
        {
            if (press < last_press - SAFETY_PRESS_DROP_HPA)
            {
                safety_trigger("pressure drop");
                return;
            }
        }
        last_press = press;
        have_press = 1;
    }

    /* IMU zrychleni */
    int16_t acc[3];
    if (bno055_read(acc, 0, 0) == 0)
    {
        float ax = acc[0] / 1000.0f;
        float ay = acc[1] / 1000.0f;
        float az = acc[2] / 1000.0f;
        float mag2 = ax * ax + ay * ay + az * az;
        if (mag2 > SAFETY_ACCEL_G * SAFETY_ACCEL_G)
        {
            safety_trigger("high accel");
            return;
        }
    }
}
