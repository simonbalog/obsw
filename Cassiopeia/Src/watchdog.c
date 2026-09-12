#include "watchdog.h"
#include "stm32h7xx_hal.h"

/* Hlidaci pes IWDG (LSI ~32 kHz).
 *
 *  - timeout ~500 ms (prescaler 64 -> 500 Hz, reload 250)
 *  - hlavni smycka musi volat watchdog_refresh(), jinak reset MCU
 *  - zaseknuty boot se tim restartuje rychle (kratka doba, nez se karta
 *    muze znovu nasyncovat CMD0/ACMD41)
 */

#define IWDG_PRESCALER_VAL IWDG_PRESCALER_64
#define IWDG_RELOAD_VAL    250

static IWDG_HandleTypeDef hiwdg;
static uint8_t  active = 0;

void watchdog_init(void)
{
    hiwdg.Instance = IWDG1;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_VAL;
    hiwdg.Init.Reload = IWDG_RELOAD_VAL;
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) == HAL_OK)
        active = 1;
}

void watchdog_refresh(void)
{
    if (!active)
        return;
    HAL_IWDG_Refresh(&hiwdg);
}

int watchdog_self_test(void) { return active ? 0 : -1; }
