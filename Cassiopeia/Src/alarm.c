#include "alarm.h"

static unsigned int warnings = 0;
static unsigned int alarms = 0;
static unsigned int master_alarms = 0;

#define RCC_AHB4ENR  (*(volatile unsigned int*)(0x580244E0))
#define GPIOB_MODER  (*(volatile unsigned int*)(0x58020400))
#define GPIOB_BSRR   (*(volatile unsigned int*)(0x58020418))
#define GPIOE_MODER  (*(volatile unsigned int*)(0x58021000))
#define GPIOE_BSRR   (*(volatile unsigned int*)(0x58021018))

void alarm_init(void)
{
    RCC_AHB4ENR |= (1 << 1) | (1 << 4);
    GPIOB_MODER = (GPIOB_MODER & ~((3 << 0) | (3 << 28))) | ((1 << 0) | (1 << 28));
    GPIOE_MODER = (GPIOE_MODER & ~(3 << 2)) | (1 << 2);
}

void warning_set(WarningType w)
{
    warnings |= (1 << w);
}

void warning_clear(WarningType w)
{
    warnings &= ~(1 << w);
}

int warning_get(WarningType w)
{
    return (warnings >> w) & 1;
}

void alarm_set(AlarmType a)
{
    alarms |= (1 << a);
}

void alarm_clear(AlarmType a)
{
    alarms &= ~(1 << a);
}

int alarm_get(AlarmType a)
{
    return (alarms >> a) & 1;
}

void master_alarm_set(MasterAlarmType a)
{
    master_alarms |= (1 << a);
}

void master_alarm_clear(MasterAlarmType a)
{
    master_alarms &= ~(1 << a);
}

int master_alarm_get(MasterAlarmType a)
{
    return (master_alarms >> a) & 1;
}

int warning_count(void)
{
    unsigned int x = warnings;
    int c = 0;
    while (x) { c += x & 1; x >>= 1; }
    return c;
}

unsigned int warning_mask(void)
{
    return warnings;
}

unsigned int alarm_mask(void)
{
    return alarms;
}

unsigned int master_alarm_mask(void)
{
    return master_alarms;
}

int alarm_count(void)
{
    unsigned int x = alarms;
    int c = 0;
    while (x) { c += x & 1; x >>= 1; }
    return c;
}

int master_alarm_count(void)
{
    unsigned int x = master_alarms;
    int c = 0;
    while (x) { c += x & 1; x >>= 1; }
    return c;
}

void led_set(int ld1, int ld2, int ld3)
{
    if (ld1) GPIOB_BSRR = (1 << 0);
    else     GPIOB_BSRR = (1 << (0 + 16));

    if (ld2) GPIOE_BSRR = (1 << 1);
    else     GPIOE_BSRR = (1 << (1 + 16));

    if (ld3) GPIOB_BSRR = (1 << 14);
    else     GPIOB_BSRR = (1 << (14 + 16));
}

void alarm_update_leds(void)
{
    if (master_alarms)
    {
        /* master alarm: cervena LD3 */
        led_set(0, 0, 1);
    }
    else if (alarms)
    {
        /* alarm: oranzova LD2 */
        led_set(0, 1, 0);
    }
    else if (warnings)
    {
        /* jen warningy: zelena LD1 */
        led_set(1, 0, 0);
    }
    else
    {
        /* OK */
        led_set(0, 0, 0);
    }
}