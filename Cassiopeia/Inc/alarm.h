#ifndef ALARM_H
#define ALARM_H

/* Tri urovne vyznamu:
 *
 *   Warning      - informativni, provozni stav; neblokuje nic, samo se
 *                  cisti, nezabrani startu.
 *   Alarm        - subsystem degradovany / neprovozni; start je mozny
 *                  ale rizikovy, indikace drzi dokud stav trva.
 *   Master alarm - let neni bezpecny; blokuje odpocet (countdown),
 *                  spousti safety akce.
 */

typedef enum {
    WRN_GPS_NO_FIX    = 0,  /* GPS nema fix po WRN_GPS_NOFIX_MS od bootu */
    WRN_GPS_LOW_SAT   = 1,  /* fix je, ale malo satelitu (< 4) */
    WRN_RTC_INVALID   = 2,  /* RTC nevalidni / nelze cist cas */
    WRN_BATTERY_LOW   = 3,  /* baterie pod WRN_BATTERY_LOW_MV */
    WRN_TEMP_OUT      = 4,  /* teplota BME280 mimo rozsah */
    WRN_LORA_TX       = 5,  /* rostouci pocet LoRa TX failu */
    WRN_SD_RETRY      = 6,  /* SD zapis presel pres retry / FAT2 fallback */
    WRN_SERVO_ERR     = 7,  /* serva mimo rozsah / chyba polohy */
    WRN_IMU_CAL       = 8,  /* BNO055 neni plne kalibrovany (sys < 3) */
    WRN_COUNT
} WarningType;

typedef enum {
    ALARM_BME280    = 0,
    ALARM_BNO055    = 1,
    ALARM_GPS       = 2,
    ALARM_LORA      = 3,
    ALARM_BATTERY   = 4,
    ALARM_PYRO      = 5,
    ALARM_SD_CARD   = 6,
    ALARM_LOGGING   = 7,
    ALARM_COUNT
} AlarmType;

typedef enum {
    MASTER_CPU      = 0,
    MASTER_MEMORY   = 1,
    MASTER_STACK    = 2,
    MASTER_CLOCK    = 3,
    MASTER_FLASH    = 4,
    MASTER_IMU      = 5,
    MASTER_SERVO    = 6,
    MASTER_SD       = 7,
    MASTER_LAUNCH   = 8,
    MASTER_BME280   = 9,   /* tlakomer = primarni detekce apogea */
    MASTER_POWER    = 10,  /* kriticke napeti baterie za letu */
    MASTER_COUNT
} MasterAlarmType;

void alarm_init(void);

void warning_set(WarningType w);
void warning_clear(WarningType w);
int warning_get(WarningType w);

void alarm_set(AlarmType a);
void alarm_clear(AlarmType a);
int alarm_get(AlarmType a);

void master_alarm_set(MasterAlarmType a);
void master_alarm_clear(MasterAlarmType a);
int master_alarm_get(MasterAlarmType a);

int warning_count(void);
int alarm_count(void);
int master_alarm_count(void);

/* masky aktivnich kodu (bit = index enumu) - pro telemetrii */
unsigned int warning_mask(void);
unsigned int alarm_mask(void);
unsigned int master_alarm_mask(void);

void alarm_update_leds(void);
void led_set(int ld1, int ld2, int ld3);

#endif
