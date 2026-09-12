#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdint.h>

#define MOD_STATUS_OK        0
#define MOD_STATUS_INIT_ERR  1
#define MOD_STATUS_TEST_ERR  2

typedef int (*mod_init_fn)(void);
typedef int (*mod_self_test_fn)(void);

typedef struct
{
    const char *name;
    mod_init_fn init;
    mod_self_test_fn self_test;
    uint8_t status;   /* MOD_STATUS_* */
    uint8_t present;  /* 1 = zarizeni nalezeno */
    int alarm;        /* AlarmType nebo MasterAlarmType; -1 = zadny */
    int master;       /* 1 = master alarm, 0 = bezny alarm */
    uint8_t optional; /* 1 = nepovinny modul - jen informativni vypis */
} ModuleDef;

void supervisor_init(void);
void supervisor_self_test(void);
void supervisor_report(void);
void supervisor_warning_update(void);
uint8_t supervisor_module_status(const char *name);
void supervisor_rtc_manual_sync(void);  /* RTC nastaveno ze zeme (SETTIME) */

#endif
