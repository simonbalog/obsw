#ifndef PREFLIGHT_H
#define PREFLIGHT_H

#include <stdint.h>

/* Run the bounded sensor/power checklist.  This is diagnostic only: it never
 * clears an alarm, changes flight state, or enables an actuator. */
int preflight_run(void);
void preflight_report_lora(void);
int preflight_has_result(void);
int preflight_passed(void);

#endif
