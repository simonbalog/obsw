#ifndef STATUS_REPORT_H
#define STATUS_REPORT_H

void status_report_init(void);
void status_report_update(void);
void status_report_send_lora(void);
void status_report_event(const char *level, unsigned int code, const char *text);

#endif
