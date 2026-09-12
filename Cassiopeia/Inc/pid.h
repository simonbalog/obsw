#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float out_min;
    float out_max;

    float integral;
    float prev_error;
    uint32_t prev_tick;
    uint8_t  initialized;
} PidCtrl;

void pid_init(PidCtrl *p, float kp, float ki, float kd, float out_min, float out_max);
void pid_reset(PidCtrl *p);
float pid_update(PidCtrl *p, float setpoint, float measurement);
float pid_update_dt(PidCtrl *p, float setpoint, float measurement, float dt);

#endif
