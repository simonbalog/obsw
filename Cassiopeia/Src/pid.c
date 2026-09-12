#include "pid.h"
#include "stm32h7xx_hal.h"

/* Genericky PID regulator s antialiasintegralem.
 *
 *  - pid_update() meri dt pres HAL_GetTick (hlavni smycka)
 *  - pid_update_dt() ma dt z volajiciho (presne casovani, ISR)
 *  - pid_reset() vynuluje stav (zajisteni navaznosti po re-engage)
 */

void pid_init(PidCtrl *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    p->integral = 0;
    p->prev_error = 0;
    p->prev_tick = 0;
    p->initialized = 0;
}

void pid_reset(PidCtrl *p)
{
    p->integral = 0;
    p->prev_error = 0;
    p->initialized = 0;
}

float pid_update_dt(PidCtrl *p, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    p->integral += error * dt;
    float i_limit = p->out_max / (p->ki + 1e-6f);
    if (p->integral >  i_limit) p->integral =  i_limit;
    if (p->integral < -i_limit) p->integral = -i_limit;

    float deriv = 0;
    if (p->initialized)
        deriv = (error - p->prev_error) / dt;

    float out = p->kp * error + p->ki * p->integral + p->kd * deriv;

    if (out > p->out_max) out = p->out_max;
    if (out < p->out_min) out = p->out_min;

    p->prev_error = error;
    p->initialized = 1;

    return out;
}

float pid_update(PidCtrl *p, float setpoint, float measurement)
{
    uint32_t now = HAL_GetTick();

    float dt = 0.001f;
    if (p->initialized)
        dt = (now - p->prev_tick) * 0.001f;
    if (dt <= 0.0f)
        dt = 0.001f;
    if (dt > 0.1f)
        dt = 0.1f;

    float out = pid_update_dt(p, setpoint, measurement, dt);
    p->prev_tick = now;

    return out;
}
