#include "pid.h"
#include "menu.h"

volatile PID_t angle_loop1;
volatile PID_t angle_loop2;
volatile PID_t speed_loop;
volatile PID_t speed_loop2;
volatile PID_t turn_loop1;
volatile PID_t turn_loop2;
volatile PID_t aspeed_loop;
volatile PID_t single_loop;
volatile PID_t yaw_loop;
volatile PID_t jos_loop;
volatile PID_t joy_loop;

static float wrap_angle_error(float target, float actual)
{
    float error = target - actual;
    if (error > 180.0f)
    {
        error -= 360.0f;
    }
    else if (error < -180.0f)
    {
        error += 360.0f;
    }
    return error;
}

void PID_Update_Lo(volatile PID_t *p)
{
    p->Error1 = p->Error0;
    if (p == &yaw_loop || p == &joy_loop)
    {
        p->Error0 = wrap_angle_error(p->Target, p->Actual);
    }
    else
    {
        p->Error0 = p->Target - p->Actual;
    }

    if ((p == &speed_loop2 || p == &speed_loop) && fabs(p->Error0) > 0.5f)
    {
        p->ErrorInt = 0.0f;
    }
    else 
    {
        p->ErrorInt += p->Error0;
    }

    if (p->ErrorInt > p->ErrorIntMax)
    {
        p->ErrorInt = p->ErrorIntMax;
    }
    else if (p->ErrorInt < -p->ErrorIntMax)
    {
        p->ErrorInt = -p->ErrorIntMax;
    }

    p->Out = p->Kp * p->Error0 + p->Ki * p->ErrorInt + p->Kd * (p->Error0 - p->Error1);

    if (p->Out > p->OutMax)
    {
        p->Out = p->OutMax;
    }
    if (p->Out < p->OutMin)
    {
        p->Out = p->OutMin;
    }
}

void an1pid_update(float kp, float ki, float kd)
{
    angle_loop1.Kp = kp;
    angle_loop1.Ki = ki;
    angle_loop1.Kd = kd;
    angle_loop1.OutMax = +7000;
    angle_loop1.OutMin = -7000;
}

void an1_zero(void)
{
    angle_loop1.Kp = 0;
    angle_loop1.Ki = 0;
    angle_loop1.Kd = 0;
}

void an2pid_update(float kp, float ki, float kd)
{
    angle_loop2.Kp = kp;
    angle_loop2.Ki = ki;
    angle_loop2.Kd = kd;
    angle_loop2.OutMax = +500;
    angle_loop2.OutMin = -500;
    angle_loop2.ErrorIntMax = 600;
}

void an2_zero(void)
{
    angle_loop2.Kp = 0;
    angle_loop2.Ki = 0;
    angle_loop2.Kd = 0;
}

void sppid_update(float kp, float ki, float kd)
{
    speed_loop.Kp = kp;
    speed_loop.Ki = ki;
    speed_loop.Kd = kd;
    speed_loop.OutMax = +50;
    speed_loop.OutMin = -50;
    speed_loop.ErrorIntMax = 500;
}

void sp_zero(void)
{
    speed_loop.Kp = 0;
    speed_loop.Ki = 0;
    speed_loop.Kd = 0;
}

void sp2pid_update(float kp, float ki, float kd)
{
    speed_loop2.Kp = kp;
    speed_loop2.Ki = ki;
    speed_loop2.Kd = kd;
    speed_loop2.OutMax = +4;
    speed_loop2.OutMin = -4;
    speed_loop2.ErrorIntMax = 500;
}

void sp2_zero(void)
{
    speed_loop2.Kp = 0;
    speed_loop2.Ki = 0;
    speed_loop2.Kd = 0;
}

void tu1pid_update(float kp, float ki, float kd)
{
    turn_loop1.Kp = kp;
    turn_loop1.Ki = ki;
    turn_loop1.Kd = kd;
    turn_loop1.OutMax = +2500;
    turn_loop1.OutMin = -2500;
}

void tu1_zero(void)
{
    turn_loop1.Kp = 0;
    turn_loop1.Ki = 0;
    turn_loop1.Kd = 0;
}

void tu2pid_update_f(float kp, float ki, float kd)
{
    turn_loop2.Kp = kp;
    turn_loop2.Ki = ki;
    turn_loop2.Kd = kd;
    turn_loop2.OutMax = +turnamf->data_int;
    turn_loop2.OutMin = -turnamf->data_int;
    turn_loop2.ErrorIntMax = 200;
}

void tu2pid_update_t(float kp, float ki, float kd)
{
    turn_loop2.Kp = kp;
    turn_loop2.Ki = ki;
    turn_loop2.Kd = kd;
    turn_loop2.OutMax = +turnamt->data_int;
    turn_loop2.OutMin = -turnamt->data_int;
    turn_loop2.ErrorIntMax = 200;
}

void tu2pid_update_j(float kp, float ki, float kd)
{
    turn_loop2.Kp = kp;
    turn_loop2.Ki = ki;
    turn_loop2.Kd = kd;
    turn_loop2.OutMax = +turnamj->data_int;
    turn_loop2.OutMin = -turnamj->data_int;
    turn_loop2.ErrorIntMax = 200;
}

void tu2pid_update_h(float kp, float ki, float kd)
{
    turn_loop2.Kp = kp;
    turn_loop2.Ki = ki;
    turn_loop2.Kd = kd;
    turn_loop2.OutMax = +turnamh->data_int;
    turn_loop2.OutMin = -turnamh->data_int;
    turn_loop2.ErrorIntMax = 200;
}

void tu2_zero(void)
{
    turn_loop2.Kp = 0;
    turn_loop2.Ki = 0;
    turn_loop2.Kd = 0;
}

void aspid_update(float kp, float ki, float kd)
{
    aspeed_loop.Kp = kp;
    aspeed_loop.Ki = ki;
    aspeed_loop.Kd = kd;
    aspeed_loop.OutMax = +7000;
    aspeed_loop.OutMin = -7000;
}

void as_zero(void)
{
    aspeed_loop.Kp = 0;
    aspeed_loop.Ki = 0;
    aspeed_loop.Kd = 0;
}

void snpid_update(float kp, float ki, float kd)
{
    single_loop.Kp = kp;
    single_loop.Ki = ki;
    single_loop.Kd = kd;
    single_loop.OutMax = +30;
    single_loop.OutMin = -30;
}

void sn_zero(void)
{
    single_loop.Kp = 0;
    single_loop.Ki = 0;
    single_loop.Kd = 0;
}

void yapid_update(float kp, float ki, float kd)
{
    yaw_loop.Kp = kp;
    yaw_loop.Ki = ki;
    yaw_loop.Kd = kd;
    yaw_loop.OutMax = +2500;
    yaw_loop.OutMin = -2500;
}

void ya_zero(void)
{
    yaw_loop.Kp = 0;
    yaw_loop.Ki = 0;
    yaw_loop.Kd = 0;
}

void jospid_update(float kp, float ki, float kd)
{
    jos_loop.Kp = kp;
    jos_loop.Ki = ki;
    jos_loop.Kd = kd;
    jos_loop.OutMax = +30;
    jos_loop.OutMin = -30;
}

void jos_zero(void)
{
    jos_loop.Kp = 0;
    jos_loop.Ki = 0;
    jos_loop.Kd = 0;
}

void joypid_update(float kp, float ki, float kd)
{
    joy_loop.Kp = kp;
    joy_loop.Ki = ki;
    joy_loop.Kd = kd;
    joy_loop.OutMax = +2500;
    joy_loop.OutMin = -2500;
}

void joy_zero(void)
{
    joy_loop.Kp = 0;
    joy_loop.Ki = 0;
    joy_loop.Kd = 0;
}

void para_zero(void)
{
    an1_zero();
    an2_zero();
    sp_zero();
    sp2_zero();
    tu1_zero();
    tu2_zero();
    as_zero();
    sn_zero();
    ya_zero();
    jos_zero();
    joy_zero();
}

void out_zero(void)
{
    angle_loop1.Out = 0.0f;
    angle_loop2.Out = 0.0f;
    speed_loop.Out = 0.0f;
    speed_loop2.Out = 0.0f;
    turn_loop1.Out = 0.0f;
    turn_loop2.Out = 0.0f;
    aspeed_loop.Out = 0.0f;
    single_loop.Out = 0.0f;
    yaw_loop.Out = 0.0f;
    jos_loop.Out = 0.0f;
    joy_loop.Out = 0.0f;
}
