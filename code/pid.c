#include "pid.h"

volatile PID_t angle_loop1;
volatile PID_t angle_loop2;
volatile PID_t speed_loop;
volatile PID_t turn_loop1;
volatile PID_t turn_loop2;
volatile PID_t aspeed_loop;
volatile PID_t single_loop;
volatile PID_t yaw_loop;

void PID_Update_Lo(volatile PID_t *p)
{
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    p->ErrorInt += p->Error0;

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

void PID_Update_In(volatile PID_t *p)
{
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;
    p->Error2 = p->Error1;

    p->Out += p->Kp * (p->Error0 - p->Error1) + p->Ki * p->Error0 + p->Kd * (p->Error0 - 2 * p->Error1 + p->Error2);

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
    speed_loop.OutMax = +30;
    speed_loop.OutMin = -30;
    speed_loop.ErrorIntMax = 300;
}

void sp_zero(void)
{
    speed_loop.Kp = 0;
    speed_loop.Ki = 0;
    speed_loop.Kd = 0;
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

void tu2pid_update(float kp, float ki, float kd)
{
    turn_loop2.Kp = kp;
    turn_loop2.Ki = ki;
    turn_loop2.Kd = kd;
    turn_loop2.OutMax = +300;
    turn_loop2.OutMin = -300;
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
    single_loop.OutMax = +100;
    single_loop.OutMin = -100;
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

void para_zero(void)
{
    an1_zero();
    an2_zero();
    sp_zero();
    tu1_zero();
    tu2_zero();
    as_zero();
    sn_zero();
    ya_zero();
}

void out_zero(void)
{
    angle_loop1.Out = 0.0f;
    angle_loop2.Out = 0.0f;
    speed_loop.Out = 0.0f;
    turn_loop1.Out = 0.0f;
    turn_loop2.Out = 0.0f;
    aspeed_loop.Out = 0.0f;
    single_loop.Out = 0.0f;
    yaw_loop.Out = 0.0f;
}
