#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Target;
    float Actual;
    float Out;

    float Kp;
    float Ki;
    float Kd;

    float Error0;
    float Error1;
    float Error2;
    float ErrorInt;

    float OutMax;
    float OutMin;

    float ErrorIntMax;
}PID_t;

extern volatile PID_t angle_loop1;
extern volatile PID_t angle_loop2;
extern volatile PID_t speed_loop;
extern volatile PID_t turn_loop1;
extern volatile PID_t turn_loop2;
extern volatile PID_t aspeed_loop;
extern volatile PID_t single_loop;
extern volatile PID_t yaw_loop;
extern volatile PID_t jos_loop;
extern volatile PID_t joy_loop;

void PID_Update_Lo(volatile PID_t *p);
void PID_Update_In(volatile PID_t *p);

void an1pid_update(float kp, float ki, float kd);
void an1_zero(void);
void an2pid_update(float kp, float ki, float kd);
void an2_zero(void);
void sppid_update(float kp, float ki, float kd);
void sp_zero(void);
void tu1pid_update(float kp, float ki, float kd);
void tu1_zero(void);
void tu2pid_update_f(float kp, float ki, float kd);
void tu2pid_update_t(float kp, float ki, float kd);
void tu2pid_update_j(float kp, float ki, float kd);
void tu2_zero(void);
void aspid_update(float kp, float ki, float kd);
void as_zero(void);
void snpid_update(float kp, float ki, float kd);
void sn_zero(void);
void yapid_update(float kp, float ki, float kd);
void ya_zero(void);
void jospid_update(float kp, float ki, float kd);
void jos_zero(void);
void joypid_update(float kp, float ki, float kd);
void joy_zero(void);

void para_zero(void);
void out_zero(void);

#endif
