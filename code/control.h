#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include "imu.h"
#include "math.h"
#include "menu.h"

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

extern volatile uint8_t mode_flag;

void control_init(void);
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
void tu2pid_update(float kp, float ki, float kd);
void tu2_zero(void);
void aspid_update(float kp, float ki, float kd);
void as_zero(void);

#endif /* CODE_CONTROL_H_ */
