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
extern volatile PID_t single_loop;

extern volatile uint8_t image_block;

extern volatile float roll_base;

void control_init(void);
void PID_Update_Lo(volatile PID_t *p);
void PID_Update_In(volatile PID_t *p);

// PID参数更新与归零函数
void an1pid_update(float kp, float ki, float kd);
void an1_zero(void);
void an2pid_update(float kp, float ki, float kd);
void an2_zero(void);
void acpid_update(float ki, float kd);
void sppid_update(float kp, float ki, float kd);
void sp_zero(void);
void tu1pid_update(float kp, float ki, float kd);
void tu1_zero(void);
void tu2pid_update(float kp, float ki, float kd);
void tu2_zero(void);
void aspid_update(float kp, float ki, float kd);
void as_zero(void);
void snpid_update(float kp, float ki, float kd);
void sn_zero(void);

// 参数与输出归零函数（汇总）
void para_zero(void);
void out_zero(void);

// 控制状态函数
void control_state_idle(void);
void control_state_stand_up(void);
void control_state_stand_still(void);
void control_state_accelerate(void);
void control_state_forward(void);
void control_state_decline_front_single_side(void);
void control_state_stand_front_single_side(void);
void control_state_single_side(void);
void control_state_thunder(void);
void control_state_jump_up(void);
void control_state_jolt(void);
void control_state_stop(void);

#endif
