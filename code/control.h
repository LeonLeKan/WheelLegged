#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include "imu.h"
#include "arm_math.h"
#include "menu.h"

extern volatile uint8_t image_block;
extern volatile float roll_base;
extern volatile float yaw_test_bias;
extern volatile uint8_t jump_count;
extern volatile float yaw_de_final;
extern volatile uint32_t jolt_trap_count;
extern volatile float servo_right_front;
extern volatile float servo_left_front;
extern volatile float servo_right_back;
extern volatile float servo_left_back;

void control_init(void);
void control_modify_judge(void);
bool control_modify_switch(int road_num, float euler_angle);

void control_state_idle(void);
void control_state_stand_still(void);
void control_state_deaccelerate(void);
void control_state_de_thunder(void);
void control_state_thunder(void);
void control_state_forward(void);
void control_state_decline_front_single_side(void);
void control_state_stand_front_single_side(void);
void control_state_single_side(void);
void control_state_de_stairs(void);
void control_state_simple_maintain(void);
void control_state_maintain_jump(void);
void control_state_simple_jump(void);
void control_state_jump_up(void);
void control_state_de_follow(void);
void control_state_follow_stairs(void);
void control_state_maintain_follow(void);
void control_state_de_jolt(void);
void control_state_jolt(void);
void control_state_jolt_trap(void);
void control_state_test(void);
void control_state_uturn(void);
void control_state_modify(void);
void control_state_stop(void);

#endif
