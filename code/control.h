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

void control_init(void);

void control_state_idle(void);
void control_state_stand_still(void);
void control_state_accelerate(void);
void control_state_deaccelerate(void);
void control_state_de_thunder(void);
void control_state_tra_thunder(void);
void control_state_thunder(void);
void control_state_forward(void);
void control_state_decline_front_single_side(void);
void control_state_stand_front_single_side(void);
void control_state_single_side(void);
void control_state_de_stairs(void);
void control_state_tra_stairs(void);
void control_state_find_jump(void);
void control_state_jump_up(void);
void control_state_tra_follow(void);
void control_state_follow_stairs(void);
void control_state_de_jolt(void);
void control_state_jolt_image(void);
void control_state_jolt(void);
void control_state_jolt_trap(void);
void control_state_test(void);
void control_state_uturn(void);
void control_state_modify(void);
void control_state_stop(void);

#endif
