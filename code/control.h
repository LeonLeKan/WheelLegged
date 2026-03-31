#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include "imu.h"
#include "arm_math.h"
#include "menu.h"

extern volatile uint8_t image_block;
extern volatile float roll_base;
extern volatile float yaw_test_bias;

void control_init(void);

void control_state_idle(void);
void control_state_stand_up(void);
void control_state_stand_still(void);
void control_state_accelerate(void);
void control_state_deaccelerate(void);
void control_state_de_thunder(void);
void control_state_tra_thunder(void);
void control_state_forward(void);
void control_state_decline_front_single_side(void);
void control_state_stand_front_single_side(void);
void control_state_single_side(void);
void control_state_thunder(void);
void control_state_jump_up(void);
void control_state_jolt(void);
void control_state_test(void);
void control_state_stop(void);

#endif
