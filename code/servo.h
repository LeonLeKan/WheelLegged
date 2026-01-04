#ifndef __SERVO_H
#define __SERVO_H

#include "zf_common_headfile.h"

#define SERVO_MOTOR_PWM1            (TCPWM_CH21_P08_2)                          
#define SERVO_MOTOR_PWM2            (TCPWM_CH12_P05_3)                          
#define SERVO_MOTOR_PWM3            (TCPWM_CH20_P08_1)                          
#define SERVO_MOTOR_PWM4            (TCPWM_CH11_P05_2)  
#define SERVO_MOTOR_FREQ            (50)

void servo_init(void);
void servo1_set_angle(float angle);
void servo2_set_angle(float angle);
void servo3_set_angle(float angle);
void servo4_set_angle(float angle);
void car_balance(void);
void servo_set(float servo_right_back, float servo_left_front, float servo_right_front, float servo_left_back);

#endif