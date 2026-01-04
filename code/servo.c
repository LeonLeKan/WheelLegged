#include "servo.h"

void servo_init(void)
{
    pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, 417);
    pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, 667);
    pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, 833);
    pwm_init(SERVO_MOTOR_PWM4, SERVO_MOTOR_FREQ, 1083);
}

void servo1_set_angle(float angle)
{
    float pulse_width = (float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(angle)/90.0);
    pwm_set_duty(SERVO_MOTOR_PWM1, (uint32_t)pulse_width);
}

void servo2_set_angle(float angle)
{
    float pulse_width = (float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(angle)/90.0);
    pwm_set_duty(SERVO_MOTOR_PWM2, (uint32_t)pulse_width);
}

void servo3_set_angle(float angle)
{
    float pulse_width = (float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(angle)/90.0);
    pwm_set_duty(SERVO_MOTOR_PWM3, (uint32_t)pulse_width);
}

void servo4_set_angle(float angle)
{
    float pulse_width = (float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(angle)/90.0);
    pwm_set_duty(SERVO_MOTOR_PWM4, (uint32_t)pulse_width);
}

void car_balance(void)
{
    servo1_set_angle(30.0f); //30
    servo2_set_angle(75.0f); //75
    servo3_set_angle(105.0f);//105
    servo4_set_angle(150.0f);//150
}

void servo_set(float servo_right_back, float servo_left_front, float servo_right_front, float servo_left_back)
{
    servo1_set_angle(servo_right_back); 
    servo2_set_angle(servo_left_front); 
    servo3_set_angle(servo_right_front);
    servo4_set_angle(servo_left_back);
}
