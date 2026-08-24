#include "servo.h"

void servo_init(void)
{
    pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, 750);
    pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, 750);
    pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, 750);
    pwm_init(SERVO_MOTOR_PWM4, SERVO_MOTOR_FREQ, 750);
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
    servo1_set_angle(90.0f);
    servo2_set_angle(90.0f);
    servo3_set_angle(90.0f);
    servo4_set_angle(90.0f);
}

static float clamp_angle(float angle) 
{
    if (angle < 0.0f) return 0.0f;
    if (angle > 180.0f) return 180.0f;
    return angle;
}

void servo_set(float servo_left_back, float servo_right_front, float servo_right_back, float servo_left_front)
{
    servo1_set_angle(clamp_angle(servo_left_back)); 
    servo2_set_angle(clamp_angle(servo_right_front)); 
    servo3_set_angle(clamp_angle(servo_right_back));
    servo4_set_angle(clamp_angle(servo_left_front));
}

void servo_set_01(float servo_left_back, float servo_right_front, float servo_right_back, float servo_left_front)
{
    static float filtered_left_back = 90.0f;
    static float filtered_right_front = 90.0f;
    static float filtered_right_back = 90.0f;
    static float filtered_left_front = 90.0f;
    static const float filter_alpha = 0.1f;

    filtered_left_back = filter_alpha * servo_left_back + (1.0f - filter_alpha) * filtered_left_back;
    filtered_right_front = filter_alpha * servo_right_front + (1.0f - filter_alpha) * filtered_right_front;
    filtered_right_back = filter_alpha * servo_right_back + (1.0f - filter_alpha) * filtered_right_back;
    filtered_left_front = filter_alpha * servo_left_front + (1.0f - filter_alpha) * filtered_left_front;

    servo1_set_angle(clamp_angle(filtered_left_back));
    servo2_set_angle(clamp_angle(filtered_right_front));
    servo3_set_angle(clamp_angle(filtered_right_back));
    servo4_set_angle(clamp_angle(filtered_left_front));
}
