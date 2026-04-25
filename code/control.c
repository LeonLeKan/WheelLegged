#include "control.h"
#include "small_driver_uart_control.h"
#include "servo.h"
#include "image.h"
#include "control_state.h"
#include "pid.h"
#include "location.h"

// relevant variables
volatile uint8_t image_block = 0;
volatile float roll_base = 0.0f;
volatile float yaw_test_bias = 0.0f;
volatile uint8_t jump_count = 0;
volatile uint8_t follow_stairs_count = 0;

void control_init(void)
{
    servo_init();
    small_driver_uart_init();
    pit_ms_init(PIT_CH1, 5);
    pit_ms_init(PIT_CH2, 20);
}

void control_state_idle(void)
{
    para_zero();
    out_zero();
    small_driver_set_duty(0, 0);
}

void control_state_stand_up(void)
{
    static uint32_t count = 0;
    count++;
    if (count >= 400)
    {
        an1pid_update(an1kp->data_float, an1ki->data_float, an1kd->data_float);

        angle_loop1.Target = pinit->data_float;
        angle_loop1.Actual = imu.output.euler[1];
        PID_Update_Lo(&angle_loop1);

        small_driver_set_duty((int16)angle_loop1.Out, (int16)-angle_loop1.Out);

        if (count >= 450)
        {
            count = 0;
            current_state = STATE_STAND_STILL;
        }
    }
}

void control_state_stand_still(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 50)
    {
        count = 0;
        if (test_state == 1)
        {
            current_state = STATE_TEST;
        }
        else if (test_state == 0)
        {
            current_state = STATE_ACCELERATE;
        }
    }

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)aspeed_loop.Out, (int16)-aspeed_loop.Out);
}

void control_state_accelerate(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)speed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(50.0f, 120.0f,
                  60.0f, 130.0f);

        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual;
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_deaccelerate(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)speed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(130.0f, 50.0f,
                  130.0f, 50.0f);
    }
    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_de_thunder(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = 1.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(110.0f, 70.0f,
                  110.0f, 70.0f);

        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);
    
    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_tra_thunder(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = 1.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f;

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_thunder(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static float yaw_thunder_now = 0.0f;
    static float yaw_thunder_last = 0.0f;
    static float yaw_thunder_diff = 0.0f;
    static uint8_t yaw_thunder_flag = 0;
    static float yaw_accumulate = 0.0f;

    servo_set(90.0f, 90.0f,
              90.0f, 90.0f);

    if (yaw_thunder_memory <= 0)
    {
        turn_loop2.Target = rospeed->data_int;
        turn_loop2.Actual = -imu.body.gyro[2];
        PID_Update_Lo(&turn_loop2);

        angle_loop2.Target = pzero->data_float;
        angle_loop2.Actual = imu.output.euler[1];
        PID_Update_Lo(&angle_loop2);

        aspeed_loop.Target = angle_loop2.Out;
        aspeed_loop.Actual = imu.output.gyro[1];
        PID_Update_Lo(&aspeed_loop);

        small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));

        yaw_thunder_now = imu.output.euler[2];
        if (yaw_thunder_flag == 0)
        {
            yaw_thunder_last = yaw_thunder_memory;
            yaw_thunder_flag = 1;
        }

        yaw_thunder_diff = yaw_thunder_now - yaw_thunder_last;
        if (yaw_thunder_diff > 180.0f)
        {
            yaw_thunder_diff -= 360.0f;
        }
        else if (yaw_thunder_diff < -180.0f)
        {
            yaw_thunder_diff += 360.0f;
        }

        yaw_accumulate += yaw_thunder_diff;
        yaw_thunder_last = yaw_thunder_now;

        if (yaw_accumulate <= -720.0f)
        {
            yaw_thunder_now = 0.0f;
            yaw_thunder_last = 0.0f;
            yaw_thunder_diff = 0.0f;
            yaw_thunder_flag = 0;
            yaw_accumulate = 0.0f;
            speed_loop.Actual = 0.0f;
            current_state = STATE_ACCELERATE;
        }
    }
    else if (yaw_thunder_memory > 0)
    {
        turn_loop2.Target = -rospeed->data_int;
        turn_loop2.Actual = -imu.body.gyro[2];
        PID_Update_Lo(&turn_loop2);

        angle_loop2.Target = pzero->data_float;
        angle_loop2.Actual = imu.output.euler[1];
        PID_Update_Lo(&angle_loop2);

        aspeed_loop.Target = angle_loop2.Out;
        aspeed_loop.Actual = imu.output.gyro[1];
        PID_Update_Lo(&aspeed_loop);

        small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));

        yaw_thunder_now = imu.output.euler[2];
        if (yaw_thunder_flag == 0)
        {
            yaw_thunder_last = yaw_thunder_memory;
            yaw_thunder_flag = 1;
        }

        yaw_thunder_diff = yaw_thunder_now - yaw_thunder_last;
        if (yaw_thunder_diff > 180.0f)
        {
            yaw_thunder_diff -= 360.0f;
        }
        else if (yaw_thunder_diff < -180.0f)
        {
            yaw_thunder_diff += 360.0f;
        }

        yaw_accumulate += yaw_thunder_diff;
        yaw_thunder_last = yaw_thunder_now;

        if (yaw_accumulate >= 720.0f)
        {
            yaw_thunder_now = 0.0f;
            yaw_thunder_last = 0.0f;
            yaw_thunder_diff = 0.0f;
            yaw_thunder_flag = 0;
            yaw_accumulate = 0.0f;
            speed_loop.Actual = 0.0f;
            current_state = STATE_ACCELERATE;
        }
    }
}

void control_state_forward(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        if (testnum->data_int == 0)
        {
            if (speed_0_change == 1)
            {
                speed_loop.Target = (float)speedl->data_int / 100.0f;
            }
            else if(speed_0_change == 0)
            {
                speed_loop.Target = (float)speed->data_int / 100.0f;
            }
        }
        else if (testnum->data_int == 1 || testnum->data_int == 2)
        {
            speed_loop.Target = (float)speed->data_int / 100.0f;
        }
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; 

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_decline_front_single_side(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)snspeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(110.0f, 70.0f,
                  110.0f, 70.0f);

        turn_loop1.Target = 0.0f; 
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_stand_front_single_side(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;
    if (count % 4 == 0)
    {
        count = 0;

        speed_loop.Target = (float)snspeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; 

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(115.0f - filtered_speed_out, 115.0f + filtered_speed_out,
                  65.0f - filtered_speed_out, 65.0f + filtered_speed_out);

        turn_loop1.Target = 0.0f; 
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_single_side(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    snpid_update(snkp->data_float, snki->data_float, snkd->data_float);
    threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;

    // 对speed_loop.Out进行低通滤波
    static float filtered_speed_out = 0.0f;
    static const float speed_alpha = 0.1f; 

    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)snspeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        turn_loop1.Target = 0.0f; 
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    // 单边桥控制
    single_loop.Target = roll_base;
    single_loop.Actual = imu.output.euler[0];
    PID_Update_Lo(&single_loop);

    // 对single_loop.Out进行低通滤波
    static float filtered_single_out = 0.0f;
    static const float single_alpha = 0.1f; 
    
    filtered_single_out = single_alpha * single_loop.Out + (1.0f - single_alpha) * filtered_single_out;
    
    servo_set(115.0f - filtered_speed_out - filtered_single_out, 115.0f + filtered_speed_out + filtered_single_out,
              65.0f - filtered_speed_out + filtered_single_out, 65.0f + filtered_speed_out - filtered_single_out);

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_de_stairs(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)stspeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(100.0f, 80.0f,
                  100.0f, 80.0f);

        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_tra_stairs(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)stspeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; 

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_find_jump(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(10.0f, spki->data_float, spkd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)stspeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; 

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        turn_loop1.Target = 0.0f; 
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_jump_up(void)
{
    angle_loop2.Ki = 0.0f;
    speed_loop.Ki = 0.0f;
    turn_loop2.Ki = 0.0f;

    static uint32_t count = 0;
    count++;

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out), (int16)(-(aspeed_loop.Out)));

    if (count < 17)
    {
        servo_set(150.0f, 150.0f,
                  30.0f, 30.0f);
    }

    else if (count >= 17 && count < 30)
    {
        servo_set(110.0f, 110.0f,
                  70.0f, 70.0f);
    }

    else if (count >= 30)
    {
        count = 0;
        image_block = 1;
        servo_set(90.0f, 90.0f,
                  90.0f, 90.0f);
        angle_loop2.ErrorInt = 0.0f;
        speed_loop.ErrorInt = 0.0f;
        turn_loop2.ErrorInt = 0.0f;
        current_state = STATE_FIND_JUMP;
    }
}

void control_state_tra_follow(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)baspeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; 

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_follow_stairs(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)baspeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; 

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        turn_loop1.Target = 0.0f; 
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_close_stairs(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);

    static uint32_t close_delay = 0;
    static uint32_t count = 0;
    close_delay++;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)baspeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; 

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);
    }

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out), (int16)(-aspeed_loop.Out));

    if (close_delay >= 40)
    {
        close_delay = 0;
        angle_loop2.ErrorInt = 0.0f;
        speed_loop.ErrorInt = 0.0f;
        turn_loop2.ErrorInt = 0.0f;
        image_block = 1;
        current_state = STATE_FOLLOW_STAIRS;
    }
}

void control_state_de_jolt(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)jospeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(110.0f, 70.0f,
                  110.0f, 70.0f);

        joy_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_tra_jolt(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)jospeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f;

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        joy_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_jolt(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    angle_loop2.Ki = 0.0f;
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    joypid_update(joykp->data_float, joyki->data_float, joykd->data_float);
    tu2pid_update_j(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    jospid_update(joskp->data_float, joski->data_float, joskd->data_float);
    threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;

    // 对speed_loop.Out进行低通滤波
    static float filtered_speed_out = 0.0f;
    static const float speed_alpha = 0.1f; 

    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)jospeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        yaw_loop.Target = 0.0f;
        yaw_loop.Actual = imu.output.euler[2];
        joy_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&joy_loop);
    }

    // 单边桥控制
    jos_loop.Target = roll_base;
    jos_loop.Actual = imu.output.euler[0];
    PID_Update_Lo(&jos_loop);

    // 对jos_loop.Out进行低通滤波
    static float filtered_jos_out = 0.0f;
    static const float jos_alpha = 0.1f; 
    
    filtered_jos_out = jos_alpha * jos_loop.Out + (1.0f - jos_alpha) * filtered_jos_out;

    servo_set(90.0f - filtered_speed_out - filtered_jos_out, 90.0f + filtered_speed_out + filtered_jos_out,
              90.0f - filtered_speed_out + filtered_jos_out, 90.0f + filtered_speed_out - filtered_jos_out);

    turn_loop2.Target = -joy_loop.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    int16 duty1 = (int16)(aspeed_loop.Out - turn_loop2.Out);
    int16 duty2 = (int16)(-(aspeed_loop.Out + turn_loop2.Out));
    limit_min_output(&duty1, 100);
    limit_min_output(&duty2, 100);
    small_driver_set_duty((int16)(duty1), (int16)(duty2));
}

void control_state_test(void)
{
    static uint16_t last_channel_0 = 0;

    if (last_channel_0 != 800 && uart_receiver.channel[0] == 800)
    {
        yaw_loop.Target = imu.output.euler[2];
    }
    last_channel_0 = uart_receiver.channel[0];

    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);

    sppid_update(spkp->data_float, 0.0f, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_t(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; 

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        yaw_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; 
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    if (uart_receiver.channel[0] == 800)
    {
        small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
    }
    else
    {
        small_driver_set_duty((int16)(aspeed_loop.Out + yaw_test_bias), (int16)(-(aspeed_loop.Out - yaw_test_bias)));
    }
}

void control_state_stop(void)
{
    para_zero();
    out_zero();
    small_driver_set_duty(0, 0);
}


void limit_min_output(int16 *output, int16 limit)
{
    if (*output > 0 && *output < limit)
    {
        *output = limit;
    }
    else if (*output < 0 && *output > -limit)
    {
        *output = -limit;
    }
}