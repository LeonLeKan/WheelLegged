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
volatile float yaw_de_final = 0.0f;
volatile uint32_t jolt_trap_count = 0;

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

void control_state_stand_still(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);

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

        servo_set(60.0f, 120.0f,
                  60.0f, 120.0f);

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

        yaw_loop.Target = yaw_de_final;
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

        speed_loop.Target = 1.2f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(130.0f, 50.0f,
                  130.0f, 50.0f);

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

        speed_loop.Target = 1.2f;
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

    static uint8_t thunder_process = 0;

    servo_set(90.0f, 90.0f,
              90.0f, 90.0f);

    if (thunder_direction_flag == 1)
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

        if (thunder_process == 0)
        {
            thunder_process = 1;
        }

        else if (thunder_process == 1)
        {
            if (fabs(imu.output.euler[2] - thunder_final_angle) < 3.0f)
            {
                thunder_process = 2;
            }
        }

        else if (thunder_process == 2)
        {
            yaw_thunder_now = imu.output.euler[2];
            if (yaw_thunder_flag == 0)
            {
                yaw_thunder_last = thunder_final_angle;
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

            if (yaw_accumulate <= -710.0f)
            {
                yaw_thunder_now = 0.0f;
                yaw_thunder_last = 0.0f;
                yaw_thunder_diff = 0.0f;
                yaw_thunder_flag = 0;
                yaw_accumulate = 0.0f;
                speed_loop.Actual = 0.0f;
                thunder_process = 0;
                if (thunder_count < thnum->data_int)
                {
                    current_state = STATE_ACCELERATE;
                }
                else if (thunder_count >= thnum->data_int)
                {
                    current_state = STATE_FORWARD;
                }
            }
        }
    }
    else if (thunder_direction_flag == 2)
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

        if (thunder_process == 0)
        {
            thunder_process = 1;
        }

        else if (thunder_process == 1)
        {
            if (fabs(imu.output.euler[2] - thunder_final_angle) < 3.0f)
            {
                thunder_process = 2;
            }
        }

        else if (thunder_process == 2)
        {
            yaw_thunder_now = imu.output.euler[2];
            if (yaw_thunder_flag == 0)
            {
                yaw_thunder_last = thunder_final_angle;
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

            if (yaw_accumulate >= 710.0f)
            {
                yaw_thunder_now = 0.0f;
                yaw_thunder_last = 0.0f;
                yaw_thunder_diff = 0.0f;
                yaw_thunder_flag = 0;
                yaw_accumulate = 0.0f;
                speed_loop.Actual = 0.0f;
                thunder_process = 0;
                if (thunder_count < thnum->data_int)
                {
                    current_state = STATE_ACCELERATE;
                }
                else if (thunder_count >= thnum->data_int)
                {
                    current_state = STATE_FORWARD;
                }
            }
        }
    }
    else if (thunder_direction_flag == 3)
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

        if (thunder_process == 0)
        {
            thunder_process = 1;
        }

        else if (thunder_process == 1)
        {
            thunder_process = 2;
        }

        else if (thunder_process == 2)
        {
            yaw_thunder_now = imu.output.euler[2];
            if (yaw_thunder_flag == 0)
            {
                yaw_thunder_last = thunder_final_angle;
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

            if (yaw_accumulate <= -710.0f)
            {
                yaw_thunder_now = 0.0f;
                yaw_thunder_last = 0.0f;
                yaw_thunder_diff = 0.0f;
                yaw_thunder_flag = 0;
                yaw_accumulate = 0.0f;
                speed_loop.Actual = 0.0f;
                thunder_process = 0;
                if (thunder_count < thnum->data_int)
                {
                    current_state = STATE_ACCELERATE;
                }
                else if (thunder_count >= thnum->data_int)
                {
                    current_state = STATE_FORWARD;
                }
            }
        }
    }
    else if (thunder_direction_flag == 4)
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

        if (thunder_process == 0)
        {
            thunder_process = 1;
        }

        else if (thunder_process == 1)
        {
            thunder_process = 2;
        }

        else if (thunder_process == 2)
        {
            yaw_thunder_now = imu.output.euler[2];
            if (yaw_thunder_flag == 0)
            {
                yaw_thunder_last = thunder_final_angle;
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

            if (yaw_accumulate >= 710.0f)
            {
                yaw_thunder_now = 0.0f;
                yaw_thunder_last = 0.0f;
                yaw_thunder_diff = 0.0f;
                yaw_thunder_flag = 0;
                yaw_accumulate = 0.0f;
                speed_loop.Actual = 0.0f;
                thunder_process = 0;
                if (thunder_count < thnum->data_int)
                {
                    current_state = STATE_ACCELERATE;
                }
                else if (thunder_count >= thnum->data_int)
                {
                    current_state = STATE_FORWARD;
                }
            }
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
            else if (speed_0_change == 0)
            {
                speed_loop.Target = (float)speed->data_int / 100.0f;
            }
            else if (speed_0_change == 2)
            {
                speed_loop.Target = 2.0f;
            }
        }
        else if (testnum->data_int == 1)
        {
            if (thunder_count >= thnum->data_int)
            {
                if (speed_1_change == 1)
                {
                    speed_loop.Target = (float)speedl->data_int / 100.0f;
                }
                else if (speed_1_change == 0)
                {
                    speed_loop.Target = (float)speed->data_int / 100.0f;
                }
                else if (speed_1_change == 2)
                {
                    speed_loop.Target = 2.0f;
                }
            }
            else if (thunder_count < thnum->data_int)
            {
                speed_loop.Target = (float)thsp->data_int / 100.0f;
            }
        }
        else if (testnum->data_int == 2)
        {
            if (speed_2_change == 1)
            {
                speed_loop.Target = 2.0f;
            }
            else if (speed_2_change == 0)
            {
                speed_loop.Target = (float)speed->data_int / 100.0f;
            }
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

        servo_set(80.0f - filtered_speed_out, 100.0f + filtered_speed_out,
                  100.0f - filtered_speed_out, 80.0f + filtered_speed_out);

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

    if (fabs(imu.output.euler[0] - roll_base) <= 3.0f)
    {
        // 单边桥控制
        single_loop.Target = roll_base;
        single_loop.Actual = imu.output.euler[0];
        PID_Update_Lo(&single_loop);
    }
    else
    {
        single_loop.Out = 0.0f;
    }

    // 对single_loop.Out进行低通滤波
    static float filtered_single_out = 0.0f;
    static const float single_alpha = 0.1f;

    filtered_single_out = single_alpha * single_loop.Out + (1.0f - single_alpha) * filtered_single_out;

    servo_set(80.0f - filtered_speed_out - filtered_single_out, 100.0f + filtered_speed_out + filtered_single_out,
              100.0f - filtered_speed_out + filtered_single_out, 80.0f + filtered_speed_out - filtered_single_out);

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

void control_state_maintain_jump(void)
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

        speed_loop.Target = 2.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f;

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        if (modify_idx[road_num] == 1)
        {
            yaw_loop.Target = 0.0f;
            yaw_loop.Actual = imu.output.euler[2];
        }
        else if (modify_idx[road_num] == 2)
        {
            yaw_loop.Target = 180.0f;
            yaw_loop.Actual = imu.output.euler[2];
        }
        else if (modify_idx[road_num] == 3)
        {
            yaw_loop.Target = -180.0f;
            yaw_loop.Actual = imu.output.euler[2];
        }
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
        servo_set(20.0f, 160.0f,
                  160.0f, 20.0f);
    }

    else if (count >= 17 && count < 45)
    {
        servo_set(120.0f, 60.0f,
                  60.0f, 120.0f);
    }

    else if (count >= 45)
    {
        count = 0;
        image_block = 1;
        servo_set(90.0f, 90.0f,
                  90.0f, 90.0f);
        angle_loop2.ErrorInt = 0.0f;
        speed_loop.ErrorInt = 0.0f;
        turn_loop2.ErrorInt = 0.0f;
        current_state = STATE_MAINTAIN_JUMP;
    }
}

void control_state_de_follow(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(10.0f, spki->data_float, spkd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = 2.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f;

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

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

void control_state_tra_follow(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(10.0f, spki->data_float, spkd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = 2.0f;
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

void control_state_follow_stairs(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        if (follow_speed_change == 0)
        {
            speed_loop.Target = (float)baspeed->data_int / 100.0f;
        }
        else if (follow_speed_change == 1)
        {
            speed_loop.Target = (float)balspeed->data_int / 100.0f;
        }
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

void control_state_de_jolt(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    joypid_update(joykp->data_float, joyki->data_float, joykd->data_float);
    tu2pid_update_j(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    jospid_update(joskp->data_float, joski->data_float, joskd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)jospeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(110.0f, 70.0f,
                  110.0f, 70.0f);

        yaw_loop.Target = 0.0f;
        yaw_loop.Actual = imu.output.euler[2];
        joy_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&joy_loop);
    }

    turn_loop2.Target = -joy_loop.Out * speed_loop.Actual;
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

    if (speed_loop.Actual < 0.5f && speed_loop.Actual > -0.5f)
    {
        jolt_trap_count++;
        if (jolt_trap_count >= 20)
        {
            jolt_trap_count = 0;
            current_state = STATE_JOLT_TRAP;
        }
    }
    else
    {
        jolt_trap_count = 0;
    } 
 
    turn_loop2.Target = -joy_loop.Out * speed_loop.Actual;
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

void control_state_jolt_trap(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    angle_loop2.Ki = 0.0f;
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    joypid_update(joykp->data_float, joyki->data_float, joykd->data_float);
    tu2pid_update_j(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    jospid_update(joskp->data_float, joski->data_float, joskd->data_float);

    static uint8_t jolt_speed_change_flag = 0;
    static uint32_t jolt_speed_change_count = 0;

    static uint32_t count = 0;
    count++;

    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = (float)jospeed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        yaw_loop.Target = 0.0f;
        yaw_loop.Actual = imu.output.euler[2];
        joy_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&joy_loop);
    }

    if (speed_loop.Actual > (float)jospeed->data_int / 100.0f)
    {
        jolt_trap_count++;
        if (jolt_trap_count >= 10)
        {
            jolt_trap_count = 0;
            current_state = STATE_JOLT;
        }
    }
    else
    {
        jolt_trap_count = 0;
    }

    if (jolt_speed_change_flag == 0)
    {
        servo_set(45.0f, 135.0f,
                  40.0f, 140.0f);
        jolt_speed_change_count++;
        if (jolt_speed_change_count >= 80)
        {
            jolt_speed_change_count = 0;
            jolt_speed_change_flag = 1;
        }
    }
    else if (jolt_speed_change_flag == 1)
    {
        jolt_speed_change_flag = 0;
        jolt_trap_count = 0;
        angle_loop2.ErrorInt = 0.0f;
        speed_loop.ErrorInt = 0.0f;
        turn_loop2.ErrorInt = 0.0f;
        current_state = STATE_JOLT;
    }

    turn_loop2.Target = -joy_loop.Out * speed_loop.Actual;
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

void control_state_test(void)
{
    static uint16_t last_channel_0 = 0;

    if (fabs(last_channel_0 - 832) > 50.0f && fabs(uart_receiver.channel[0] - 832) <= 50.0f)
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

    if (fabs(uart_receiver.channel[0] - 832) <= 50.0f)
    {
        small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
    }
    else
    {
        small_driver_set_duty((int16)(aspeed_loop.Out + yaw_test_bias), (int16)(-(aspeed_loop.Out - yaw_test_bias)));
    }
}

void control_state_uturn(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    servo_set(90.0f, 90.0f,
              90.0f, 90.0f);

    turn_loop2.Target = (float)rotusp->data_int;
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));

    if (fabs(fabs(imu.output.euler[2]) - 180.0f) <= 5.0f)
    {
        speed_loop.ErrorInt = 0.0f;
        current_state = STATE_FORWARD;
    }
}

void control_state_modify(void)
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

        speed_loop.Target = (float)speed->data_int / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f;

        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        if (modify_idx[road_num] == 1)
        {
            yaw_loop.Target = 0.0f;
            yaw_loop.Actual = imu.output.euler[2];
        }
        else if (modify_idx[road_num] == 2)
        {
            yaw_loop.Target = 180.0f;
            yaw_loop.Actual = imu.output.euler[2];
        }
        else if (modify_idx[road_num] == 3)
        {
            yaw_loop.Target = -180.0f;
            yaw_loop.Actual = imu.output.euler[2];
        }
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

void control_state_stop(void)
{
    para_zero();
    out_zero();
    small_driver_set_duty(0, 0);
}
