#include "control.h"
#include "small_driver_uart_control.h"
#include "servo.h"
#include "image.h"
#include "control_state.h"
#include "pid.h"
#include "location.h"

// relevant variables
volatile uint8_t image_block = 1;
volatile float roll_base = 0.0f;

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
    if (count >= 200)
    {
        an1pid_update(an1kp->data_float, an1ki->data_float, an1kd->data_float);

        angle_loop1.Target = pinit->data_float;
        angle_loop1.Actual = imu.output.euler[1];
        PID_Update_Lo(&angle_loop1);

        small_driver_set_duty((int16)angle_loop1.Out, (int16)-angle_loop1.Out);

        if (count >= 250)
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
    // tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = speed->data_float / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(75.0f, 105.0f,
                  75.0f, 105.0f);

        // turn_loop1.Target = 0.0f; // outer
        // turn_loop1.Actual = (float)error;
        // yaw_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; // inner
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
    // tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = speed->data_float / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(120.0f, 60.0f,
                  120.0f, 60.0f);
    }
    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));
}

void control_state_forward(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);

    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    // tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = speed->data_float / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; // 速度环滤波系数，可调整

        // 更新滤波器
        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        // turn_loop1.Target = 0.0f; // outer
        // turn_loop1.Actual = (float)error;
        // yaw_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; // inner
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
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    // tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = snspeed->data_float / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(110.0f, 70.0f,
                  110.0f, 70.0f);

        yaw_loop.Target = 0.0f; // outer
        yaw_loop.Actual = (float)error;
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; // inner
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
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    // tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    static uint32_t single_delay = 0;
    count++;
    single_delay++;
    if (count % 4 == 0)
    {
        count = 0;

        speed_loop.Target = snspeed->data_float / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; // 速度环滤波系数，可调整

        // 更新滤波器
        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        // 使用滤波后的速度输出
        servo_set(100.0f - filtered_speed_out, 100.0f + filtered_speed_out,
                  80.0f - filtered_speed_out, 80.0f + filtered_speed_out);

        yaw_loop.Target = 0.0f; // outer
        yaw_loop.Actual = (float)error;
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; // inner
    turn_loop2.Actual = -imu.body.gyro[2];
    PID_Update_Lo(&turn_loop2);

    angle_loop2.Target = pzero->data_float;
    angle_loop2.Actual = imu.output.euler[1];
    PID_Update_Lo(&angle_loop2);

    aspeed_loop.Target = angle_loop2.Out;
    aspeed_loop.Actual = imu.output.gyro[1];
    PID_Update_Lo(&aspeed_loop);

    small_driver_set_duty((int16)(aspeed_loop.Out - turn_loop2.Out), (int16)(-(aspeed_loop.Out + turn_loop2.Out)));

    if (single_delay >= 60)
    {
        single_delay = 0;
        roll_base = rollba->data_float;
        current_state = STATE_SINGLE_SIDE;
    }
}

void control_state_single_side(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);

    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    // tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    snpid_update(snkp->data_float, snki->data_float, snkd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;

    // 对speed_loop.Out进行低通滤波
    static float filtered_speed_out = 0.0f;
    static const float speed_alpha = 0.1f; // 速度环滤波系数，可调整

    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = snspeed->data_float / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 更新滤波器
        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        yaw_loop.Target = 0.0f; // outer
        yaw_loop.Actual = (float)error;
        PID_Update_Lo(&yaw_loop);
    }

    // 单边桥控制
    single_loop.Target = roll_base;
    single_loop.Actual = imu.output.euler[0];
    if (fabs(single_loop.Target - single_loop.Actual) > 2.0f)
    {
        PID_Update_Lo(&single_loop);
    }

    // 使用滤波后的速度输出
    servo_set(100.0f - filtered_speed_out - single_loop.Out, 100.0f + filtered_speed_out + single_loop.Out,
              80.0f - filtered_speed_out + single_loop.Out, 80.0f + filtered_speed_out - single_loop.Out);

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; // inner
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

    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    // tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static float yaw_thunder_now = 0.0f;
    static float yaw_thunder_last = 0.0f;
    static float yaw_thunder_diff = 0.0f;
    static uint8_t yaw_thunder_flag = 0;
    static float yaw_accumulate = 0.0f;

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = 0.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; // 速度环滤波系数，可调整

        // 更新滤波器
        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);
    }

    if (yaw_memory <= 0)
    {
        turn_loop2.Target = 360.0f;
        turn_loop2.Actual = -imu.body.gyro[2];
        PID_Update_Lo(&turn_loop2);

        angle_loop2.Target = pzero->data_float;
        angle_loop2.Actual = imu.output.euler[1];
        PID_Update_Lo(&angle_loop2);

        aspeed_loop.Target = angle_loop2.Out;
        aspeed_loop.Actual = imu.output.gyro[1];
        PID_Update_Lo(&aspeed_loop);

        small_driver_set_duty((int16)(aspeed_loop.Out - 500), (int16)(-(aspeed_loop.Out + 500)));

        yaw_thunder_now = imu.output.euler[2];
        if (yaw_thunder_flag == 0)
        {
            yaw_thunder_last = yaw_memory;
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
            current_state = STATE_ACCELERATE;
        }
    }
    else if (yaw_memory > 0)
    {
        turn_loop2.Target = -360.0f;
        turn_loop2.Actual = -imu.body.gyro[2];
        PID_Update_Lo(&turn_loop2);

        angle_loop2.Target = pzero->data_float;
        angle_loop2.Actual = imu.output.euler[1];
        PID_Update_Lo(&angle_loop2);

        aspeed_loop.Target = angle_loop2.Out;
        aspeed_loop.Actual = imu.output.gyro[1];
        PID_Update_Lo(&aspeed_loop);

        small_driver_set_duty((int16)(aspeed_loop.Out + 500), (int16)(-(aspeed_loop.Out - 500)));

        yaw_thunder_now = imu.output.euler[2];
        if (yaw_thunder_flag == 0)
        {
            yaw_thunder_last = yaw_memory;
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
            current_state = STATE_ACCELERATE;
        }
    }
}

void control_state_jump_up(void)
{
    angle_loop2.Ki = 0.0f;

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
        servo_set(160.0f, 160.0f,
                  20.0f, 20.0f);
    }

    else if (count >= 17 && count < 42)
    {
        servo_set(45.0f, 45.0f,
                  135.0f, 135.0f);
    }

    else if (count >= 42 && count < 50)
    {
        servo_set(120.0f, 120.0f,
                  60.0f, 60.0f);
    }

    else if (count >= 50)
    {
        count = 0;
        image_block = 1;
        servo_set(90.0f, 90.0f,
                  90.0f, 90.0f);
        traverse_state = 0;
        angle_loop2.ErrorInt = 0.0f;
        current_state = STATE_FORWARD;
    }
}

void control_state_jolt(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);

    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    // tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    snpid_update(snkp->data_float, snki->data_float, snkd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;

    // 对speed_loop.Out进行低通滤波
    static float filtered_speed_out = 0.0f;
    static const float speed_alpha = 0.1f; // 速度环滤波系数，可调整

    if (count >= 4)
    {
        count = 0;

        speed_loop.Target = speed->data_float / 100.0f;
        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 更新滤波器
        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        yaw_loop.Target = 0.0f; // outer
        yaw_loop.Actual = (float)error;
        PID_Update_Lo(&yaw_loop);
    }

    // 单边桥控制
    single_loop.Target = 0.0f;
    single_loop.Actual = imu.output.euler[0];
    PID_Update_Lo(&single_loop);

    // 使用滤波后的速度输出
    servo_set(90.0f - filtered_speed_out - single_loop.Out, 90.0f + filtered_speed_out + single_loop.Out,
              90.0f - filtered_speed_out + single_loop.Out, 90.0f + filtered_speed_out - single_loop.Out);

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; // inner
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
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);

    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    // tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_t(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        PID_Update_Lo(&speed_loop);

        // 对speed_loop.Out进行低通滤波
        static float filtered_speed_out = 0.0f;
        static const float speed_alpha = 0.1f; // 速度环滤波系数，可调整

        // 更新滤波器
        filtered_speed_out = speed_alpha * speed_loop.Out + (1.0f - speed_alpha) * filtered_speed_out;

        servo_set(90.0f - filtered_speed_out, 90.0f + filtered_speed_out,
                  90.0f - filtered_speed_out, 90.0f + filtered_speed_out);

        // turn_loop1.Target = 0.0f; // outer
        // turn_loop1.Actual = (float)error;
        yaw_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop.Actual; // inner
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
