#include "control.h"
#include "small_driver_uart_control.h"
#include "servo.h"
#include "image.h"
#include "control_state.h"
#include "pid.h"

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
        current_state = STATE_ACCELERATE;
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
    //yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    //threshold = (uint8_t)thre->data_int;
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

        turn_loop1.Target = 0.0f; // outer
        turn_loop1.Actual = (float)error;
        //yaw_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; // inner
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

void control_state_forward(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);

    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    //yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    //threshold = (uint8_t)thre->data_int;
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

        turn_loop1.Target = 0.0f; // outer
        turn_loop1.Actual = (float)error;
        //yaw_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; // inner
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
    tu2pid_update(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
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

        turn_loop1.Target = 0.0f; // outer
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; // inner
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
    tu2pid_update(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
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
        servo_set(110.0f - filtered_speed_out, 110.0f + filtered_speed_out,
                   70.0f - filtered_speed_out, 70.0f + filtered_speed_out);

        turn_loop1.Target = 0.0f; // outer
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; // inner
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
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
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

        turn_loop1.Target = 0.0f; // outer
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    // 单边桥控制
    single_loop.Target = roll_base;
    single_loop.Actual = imu.output.euler[0];
    if (fabs(single_loop.Target - single_loop.Actual) > 2.0f)
    {
        PID_Update_Lo(&single_loop);
    }

    // 使用滤波后的速度输出
    servo_set(110.0f - filtered_speed_out - single_loop.Out, 110.0f + filtered_speed_out + single_loop.Out,
               70.0f - filtered_speed_out + single_loop.Out, 70.0f + filtered_speed_out - single_loop.Out);

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; // inner
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
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;

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

    turn_loop2.Target = 180.0f;
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

    else if(count >= 50)
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
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
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

        turn_loop1.Target = 0.0f; // outer
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    // 单边桥控制
    single_loop.Target = 0.0f;
    single_loop.Actual = imu.output.euler[0];
    PID_Update_Lo(&single_loop);

    // 使用滤波后的速度输出
    servo_set(90.0f - filtered_speed_out - single_loop.Out, 90.0f + filtered_speed_out + single_loop.Out,
              90.0f - filtered_speed_out + single_loop.Out, 90.0f + filtered_speed_out - single_loop.Out);

    turn_loop2.Target = -turn_loop1.Out * speed_loop.Actual; // inner
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
    static uint32_t count;
    count++;

    para_zero();
    out_zero();
    small_driver_set_duty(-1000, 1000);
    if (count >= 100)
    {
        count = 0;
        current_state = STATE_IDLE;

        small_driver_set_duty(0, 0);
    }
}
