#include "control.h"
#include "small_driver_uart_control.h"
#include "servo.h"
#include "image.h"
#include "control_state.h"
#include "menu.h"

volatile PID_t angle_loop1;
volatile PID_t angle_loop2;
volatile PID_t speed_loop;
volatile PID_t turn_loop1;
volatile PID_t turn_loop2;
volatile PID_t aspeed_loop;
volatile PID_t single_loop;

volatile uint8_t image_block = 1;

volatile float roll_base = 0.0f;

void control_init(void)
{
    servo_init();
    small_driver_uart_init();
    pit_ms_init(PIT_CH1, 5);
}

void PID_Update_Lo(volatile PID_t *p)
{
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    p->ErrorInt += p->Error0;

    if (p->ErrorInt > p->ErrorIntMax)
    {
        p->ErrorInt = p->ErrorIntMax;
    }
    else if (p->ErrorInt < -p->ErrorIntMax)
    {
        p->ErrorInt = -p->ErrorIntMax;
    }

    p->Out = p->Kp * p->Error0 + p->Ki * p->ErrorInt + p->Kd * (p->Error0 - p->Error1);

    if (p->Out > p->OutMax)
    {
        p->Out = p->OutMax;
    }
    if (p->Out < p->OutMin)
    {
        p->Out = p->OutMin;
    }
}

void PID_Update_In(volatile PID_t *p)
{
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;
    p->Error2 = p->Error1;

    p->Out += p->Kp * (p->Error0 - p->Error1) + p->Ki * p->Error0 + p->Kd * (p->Error0 - 2 * p->Error1 + p->Error2);

    if (p->Out > p->OutMax)
    {
        p->Out = p->OutMax;
    }
    if (p->Out < p->OutMin)
    {
        p->Out = p->OutMin;
    }
}

void an1pid_update(float kp, float ki, float kd)
{
    angle_loop1.Kp = kp;
    angle_loop1.Ki = ki;
    angle_loop1.Kd = kd;
    angle_loop1.OutMax = +7000;
    angle_loop1.OutMin = -7000;
}

void an1_zero(void)
{
    angle_loop1.Kp = 0;
    angle_loop1.Ki = 0;
    angle_loop1.Kd = 0;
}

void an2pid_update(float kp, float ki, float kd)
{
    angle_loop2.Kp = kp;
    angle_loop2.Ki = ki;
    angle_loop2.Kd = kd;
    angle_loop2.OutMax = +500;
    angle_loop2.OutMin = -500;
    angle_loop2.ErrorIntMax = 600;
}

void an2_zero(void)
{
    angle_loop2.Kp = 0;
    angle_loop2.Ki = 0;
    angle_loop2.Kd = 0;
}

void acpid_update(float ki, float kd)
{
    speed_loop.Kp = 3.0f;
    speed_loop.Ki = ki;
    speed_loop.Kd = kd;
    speed_loop.OutMax = +30;
    speed_loop.OutMin = -30;
    speed_loop.ErrorIntMax = 300;
}

void sppid_update(float kp, float ki, float kd)
{
    speed_loop.Kp = kp;
    speed_loop.Ki = ki;
    speed_loop.Kd = kd;
    speed_loop.OutMax = +30;
    speed_loop.OutMin = -30;
    speed_loop.ErrorIntMax = 300;
}

void sp_zero(void)
{
    speed_loop.Kp = 0;
    speed_loop.Ki = 0;
    speed_loop.Kd = 0;
}

void tu1pid_update(float kp, float ki, float kd)
{
    turn_loop1.Kp = kp;
    turn_loop1.Ki = ki;
    turn_loop1.Kd = kd;
    turn_loop1.OutMax = +2500;
    turn_loop1.OutMin = -2500;
}

void tu1_zero(void)
{
    turn_loop1.Kp = 0;
    turn_loop1.Ki = 0;
    turn_loop1.Kd = 0;
}

void tu2pid_update(float kp, float ki, float kd)
{
    turn_loop2.Kp = kp;
    turn_loop2.Ki = ki;
    turn_loop2.Kd = kd;
    turn_loop2.OutMax = +2500;
    turn_loop2.OutMin = -2500;
    turn_loop2.ErrorIntMax = 200;
}

void tu2_zero(void)
{
    turn_loop2.Kp = 0;
    turn_loop2.Ki = 0;
    turn_loop2.Kd = 0;
}

void aspid_update(float kp, float ki, float kd)
{
    aspeed_loop.Kp = kp;
    aspeed_loop.Ki = ki;
    aspeed_loop.Kd = kd;
    aspeed_loop.OutMax = +7000;
    aspeed_loop.OutMin = -7000;
}

void as_zero(void)
{
    aspeed_loop.Kp = 0;
    aspeed_loop.Ki = 0;
    aspeed_loop.Kd = 0;
}

void snpid_update(float kp, float ki, float kd)
{
    single_loop.Kp = kp;
    single_loop.Ki = ki;
    single_loop.Kd = kd;
    single_loop.OutMax = +100;
    single_loop.OutMin = -100;
}

void sn_zero(void)
{
    single_loop.Kp = 0;
    single_loop.Ki = 0;
    single_loop.Kd = 0;
}

void para_zero(void)
{
    an1_zero();
    an2_zero();
    sp_zero();
    tu1_zero();
    tu2_zero();
    as_zero();
    sn_zero();
}

void out_zero(void)
{
    angle_loop1.Out = 0.0f;
    angle_loop2.Out = 0.0f;
    speed_loop.Out = 0.0f;
    turn_loop1.Out = 0.0f;
    turn_loop2.Out = 0.0f;
    aspeed_loop.Out = 0.0f;
    single_loop.Out = 0.0f;
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

    acpid_update(spki->data_float, spkd->data_float);
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

        servo_set(95.0f, 85.0f,
                  95.0f, 85.0f);

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
        servo_set(100.0f - filtered_speed_out, 100.0f + filtered_speed_out,
                   80.0f - filtered_speed_out, 80.0f + filtered_speed_out);

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
        roll_base = imu.output.euler[0];
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
    PID_Update_Lo(&single_loop);

    // 使用滤波后的速度输出
    servo_set(100.0f - filtered_speed_out - single_loop.Out, 100.0f + filtered_speed_out + single_loop.Out,
               80.0f - filtered_speed_out + single_loop.Out, 80.0f + filtered_speed_out - single_loop.Out);

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

    if (count < 20)
    {
        servo_set(160.0f, 160.0f,
                   20.0f, 20.0f);
    }

    else if (count >= 20 && count < 30)
    {
        servo_set(60.0f, 60.0f,
                  120.0f, 120.0f);
    }

    else if (count >= 30 && count < 60)
    {
        servo_set(120.0f, 120.0f,
                   60.0f, 60.0f);
    }

    else if (count >= 60)
    {
        count = 0;
        servo_set(90.0f, 90.0f,
                  90.0f, 90.0f);
        image_block = 1;
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
