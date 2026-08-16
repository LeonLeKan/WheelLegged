#include "control.h"
#include "small_driver_uart_control.h"
#include "servo.h"
#include "image.h"
#include "control_state.h"
#include "pid.h"
#include "location.h"
#include "ins.h"
#include "connecting_rod.h"

// relevant variables
volatile uint8_t image_block = 0;
volatile float roll_base = 0.0f;
volatile float yaw_test_bias = 0.0f;
volatile uint8_t jump_count = 0;
volatile float yaw_de_final = 0.0f;
volatile uint32_t jolt_trap_count = 0;
volatile float servo_right_front = 90.0f;
volatile float servo_left_front = 90.0f;
volatile float servo_right_back = 90.0f;
volatile float servo_left_back = 90.0f;
volatile uint16_t tof_distance = 0;

void control_init(void)
{
    servo_init();
    small_driver_uart_init();
    pit_ms_init(PIT_CH1, 5);
    pit_ms_init(PIT_CH2, 20);
}

void control_modify_judge(void)
{
    if (modify_idx[road_num] == 1)
    {
        yaw_loop.Target = modi1->data_float;
        yaw_loop.Actual = imu.output.euler[2];
    }
    else if (modify_idx[road_num] == 2)
    {
        yaw_loop.Target = modi2->data_float;
        yaw_loop.Actual = imu.output.euler[2];
    }
    else if (modify_idx[road_num] == 3)
    {
        yaw_loop.Target = modi3->data_float;
        yaw_loop.Actual = imu.output.euler[2];
    }
    else if (modify_idx[road_num] == 4)
    {
        yaw_loop.Target = modi4->data_float;
        yaw_loop.Actual = imu.output.euler[2];
    }
    else if (modify_idx[road_num] == 5)
    {
        yaw_loop.Target = modi5->data_float;
        yaw_loop.Actual = imu.output.euler[2];
    }
    else if (modify_idx[road_num] == 6)
    {
        yaw_loop.Target = modi6->data_float;
        yaw_loop.Actual = imu.output.euler[2];
    }
    else if (modify_idx[road_num] == 7)
    {
        yaw_loop.Target = modi7->data_float;
        yaw_loop.Actual = imu.output.euler[2];
    }
    else if (modify_idx[road_num] == 8)
    {
        yaw_loop.Target = modi8->data_float;
        yaw_loop.Actual = imu.output.euler[2];
    }
    else if (modify_idx[road_num] == 9)
    {
        yaw_loop.Target = modi9->data_float;
        yaw_loop.Actual = imu.output.euler[2];
    }
}

bool control_modify_switch(int road_num, float euler_angle)
{
    int idx = modify_idx[road_num];

    return (idx == 1 && fabs(euler_angle - modi1->data_float) < 0.5f) ||
           (idx == 2 && fabs(euler_angle - modi2->data_float) < 0.5f) ||
           (idx == 3 && fabs(euler_angle - modi3->data_float) < 0.5f) ||
           (idx == 4 && fabs(euler_angle - modi4->data_float) < 0.5f) ||
           (idx == 5 && fabs(euler_angle - modi5->data_float) < 0.5f) ||
           (idx == 6 && fabs(euler_angle - modi6->data_float) < 0.5f) ||
           (idx == 7 && fabs(euler_angle - modi7->data_float) < 0.5f) ||
           (idx == 8 && fabs(euler_angle - modi8->data_float) < 0.5f) ||
           (idx == 9 && fabs(euler_angle - modi9->data_float) < 0.5f);
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

void control_state_deaccelerate(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = 0.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 4.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 4.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        yaw_loop.Target = yaw_de_final;
        yaw_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    speed_loop2.OutMax = +5.0;
    speed_loop2.OutMin = -5.0;
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)thunsp->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 6.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 6.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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
    tu2pid_update_h(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static float yaw_thunder_now = 0.0f;
    static float yaw_thunder_last = 0.0f;
    static float yaw_thunder_diff = 0.0f;
    static uint8_t yaw_thunder_flag = 0;
    static float yaw_accumulate = 0.0f;

    static uint8_t thunder_process = 0;

    transform_xy_right(0.0f, 4.0f, &servo_right_front, &servo_right_back);
    transform_xy_left(0.0f, 4.0f, &servo_left_front, &servo_left_back);

    servo_set_01(servo_left_back, servo_right_front,
                 servo_right_back, servo_left_front);

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

        imu.rot_comp.param.yaw_compensate_cw = cwtucop1->data_float;
        imu.rot_comp.param.yaw_compensate_ccw = ccwtucop1->data_float;

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

            if (yaw_accumulate <= -670.0f)
            {
                yaw_thunder_now = 0.0f;
                yaw_thunder_last = 0.0f;
                yaw_thunder_diff = 0.0f;
                yaw_thunder_flag = 0;
                yaw_accumulate = 0.0f;
                speed_loop2.Actual = 0.0f;
                speed_loop2.ErrorInt = 0.0f;
                turn_loop2.ErrorInt = 0.0f;
                thunder_process = 0;
                imu.rot_comp.param.yaw_compensate_cw = 0;
                imu.rot_comp.param.yaw_compensate_ccw = 0;
                current_state = STATE_FORWARD;
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

        imu.rot_comp.param.yaw_compensate_cw = cwtucop1->data_float;
        imu.rot_comp.param.yaw_compensate_ccw = ccwtucop1->data_float;

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

            if (yaw_accumulate >= 670.0f)
            {
                yaw_thunder_now = 0.0f;
                yaw_thunder_last = 0.0f;
                yaw_thunder_diff = 0.0f;
                yaw_thunder_flag = 0;
                yaw_accumulate = 0.0f;
                speed_loop2.Actual = 0.0f;
                speed_loop2.ErrorInt = 0.0f;
                turn_loop2.ErrorInt = 0.0f;
                thunder_process = 0;
                imu.rot_comp.param.yaw_compensate_cw = 0;
                imu.rot_comp.param.yaw_compensate_ccw = 0;
                current_state = STATE_FORWARD;
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

        imu.rot_comp.param.yaw_compensate_cw = cwtucop1->data_float;
        imu.rot_comp.param.yaw_compensate_ccw = ccwtucop1->data_float;

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

            if (yaw_accumulate <= -670.0f)
            {
                yaw_thunder_now = 0.0f;
                yaw_thunder_last = 0.0f;
                yaw_thunder_diff = 0.0f;
                yaw_thunder_flag = 0;
                yaw_accumulate = 0.0f;
                speed_loop2.Actual = 0.0f;
                speed_loop2.ErrorInt = 0.0f;
                turn_loop2.ErrorInt = 0.0f;
                thunder_process = 0;
                imu.rot_comp.param.yaw_compensate_cw = 0;
                imu.rot_comp.param.yaw_compensate_ccw = 0;
                current_state = STATE_FORWARD;
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

        imu.rot_comp.param.yaw_compensate_cw = cwtucop1->data_float;
        imu.rot_comp.param.yaw_compensate_ccw = ccwtucop1->data_float;

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

            if (yaw_accumulate >= 670.0f)
            {
                yaw_thunder_now = 0.0f;
                yaw_thunder_last = 0.0f;
                yaw_thunder_diff = 0.0f;
                yaw_thunder_flag = 0;
                yaw_accumulate = 0.0f;
                speed_loop2.Actual = 0.0f;
                speed_loop2.ErrorInt = 0.0f;
                turn_loop2.ErrorInt = 0.0f;
                thunder_process = 0;
                imu.rot_comp.param.yaw_compensate_cw = 0;
                imu.rot_comp.param.yaw_compensate_ccw = 0;
                current_state = STATE_FORWARD;
            }
        }
    }
}

void control_state_forward(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    speed_loop2.OutMax = +5.0;
    speed_loop2.OutMin = -5.0;
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        if (speed_2_change == 2)
        {
            speed_loop2.Target = (float)uturnsp->data_int / 100.0f;
        }
        else if (speed_2_change == 0)
        {
            if (road_idx[road_num] == 0 || road_idx[road_num] == 7)
            {
                speed_loop2.Target = (float)speed0->data_int / 100.0f;
            }
            else if (road_idx[road_num] == 1 || road_idx[road_num] == 5)
            {
                speed_loop2.Target = (float)speed1->data_int / 100.0f;
            }
            else if (road_idx[road_num] == 2)
            {
                speed_loop2.Target = (float)speed2->data_int / 100.0f;
            }
            else if (road_idx[road_num] == 3)
            {
                speed_loop2.Target = (float)speed3->data_int / 100.0f;
            }
            else if (road_idx[road_num] == 4 || road_idx[road_num] == 6)
            {
                speed_loop2.Target = (float)speed4->data_int / 100.0f;
            }
        }
        else if (speed_2_change == 3)
        {
            speed_loop2.Target = (float)thsp->data_int / 100.0f;
        }
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 6.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 6.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    speed_loop2.OutMax = +5.0;
    speed_loop2.OutMin = -5.0;
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)snspeed->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 6.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 6.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        turn_loop1.Target = 0.0f;
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop2.Actual;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count % 4 == 0)
    {
        count = 0;

        speed_loop2.Target = (float)snspeed->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        speed_loop2.OutMax = +2;
        speed_loop2.OutMin = -2;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 7.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 7.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        turn_loop1.Target = 0.0f;
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop2.Actual;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    snpid_update(snkp->data_float, snki->data_float, snkd->data_float);

    static uint32_t count = 0;
    count++;

    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)snspeed->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        speed_loop2.OutMax = +2;
        speed_loop2.OutMin = -2;
        PID_Update_Lo(&speed_loop2);

        single_loop.Target = roll_base;
        single_loop.Actual = imu.output.euler[0];
        PID_Update_In(&single_loop);

        transform_xy_right(speed_loop2.Out, 7.0f + single_loop.Out, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 7.0f - single_loop.Out, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        turn_loop1.Target = 0.0f;
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop2.Actual;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        if (jump->data_int == 1 || jump->data_int == 3)
        {
            speed_loop2.Target = (float)stspeed->data_int / 100.0f;
        }
        else if (jump->data_int == 2 || jump->data_int == 4)
        {
            speed_loop2.Target = (float)sipsp->data_int / 100.0f;
        }
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 4.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 4.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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

void control_state_simple_maintain(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)sipsp->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        if (jump->data_int == 4)
        {
            speed_loop2.OutMax = +0.4;
            speed_loop2.OutMin = -0.4;
        }
        PID_Update_Lo(&speed_loop2);

        if (jump->data_int == 2)
        {
            transform_xy_right(speed_loop2.Out, 4.0f, &servo_right_front, &servo_right_back);
            transform_xy_left(speed_loop2.Out, 4.0f, &servo_left_front, &servo_left_back);
        }
        else if (jump->data_int == 4)
        {
            transform_xy_right(speed_loop2.Out, 2.6f, &servo_right_front, &servo_right_back);
            transform_xy_left(speed_loop2.Out, 2.6f, &servo_left_front, &servo_left_back);
        }

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        control_modify_judge();
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)stspeed->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        if (jump->data_int == 3)
        {
            speed_loop2.OutMax = +0.4;
            speed_loop2.OutMin = -0.4;
        }
        PID_Update_Lo(&speed_loop2);

        if (jump->data_int == 1)
        {
            transform_xy_right(speed_loop2.Out, 4.0f, &servo_right_front, &servo_right_back);
            transform_xy_left(speed_loop2.Out, 4.0f, &servo_left_front, &servo_left_back);
        }
        else if (jump->data_int == 3)
        {
            transform_xy_right(speed_loop2.Out, 2.6f, &servo_right_front, &servo_right_back);
            transform_xy_left(speed_loop2.Out, 2.6f, &servo_left_front, &servo_left_back);
        }

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        control_modify_judge();
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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


void control_state_simple_jump(void)
{
    static uint8_t simple_jump_count = 0;
    static uint8_t simple_jump_flag = 1;
    static uint32_t quit_simple_count = 0;

    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    if (simple_jump_flag == 1 && simple_jump_count == 0)
    {
        static uint32_t count = 0;
        count++;
        if (count < 20)
        {
        servo_set(40.0f, 40.0f,
                  140.0f, 140.0f);
        }

        else if (count >= 20 && count < 32)
        {
            servo_set(120.0f, 120.0f,
                      60.0f, 60.0f);
        }

        else if (count >= 32)
        {
            count = 0;
            servo_set(90.0f, 90.0f,
                      90.0f, 90.0f);
            angle_loop2.ErrorInt = 0.0f;
            speed_loop2.ErrorInt = 0.0f;
            turn_loop2.ErrorInt = 0.0f;
            simple_jump_count += 1;
            simple_jump_flag = 0;
        }

        angle_loop2.Target = pzero->data_float;
        angle_loop2.Actual = imu.output.euler[1];
        PID_Update_Lo(&angle_loop2);

        aspeed_loop.Target = angle_loop2.Out;
        aspeed_loop.Actual = imu.output.gyro[1];
        PID_Update_Lo(&aspeed_loop);

        small_driver_set_duty((int16)(aspeed_loop.Out), (int16)(-(aspeed_loop.Out)));
    }

    if (simple_jump_flag == 1 && simple_jump_count <= 2 && simple_jump_count > 0)
    {
        static uint32_t count = 0;
        count++;
        if (count < 20)
        {
        servo_set(40.0f, 40.0f,
                  140.0f, 140.0f);
        }

        else if (count >= 20 && count < 32)
        {
            servo_set(120.0f, 120.0f,
                      60.0f, 60.0f);
        }

        else if (count >= 32)
        {
            count = 0;
            servo_set(90.0f, 90.0f,
                      90.0f, 90.0f);
            angle_loop2.ErrorInt = 0.0f;
            speed_loop2.ErrorInt = 0.0f;
            turn_loop2.ErrorInt = 0.0f;
            simple_jump_count += 1;
            simple_jump_flag = 0;
        }

        angle_loop2.Target = pzero->data_float;
        angle_loop2.Actual = imu.output.euler[1];
        PID_Update_Lo(&angle_loop2);

        aspeed_loop.Target = angle_loop2.Out;
        aspeed_loop.Actual = imu.output.gyro[1];
        PID_Update_Lo(&aspeed_loop);

        small_driver_set_duty((int16)(aspeed_loop.Out), (int16)(-(aspeed_loop.Out)));
    }

    else if (simple_jump_flag == 0)
    {
        static uint32_t count1 = 0;
        static uint32_t count2 = 0;
        count1++;
        count2++;
        if (count2 >= 30 && simple_jump_count == 1)
        {
            count2 = 0;
            simple_jump_flag = 1;
        }
        else if (count2 >= 30 && simple_jump_count == 2)
        {
            count2 = 0;
            simple_jump_flag = 1;
        }
        else if (simple_jump_count >= 3)
        {
            count2 = 0;
            quit_simple_count++;
            if (quit_simple_count >= 100)
            {
                quit_simple_count = 0;
                image_block = 0;
                current_state = STATE_MODIFY;
            }
        }
        if (count1 >= 4)
        {
            count1 = 0;

            speed_loop2.Target = (float)sipsp->data_int / 100.0f;
            speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
            PID_Update_Lo(&speed_loop2);

            transform_xy_right(speed_loop2.Out, 4.0f, &servo_right_front, &servo_right_back);
            transform_xy_left(speed_loop2.Out, 4.0f, &servo_left_front, &servo_left_back);

            servo_set_01(servo_right_front, servo_left_front,
                         servo_right_back, servo_left_back);

            control_modify_judge();
            PID_Update_Lo(&yaw_loop);
        }

        turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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
}

void control_state_jump_up(void)
{
    if (jump->data_int == 1)
    {
        static uint32_t count = 0;
        count++;

        angle_loop2.Target = pzero->data_float;
        angle_loop2.Actual = imu.output.euler[1];
        PID_Update_Lo(&angle_loop2);

        aspeed_loop.Target = angle_loop2.Out;
        aspeed_loop.Actual = imu.output.gyro[1];
        PID_Update_Lo(&aspeed_loop);

        small_driver_set_duty((int16)(aspeed_loop.Out), (int16)(-(aspeed_loop.Out)));

        if (count < 15)
        {
            servo_set(20.0f, 20.0f,
                      160.0f, 160.0f);
        }

        else if (count >= 15 && count < 35)
        {
            servo_set(120.0f, 120.0f,
                      60.0f, 60.0f);
        }

        else if (count >= 35)
        {
            count = 0;
            image_block = 0;
            jump_lock = 1;
            servo_set(90.0f, 90.0f,
                      90.0f, 90.0f);
            angle_loop2.ErrorInt = 0.0f;
            speed_loop2.ErrorInt = 0.0f;
            speed_loop2.Out = 0.0f;
            turn_loop2.ErrorInt = 0.0f;
            current_state = STATE_MAINTAIN_JUMP;
        }
    }
    else if (jump->data_int == 3)
    {
        static uint32_t count = 0;
        count++;

        angle_loop2.Target = pzero->data_float;
        angle_loop2.Actual = imu.output.euler[1];
        PID_Update_Lo(&angle_loop2);

        aspeed_loop.Target = angle_loop2.Out;
        aspeed_loop.Actual = imu.output.gyro[1];
        PID_Update_Lo(&aspeed_loop);

        small_driver_set_duty((int16)(aspeed_loop.Out), (int16)(-(aspeed_loop.Out)));

        if (count < 15)
        {
            servo_set(20.0f, 20.0f,
                      160.0f, 160.0f);
        }

        else if (count >= 15 && count < 30)
        {
            servo_set(130.0f, 130.0f,
                      50.0f, 50.0f);
        }

        else if (count >= 30)
        {
            count = 0;
            image_block = 0;
            jump_lock = 1;
            servo_set(130.0f, 130.0f,
                      50.0f, 50.0f);
            angle_loop2.ErrorInt = 0.0f;
            speed_loop2.ErrorInt = 0.0f;
            speed_loop2.Out = 0.0f;
            turn_loop2.ErrorInt = 0.0f;
            current_state = STATE_MAINTAIN_JUMP;
        }
    }
}

void control_state_de_follow(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)follsp->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 4.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 4.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        turn_loop1.Target = 0.0f;
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop2.Actual;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)baspeed->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 6.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 6.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        turn_loop1.Target = 0.0f;
        turn_loop1.Actual = (float)error;
        PID_Update_Lo(&turn_loop1);
    }

    turn_loop2.Target = -turn_loop1.Out * speed_loop2.Actual;
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

void control_state_maintain_follow(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)balspeed->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 6.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 6.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        control_modify_judge();
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)jospeed->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        servo_set(110.0f, 70.0f,
                  70.0f, 110.0f);

        yaw_loop.Target = joangle->data_float;
        yaw_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    joypid_update(joykp->data_float, joyki->data_float, joykd->data_float);
    tu2pid_update_j(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    jospid_update(joskp->data_float, joski->data_float, joskd->data_float);

    static uint32_t count = 0;
    count++;

    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)jospeed->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 4.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 4.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        yaw_loop.Target = 0.0f;
        yaw_loop.Actual = imu.output.euler[2];
        joy_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&joy_loop);
    }

    if (speed_loop2.Actual < 0.5f && speed_loop2.Actual > -0.5f)
    {
        jolt_trap_count++;
        if (jolt_trap_count >= 30)
        {
            jolt_trap_count = 0;
            if (fabs(imu.output.euler[2] - joy_loop.Target) < 20.0f)
            {
                current_state = STATE_JOLT_TRAP;
            }
        }
    }
    else
    {
        jolt_trap_count = 0;
    }

    turn_loop2.Target = -joy_loop.Out;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
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

        speed_loop2.Target = (float)jospeed->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;

        yaw_loop.Target = 0.0f;
        yaw_loop.Actual = imu.output.euler[2];
        joy_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&joy_loop);
    }

    if (speed_loop2.Actual > 1.0f)
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

    if (jolt_speed_change_flag == 0 && imu.output.euler[1] <= 30.0f)
    {
        servo_set(45.0f, 135.0f,
                  135.0f, 45.0f);
        jolt_speed_change_count++;
        if (jolt_speed_change_count >= 80)
        {
            jolt_speed_change_count = 0;
            jolt_speed_change_flag = 1;
        }
    }
    else if (jolt_speed_change_flag == 0 && imu.output.euler[1] > 30.0f)
    {
        servo_set(70.0f, 110.0f,
                  110.0f, 70.0f);
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
        speed_loop2.ErrorInt = 0.0f;
        turn_loop2.ErrorInt = 0.0f;
        current_state = STATE_JOLT;
    }

    turn_loop2.Target = -joy_loop.Out * speed_loop2.Actual;
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
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_t(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 4.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 4.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        yaw_loop.Actual = imu.output.euler[2];
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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
    an2pid_update(13.0f, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sp2pid_update(1.0f, sp2ki->data_float, sp2kd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    imu.rot_comp.param.yaw_compensate_cw = cwtucop->data_float;
    imu.rot_comp.param.yaw_compensate_ccw = ccwtucop->data_float;

    speed_loop2.Target = (float)uturnsp->data_int / 100.0f;
    speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
    PID_Update_Lo(&speed_loop2);

    transform_xy_right(speed_loop2.Out, 4.0f, &servo_right_front, &servo_right_back);
    transform_xy_left(speed_loop2.Out, 4.0f, &servo_left_front, &servo_left_back);

    servo_set_01(servo_left_back, servo_right_front,
                 servo_right_back, servo_left_front);

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
        speed_loop2.ErrorInt = 0.0f;
        turn_loop2.ErrorInt = 0.0f;
        imu.rot_comp.param.yaw_compensate_cw = 0.0f;
        imu.rot_comp.param.yaw_compensate_ccw = 0.0f;
        current_state = STATE_FORWARD;
    }
}

void control_state_modify(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
    sp2pid_update(sp2kp->data_float, sp2ki->data_float, sp2kd->data_float);
    yapid_update(yakp->data_float, yaki->data_float, yakd->data_float);
    tu2pid_update_f(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);

    static uint32_t count = 0;
    count++;
    if (count >= 4)
    {
        count = 0;

        speed_loop2.Target = (float)modisp->data_int / 100.0f;
        speed_loop2.Actual = ((float)motor_value.receive_left_speed_data - (float)motor_value.receive_right_speed_data) / 2.0f / 100.0f;
        PID_Update_Lo(&speed_loop2);

        transform_xy_right(speed_loop2.Out, 4.0f, &servo_right_front, &servo_right_back);
        transform_xy_left(speed_loop2.Out, 4.0f, &servo_left_front, &servo_left_back);

        servo_set_01(servo_left_back, servo_right_front,
                     servo_right_back, servo_left_front);

        control_modify_judge();
        PID_Update_Lo(&yaw_loop);
    }

    turn_loop2.Target = -yaw_loop.Out * speed_loop2.Actual;
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
