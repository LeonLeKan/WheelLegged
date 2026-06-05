#include "control_state.h"
#include "control.h"
#include "image.h"
#include "pid.h"
#include "location.h"
#include "ins.h"

volatile control_state_t current_state = STATE_IDLE;
volatile uint8_t single_lock = 0;
volatile uint32_t single_lock_count = 0;
volatile uint8_t single_delay = 0;
volatile uint8_t test_state = 0;
volatile float yaw_thunder_memory = 0.0f;
volatile uint8_t jolt_lock = 0;
volatile uint32_t jolt_lock_count = 0;
volatile uint8_t speed_0_change = 0;
volatile uint8_t speed_1_change = 0;    
volatile uint8_t speed_2_change = 0;
volatile uint8_t jump_lock = 1;
volatile uint32_t jump_lock_count = 0;
volatile uint8_t follow_lock = 0;
volatile uint32_t follow_lock_count_1 = 0;
volatile uint32_t follow_lock_count_2 = 0;
volatile uint8_t follow_speed_change = 0;

static void accelerate_to_forward(void)
{
    if (testnum->data_int == 1)
    {
        if (current_state == STATE_ACCELERATE && (speed_loop.Actual > (float)thsp->data_int / 100.0f))
        {
            current_state = STATE_FORWARD;
        }
    }
    else if (testnum->data_int == 0 || testnum->data_int == 2)
    {
        if (current_state == STATE_ACCELERATE && (speed_loop.Actual > (float)speed->data_int / 100.0f))
        {
            current_state = STATE_FORWARD;
        }
    }
}

static void deaccelerate_to_stop(void)
{
    if (current_state == STATE_DEACCELERATE && (speed_loop.Actual < 1.0f))
    {
        current_state = STATE_STOP;
    }
}

static void deaccelerate_within_0(void)
{
    if (current_target && current_target->next)
    {
        double x_within_now = imu.output.position_x;
        double y_within_now = imu.output.position_y;
        double x_within_tgt = current_target->waypoint.x;
        double y_within_tgt = current_target->waypoint.y;
        double x_within_tgt_next = current_target->next->waypoint.x;
        double y_within_tgt_next = current_target->next->waypoint.y;

        double dis_0 = sqrt((x_within_tgt - x_within_tgt_next) * (x_within_tgt - x_within_tgt_next) + (y_within_tgt - y_within_tgt_next) * (y_within_tgt - y_within_tgt_next));
        double dis_1 = sqrt((x_within_now - x_within_tgt) * (x_within_now - x_within_tgt) + (y_within_now - y_within_tgt) * (y_within_now - y_within_tgt));
        if (current_target->id_startup != tunum->data_int && dis_0 < dis0->data_float)
        {
            if (dis_1 < dis1->data_float)
            {
                speed_0_change = 1;
            }
        }
        if(current_target->id_startup == tunum->data_int)
        {
            if (dis_1 < dis1->data_float)
            {
                speed_0_change = 2;
            }
        }
        if(current_target->id_startup != tunum->data_int && dis_0 >= dis0->data_float)
        {
            speed_0_change = 0;
        }
    }
    else if (current_target && !current_target->next)
    {
        speed_0_change = 0;
    }
}

static void deaccelerate_within_1(void)
{
    if (current_target && current_target->next)
    {
        double x_within_now = imu.output.position_x;
        double y_within_now = imu.output.position_y;
        double x_within_tgt = current_target->waypoint.x;
        double y_within_tgt = current_target->waypoint.y;
        double x_within_tgt_next = current_target->next->waypoint.x;
        double y_within_tgt_next = current_target->next->waypoint.y;

        double dis_0 = sqrt((x_within_tgt - x_within_tgt_next) * (x_within_tgt - x_within_tgt_next) + (y_within_tgt - y_within_tgt_next) * (y_within_tgt - y_within_tgt_next));
        double dis_1 = sqrt((x_within_now - x_within_tgt) * (x_within_now - x_within_tgt) + (y_within_now - y_within_tgt) * (y_within_now - y_within_tgt));
        if (current_target->id_startup != tunum->data_int && dis_0 < dis0->data_float)
        {
            if (dis_1 < dis1->data_float)
            {
                speed_1_change = 1;
            }
        }
        if(current_target->id_startup == tunum->data_int)
        {
            if (dis_1 < dis1->data_float)
            {
                speed_1_change = 2;
            }
        }
        if(current_target->id_startup != tunum->data_int && dis_0 >= dis0->data_float)
        {
            speed_1_change = 0;
        }
    }
    else if (current_target && !current_target->next)
    {
        speed_1_change = 0;
    }
}

static void deaccelerate_to_thunder(void)
{
    if (current_state == STATE_DE_THUNDER && (speed_loop.Actual < 2.5f))
    {
        current_state = STATE_TRA_THUNDER;
    }
}

static void deaccelerate_within_2(void)
{
    if (current_state == STATE_FORWARD && (fabs(imu.output.euler[2]) > 45.0f && fabs(imu.output.euler[2]) < 135.0f))
    {
        speed_2_change = 1;
    }
    else
    {
        speed_2_change = 0;
    }
}

static void decline_to_single_side_stand(void)
{
    if (current_state == STATE_DECLINE_FRONT_SINGLE_SIDE
      && speed_loop.Actual < (float)snspeed->data_int / 100.0f)
    {
        current_state = STATE_STAND_FRONT_SINGLE_SIDE;
    }
}

static void stand_to_single_side(void)
{
    if (current_state == STATE_STAND_FRONT_SINGLE_SIDE)
    {
        single_delay++;
        if (single_delay >= 60)
        {
            single_delay = 0;
            roll_base = rollba->data_float;
            current_state = STATE_SINGLE_SIDE;
        }
    }
}

static void quit_single_side(void)
{
    static uint32_t switch_from_single_side_to_forward_count = 0;

    if (current_state == STATE_SINGLE_SIDE)
    {
        if (single_lock == 0)
        {
            single_lock = 1;
            single_lock_count = 0;
        }
        if (single_lock == 1)
        {
            single_lock_count++;
            if (single_lock_count >= snlock->data_int)
            {
                single_lock = 2;
                single_lock_count = 0;
            }
        }
        if (single_lock == 2 && single_side_state == 0)
        {
            single_lock = 3;
        }
        if (single_lock == 3)
        {
            switch_from_single_side_to_forward_count++;
            if (switch_from_single_side_to_forward_count >= 100)
            {
                single_lock = 0;
                switch_from_single_side_to_forward_count = 0;
                image_block = 0;
                current_state = STATE_MODIFY;
            }
        }
    }
    else if (current_state == STATE_MODIFY)
    {
        if ((modify_idx[road_num] == 1 && fabs(imu.output.euler[2] - 0.0f) < 2.0f)   ||
            (modify_idx[road_num] == 2 && fabs(imu.output.euler[2] - 180.0f) < 2.0f) ||
            (modify_idx[road_num] == 3 && fabs(imu.output.euler[2] + 180.0f) < 2.0f))
        {
            disturb_flag = 1;
            last_raw_yaw = 0.0f;
            target_yaw = 0.0f;
            ins_reset();
            road_num++;
            save_manager_load(testnum->data_int + road_idx[road_num]);
            speed_loop.ErrorInt = 0.0f;
            current_state = STATE_ACCELERATE;
            disturb_flag = 0;
        }
    }
}

static void decline_to_stairs(void)
{
    if (current_state == STATE_DE_STAIRS
      && speed_loop.Actual < ((float)stspeed->data_int / 100.0f + 1.0f)
      && current_target)
    {
        current_state = STATE_TRA_STAIRS;
    }
}

static void judge_time_to_jump(void)
{
    if (current_state == STATE_FIND_JUMP && jump_lock == 0)
    {
        if (stair_distance <= stdis1->data_int && jump_count == 0 && mid_update == 1 && fabs(imu.output.euler[2]) < 15.0f)
        {
            mid_update = 0;
            image_block = 0;
            jump_count += 1;
            current_state = STATE_JUMP_UP;
            jump_lock = 1;
        }
        else if (stair_distance <= stdis2->data_int && jump_count == 1 && mid_update == 1 && fabs(imu.output.euler[2]) < 15.0f)
        {
            mid_update = 0;
            image_block = 0;
            jump_count += 1;
            current_state = STATE_JUMP_UP;
            jump_lock = 1;
        }
        else if (stair_distance <= stdis3->data_int && jump_count == 2 && mid_update == 1 && fabs(imu.output.euler[2]) < 15.0f)
        {
            mid_update = 0;
            image_block = 0;
            jump_count += 1;
            current_state = STATE_JUMP_UP;
            jump_lock = 1;
        }
        else if (jump_count >= 3)
        {
            current_state = STATE_FIND_JUMP;
            if (front_stairs_end_flag == 1)
            {
                image_block = 0;
                current_state = STATE_MODIFY;
            }
        }
    }
    else if (current_state == STATE_MODIFY)
    {
        if ((modify_idx[road_num] == 1 && fabs(imu.output.euler[2] - 0.0f) < 2.0f)   ||
            (modify_idx[road_num] == 2 && fabs(imu.output.euler[2] - 180.0f) < 2.0f) ||
            (modify_idx[road_num] == 3 && fabs(imu.output.euler[2] + 180.0f) < 2.0f))
        {
            disturb_flag = 1;
            jump_count = 0;
            last_raw_yaw = 0.0f;
            target_yaw = 0.0f;
            ins_reset();
            road_num++;
            save_manager_load(testnum->data_int + road_idx[road_num]);
            speed_loop.ErrorInt = 0.0f;
            current_state = STATE_ACCELERATE;
            disturb_flag = 0;
        }
    }
    if (jump_lock == 1 && jump_count < 3)
    {
        jump_lock_count++;
        if (jump_lock_count >= 120)
        {
            jump_lock = 0;
            jump_lock_count = 0;
        }
    }
    else if (jump_lock == 1 && jump_count >= 3)
    {
        jump_lock_count++;
        if (jump_lock_count >= 1)
        {
            jump_lock = 0;
            jump_lock_count = 0;
        }
    }
}

static void quit_follow_stairs(void)
{
    if (current_state == STATE_FOLLOW_STAIRS)
    {
        if (follow_lock == 0)
        {
            follow_lock = 1;
            follow_lock_count_1 = 0;
        }
        if (follow_lock == 1)
        {
            follow_lock_count_1++;
            if (follow_lock_count_1 >= 100)
            {
                follow_lock = 2;
                follow_lock_count_1 = 0;
                follow_speed_change = 1;
            }
        }
        if (follow_lock == 2)
        {
            follow_lock_count_2++;
            if (follow_lock_count_2 >= 400)
            {
                follow_lock = 3;
                follow_lock_count_2 = 0;
            }
        }
        if (follow_lock == 3 && back_stairs_end_flag == 1)
        {
            follow_lock = 4;
        }
        if (follow_lock == 4)
        {
            image_block = 0;
            current_state = STATE_MODIFY;
        }
    }
    else if (current_state == STATE_MODIFY)
    {
        if ((modify_idx[road_num] == 1 && fabs(imu.output.euler[2] - 0.0f) < 2.0f)   ||
            (modify_idx[road_num] == 2 && fabs(imu.output.euler[2] - 180.0f) < 2.0f) ||
            (modify_idx[road_num] == 3 && fabs(imu.output.euler[2] + 180.0f) < 2.0f))
        {
            disturb_flag = 1;
            follow_speed_change = 0;
            last_raw_yaw = 0.0f;
            target_yaw = 0.0f;
            ins_reset();
            road_num++;
            save_manager_load(testnum->data_int + road_idx[road_num]);
            speed_loop.ErrorInt = 0.0f;
            current_state = STATE_ACCELERATE;
            disturb_flag = 0;
        }
    }
}

static void de_to_tra_jolt(void)
{
    if (current_state == STATE_DE_JOLT && speed_loop.Actual <= (float)jospeed->data_int / 100.0f + 1.0f)
    {
        current_state = STATE_TRA_JOLT;
    }
}

static void quit_jolt(void)
{
    if (current_state == STATE_JOLT || current_state == STATE_JOLT_TRAP)
    {
        if (jolt_lock == 0)
        {
            jolt_lock = 1;
            jolt_lock_count = 0;
        }
        if (jolt_lock == 1)
        {
            jolt_lock_count++;
            if (jolt_lock_count >= jolock->data_int)
            {
                jolt_lock = 2;
                jolt_lock_count = 0;
            }
        }
        if (jolt_lock == 2 && jolt_end_flag == 1)
        {
            jolt_lock = 3;
        }
        if (jolt_lock == 3)
        {
            image_block = 0;
            current_state = STATE_MODIFY;
        }
    }
    else if (current_state == STATE_MODIFY)
    {
        if ((modify_idx[road_num] == 1 && fabs(imu.output.euler[2] - 0.0f) < 2.0f)   ||
            (modify_idx[road_num] == 2 && fabs(imu.output.euler[2] - 180.0f) < 2.0f) ||
            (modify_idx[road_num] == 3 && fabs(imu.output.euler[2] + 180.0f) < 2.0f))
        {
            disturb_flag = 1;
            angle_loop2.ErrorInt = 0.0f;
            speed_loop.ErrorInt = 0.0f;
            turn_loop2.ErrorInt = 0.0f;
            last_raw_yaw = 0.0f;
            target_yaw = 0.0f;
            ins_reset();
            road_num++;
            save_manager_load(testnum->data_int + road_idx[road_num]);
            current_state = STATE_ACCELERATE;
            disturb_flag = 0;
        }
    }
}

void control_state_switch(void)
{
    accelerate_to_forward();
    deaccelerate_to_stop();

    if (testnum->data_int == 0)
    {
        deaccelerate_within_0();
    }

    else if (testnum->data_int == 1)
    {
        if (thunder_count >= thnum->data_int)
        {
            deaccelerate_within_1();
        }
        else if (thunder_count < thnum->data_int)
        {
            deaccelerate_to_thunder();
        }
    }

    else if (testnum->data_int == 2 && (road_idx[road_num] == 1 || road_idx[road_num] == 5))
    {
        deaccelerate_within_2();
        decline_to_single_side_stand();
        stand_to_single_side();
        quit_single_side();
    }

    else if (testnum->data_int == 2 && road_idx[road_num] == 2)
    {
        deaccelerate_within_2();
        decline_to_stairs();
        judge_time_to_jump();
    }

    else if (testnum->data_int == 2 && road_idx[road_num] == 3)
    {
        deaccelerate_within_2();
        quit_follow_stairs();
    }

    else if (testnum->data_int == 2 && (road_idx[road_num] == 4 || road_idx[road_num] == 6))
    {
        deaccelerate_within_2();
        de_to_tra_jolt();
        quit_jolt();
    }
}

void control_state_update(void)
{
    switch (current_state)
    {
    case STATE_IDLE:
        control_state_idle();
        break;
    case STATE_STAND_STILL:
        control_state_stand_still();
        break;
    case STATE_ACCELERATE:
        control_state_accelerate();
        break;
    case STATE_DEACCELERATE:
        control_state_deaccelerate();
        break;
    case STATE_DE_THUNDER:
        control_state_de_thunder();
        break;
    case STATE_TRA_THUNDER:
        control_state_tra_thunder();
        break;
    case STATE_THUNDER:
        control_state_thunder();
        break;
    case STATE_FORWARD:
        control_state_forward();
        break;
    case STATE_DECLINE_FRONT_SINGLE_SIDE:
        control_state_decline_front_single_side();
        break;
    case STATE_STAND_FRONT_SINGLE_SIDE:
        control_state_stand_front_single_side();
        break;
    case STATE_SINGLE_SIDE:
        control_state_single_side();
        break;
    case STATE_DE_STAIRS:
        control_state_de_stairs();
        break;
    case STATE_TRA_STAIRS:
        control_state_tra_stairs();
        break;
    case STATE_FIND_JUMP:
        control_state_find_jump();
        break;
    case STATE_JUMP_UP:
        control_state_jump_up();
        break;
    case STATE_TRA_FOLLOW:
        control_state_tra_follow();
        break;
    case STATE_FOLLOW_STAIRS:
        control_state_follow_stairs();
        break;
    case STATE_DE_JOLT:
        control_state_de_jolt();
        break;
    case STATE_TRA_JOLT:
        control_state_tra_jolt();
        break;
    case STATE_JOLT:
        control_state_jolt();
        break;
    case STATE_JOLT_TRAP:
        control_state_jolt_trap();
        break;
    case STATE_TEST:
        control_state_test();
        break;
    case STATE_UTURN:
        control_state_uturn();
        break;
    case STATE_STOP:
        control_state_stop();
        break;
    case STATE_MODIFY:
        control_state_modify();
        break;
    default:
        control_state_forward();
        break;
    }
}
