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

static void deaccelerate_to_stop(void)
{
    if (current_state == STATE_DEACCELERATE && (speed_loop2.Actual < 0.5f))
    {
        current_state = STATE_STOP;
    }
}

static void deaccelerate_within_2(void)
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
        if(road_idx[road_num] == uturn_3_idx[0] && current_target->id_startup == uturn_3_idx[1])
        {
            if (dis_1 < dis2->data_float)
            {
                speed_2_change = 2;
            }
        }
        else
        {
            speed_2_change = 0;
        }
    }
    else if (current_target && !current_target->next)
    {
        speed_2_change = 0;
    }
}

static void decline_to_single_side_stand(void)
{
    if (current_state == STATE_DECLINE_FRONT_SINGLE_SIDE
      && speed_loop2.Actual < (float)snspeed->data_int / 100.0f)
    {
        current_state = STATE_STAND_FRONT_SINGLE_SIDE;
    }
}

static void stand_to_single_side(void)
{
    if (current_state == STATE_STAND_FRONT_SINGLE_SIDE)
    {
        single_delay++;
        if (single_delay >= 40)
        {
            single_delay = 0;
            roll_base = rollba->data_float;
            current_state = STATE_SINGLE_SIDE;
        }
    }
}

static void quit_single_side(void)
{
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
        if (single_lock == 2 && single_side_end_judge.end_flag == 1)
        {
            single_lock = 3;
        }
        if (single_lock == 3)
        {
            single_lock = 0;
            image_block = 0;
            current_state = STATE_MODIFY;
        }
    }
    else if (current_state == STATE_MODIFY)
    {
        if ((modify_idx[road_num] == 1 && fabs(imu.output.euler[2] - 0.0f) < 0.5f)   ||
            (modify_idx[road_num] == 2 && fabs(imu.output.euler[2] - 180.0f) < 0.5f) ||
            (modify_idx[road_num] == 3 && fabs(imu.output.euler[2] + 180.0f) < 0.5f))
        {
            disturb_flag = 1;
            last_raw_yaw = 0.0f;
            target_yaw = 0.0f;
            ins_reset();
            road_num++;
            save_manager_load(testnum->data_int + road_idx[road_num]);
            speed_loop2.ErrorInt = 0.0f;
            current_state = STATE_FORWARD;
            disturb_flag = 0;
        }
    }
}

static void judge_time_to_jump(void)
{
    if (current_state == STATE_MAINTAIN_JUMP && jump_lock == 0)
    {
        if (stair_distance <= stdis1->data_int && jump_count == 0 && mid_update == 1 && fabs(imu.output.euler[2]) < 15.0f && stair_distance > 0)
        {
            mid_update = 0;
            image_block = 0;
            jump_count += 1;
            current_state = STATE_JUMP_UP;
        }
        else if (stair_distance <= stdis2->data_int && jump_count == 1 && mid_update == 1 && fabs(imu.output.euler[2]) < 15.0f && stair_distance > 0)
        {
            mid_update = 0;
            image_block = 0;
            jump_count += 1;
            current_state = STATE_JUMP_UP;
        }
        else if (stair_distance <= stdis3->data_int && jump_count == 2 && mid_update == 1 && fabs(imu.output.euler[2]) < 15.0f && stair_distance > 0)
        {
            mid_update = 0;
            image_block = 0;
            jump_count += 1;
            current_state = STATE_JUMP_UP;
        }
        else if (jump_count >= 3)
        {
            current_state = STATE_MAINTAIN_JUMP;
            static uint32_t quit_image_jump_count = 0;
            quit_image_jump_count++;
            if (quit_image_jump_count >= 100)
            {
                quit_image_jump_count = 0;
                image_block = 0;
                current_state = STATE_MODIFY;
            }
        }
    }
    else if (current_state == STATE_MODIFY)
    {
        if ((modify_idx[road_num] == 1 && fabs(imu.output.euler[2] - 0.0f) < 0.5f)   ||
            (modify_idx[road_num] == 2 && fabs(imu.output.euler[2] - 180.0f) < 0.5f) ||
            (modify_idx[road_num] == 3 && fabs(imu.output.euler[2] + 180.0f) < 0.5f))
        {
            disturb_flag = 1;
            jump_count = 0;
            last_raw_yaw = 0.0f;
            target_yaw = 0.0f;
            ins_reset();
            road_num++;
            save_manager_load(testnum->data_int + road_idx[road_num]);
            speed_loop2.ErrorInt = 0.0f;
            current_state = STATE_FORWARD;
            disturb_flag = 0;
        }
    }
    if (jump_lock == 1 && jump_count < 3)
    {
        jump_lock_count++;
        if (jump_lock_count >= 30)
        {
            image_block = 1;
            jump_lock = 0;
            jump_lock_count = 0;
        }
    }
    else if (jump_lock == 1 && jump_count >= 3)
    {
        jump_lock_count++;
        if (jump_lock_count >= 30)
        {
            jump_lock = 0;
            jump_lock_count = 0;
        }
    }
}

static void simple_find_jump(void)
{
    if (current_state == STATE_SIMPLE_MAINTAIN && stair_distance <= stdis1->data_int && stair_distance > 0)
    {
        image_block = 0;
        current_state = STATE_SIMPLE_JUMP;
    }
}

static void quit_follow_stairs(void)
{
    static uint32_t not_find_follow = 0;
    static uint8_t not_find_flag = 0;
    if (current_state == STATE_DE_FOLLOW && not_find_flag == 0)
    {
        not_find_follow++;
        if (not_find_follow >= 600)
        {
            current_state = STATE_MODIFY;
        }
    }
    if (current_state == STATE_DE_FOLLOW && stair_distance <= 50 && stair_distance > 0)
    {
        not_find_flag = 1;
        current_state = STATE_FOLLOW_STAIRS;
    }
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
            if (follow_lock_count_1 >= 200)
            {
                follow_lock = 0;
                follow_lock_count_1 = 0;
                current_state = STATE_MAINTAIN_FOLLOW;
            }
        }
    }
    if (current_state == STATE_MAINTAIN_FOLLOW)
    {
        if (follow_lock == 0)
        {
            follow_lock = 1;
            follow_lock_count_2 = 0;
        }
        if (follow_lock == 1)
        {
            follow_lock_count_2++;
            if (follow_lock_count_2 >= 100)
            {
                follow_lock = 3;
                follow_lock_count_2 = 0;
            }
        }
        if (follow_lock == 3 && back_stairs_end_judge.end_flag == 1)
        {
            image_block = 0;
            current_state = STATE_MODIFY;
        }
    }
    if (current_state == STATE_MODIFY)
    {
        if ((modify_idx[road_num] == 1 && fabs(imu.output.euler[2] - 0.0f) < 0.5f)   ||
            (modify_idx[road_num] == 2 && fabs(imu.output.euler[2] - 180.0f) < 0.5f) ||
            (modify_idx[road_num] == 3 && fabs(imu.output.euler[2] + 180.0f) < 0.5f))
        {
            disturb_flag = 1;
            last_raw_yaw = 0.0f;
            target_yaw = 0.0f;
            ins_reset();
            road_num++;
            save_manager_load(testnum->data_int + road_idx[road_num]);
            speed_loop2.ErrorInt = 0.0f;
            current_state = STATE_FORWARD;
            disturb_flag = 0;
        }
    }
}

static void de_to_jolt(void)
{
    if (current_state == STATE_DE_JOLT && speed_loop2.Actual <= (float)jospeed->data_int / 100.0f + 1.0f)
    {
        current_state = STATE_JOLT;
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
        if (jolt_lock == 2 && jolt_end_judge.end_flag == 1)
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
        if ((modify_idx[road_num] == 1 && fabs(imu.output.euler[2] - 0.0f) < 0.5f)   ||
            (modify_idx[road_num] == 2 && fabs(imu.output.euler[2] - 180.0f) < 0.5f) ||
            (modify_idx[road_num] == 3 && fabs(imu.output.euler[2] + 180.0f) < 0.5f))
        {
            disturb_flag = 1;
            angle_loop2.ErrorInt = 0.0f;
            speed_loop2.ErrorInt = 0.0f;
            turn_loop2.ErrorInt = 0.0f;
            last_raw_yaw = 0.0f;
            target_yaw = 0.0f;
            ins_reset();
            road_num++;
            save_manager_load(testnum->data_int + road_idx[road_num]);
            current_state = STATE_FORWARD;
            disturb_flag = 0;
        }
    }
}

void control_state_switch(void)
{
    deaccelerate_to_stop();

    if (testnum->data_int == 2 && (road_idx[road_num] == 1 || road_idx[road_num] == 5))
    {
        deaccelerate_within_2();
        decline_to_single_side_stand();
        stand_to_single_side();
        quit_single_side();
    }

    else if (testnum->data_int == 2 && road_idx[road_num] == 2)
    {
        deaccelerate_within_2();
        judge_time_to_jump();
        simple_find_jump();
    }

    else if (testnum->data_int == 2 && road_idx[road_num] == 3)
    {
        deaccelerate_within_2();
        quit_follow_stairs();
    }

    else if (testnum->data_int == 2 && (road_idx[road_num] == 4 || road_idx[road_num] == 6))
    {
        deaccelerate_within_2();
        de_to_jolt();
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
    case STATE_DEACCELERATE:
        control_state_deaccelerate();
        break;
    case STATE_DE_THUNDER:
        control_state_de_thunder();
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
    case STATE_SIMPLE_MAINTAIN:
        control_state_simple_maintain();
        break;
    case STATE_MAINTAIN_JUMP:
        control_state_maintain_jump();
        break;
    case STATE_JUMP_UP:
        control_state_jump_up();
        break;
    case STATE_SIMPLE_JUMP:
        control_state_simple_jump();
        break;
    case STATE_DE_FOLLOW:
        control_state_de_follow();
        break;
    case STATE_FOLLOW_STAIRS:
        control_state_follow_stairs();
        break;
    case STATE_MAINTAIN_FOLLOW:
        control_state_maintain_follow();
        break;
    case STATE_DE_JOLT:
        control_state_de_jolt();
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
