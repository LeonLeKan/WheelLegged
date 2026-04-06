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

static void accelerate_to_forward(void)
{
    if (current_state == STATE_ACCELERATE && (speed_loop.Actual > (float)speed->data_int / 100.0f))
    {
        current_state = STATE_FORWARD;
    }
}

static void deaccelerate_to_stop(void)
{
    if (current_state == STATE_DEACCELERATE && (speed_loop.Actual < 0.1f))
    {
        current_state = STATE_STOP;
    }
}

static void deaccelerate_to_thunder(void)
{
    if (current_state == STATE_DE_THUNDER && (speed_loop.Actual < 1.0f))
    {
        current_state = STATE_TRA_THUNDER;
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
                ins_reset();
                image_block = 0;
                road_num++;
                save_manager_load(testnum->data_int + road_idx[road_num]);
                current_state = STATE_ACCELERATE;
            }
        }
    }
}

static void decline_to_stairs(void)
{
    if (current_state == STATE_DE_STAIRS
      && speed_loop.Actual < (float)stspeed->data_int / 100.0f)
    {
        current_state = STATE_TRA_STAIRS;
    }
}

static void judge_time_to_jump(void)
{
    if (current_state == STATE_DE_STAIRS || current_state == STATE_TRA_STAIRS)
    {
        if (stair_distance <= stdis1->data_int && jump_count == 0 && mid_update == 1)
        {
            mid_update = 0;
            image_block = 0;
            jump_count += 1;
            current_state = STATE_JUMP_UP;
        }
        else if (stair_distance <= stdis2->data_int && jump_count == 1 && mid_update == 1)
        {
            mid_update = 0;
            image_block = 0;
            jump_count += 1;
            current_state = STATE_JUMP_UP;
        }
        else if (stair_distance <= stdis3->data_int && jump_count == 2 && mid_update == 1)
        {
            mid_update = 0;
            image_block = 0;
            jump_count += 1;
            current_state = STATE_JUMP_UP;
        }
        else if (jump_count >= 3)
        {
            current_state = STATE_TRA_STAIRS;
            if (front_stairs_end_flag == 1)
            {
                jump_count = 0;
                image_block = 0;
                ins_reset();
                road_num++;
                save_manager_load(testnum->data_int + road_idx[road_num]);
                current_state = STATE_ACCELERATE;
            }
        }
    }
}

static void quit_follow_stairs(void)
{
    if (current_state == STATE_FOLLOW_STAIRS)
    {
        if (stair_distance <= stdis4->data_int && follow_stairs_count == 0 && mid_update == 1)
        {
            mid_update = 0;
            follow_stairs_count += 1;
        }
        else if (stair_distance <= stdis5->data_int && follow_stairs_count == 1 && mid_update == 1)
        {
            mid_update = 0;
            image_block = 0;
            follow_stairs_count += 1;
            current_state = STATE_CLOSE_STAIRS;
        }
        else if (stair_distance <= stdis6->data_int && follow_stairs_count == 2 && mid_update == 1)
        {
            mid_update = 0;
            image_block = 0;
            follow_stairs_count += 1;
            current_state = STATE_CLOSE_STAIRS;
        }
        else if (stair_distance <= stdis7->data_int && follow_stairs_count == 3 && mid_update == 1)
        {
            mid_update = 0;
            image_block = 0;
            follow_stairs_count += 1;
            current_state = STATE_CLOSE_STAIRS;
        }
        else if (follow_stairs_count >= 4)
        {
            follow_stairs_count = 0;
            ins_reset();
            image_block = 0;
            road_num++;
            save_manager_load(testnum->data_int + road_idx[road_num]);
            current_state = STATE_ACCELERATE;
        }
    }
}

static void de_to_tra_jolt(void)
{
    if (current_state == STATE_DE_JOLT && speed_loop.Actual <= (float)jospeed->data_int / 100.0f)
    {
        current_state = STATE_TRA_JOLT;
    }
}

static void quit_jolt(void)
{
    if (current_state == STATE_JOLT && jolt_end_flag == 1)
    {
        ins_reset();
        image_block = 0;
        road_num++;
        save_manager_load(testnum->data_int + road_idx[road_num]);
        current_state = STATE_DEACCELERATE;
    }
}

void control_state_switch(void)
{
    accelerate_to_forward();
    deaccelerate_to_stop();

    if (testnum->data_int == 1)
    {
        deaccelerate_to_thunder();
    }

    else if (testnum->data_int == 2 && road_idx[road_num] == 1)
    {
        decline_to_single_side_stand();
        stand_to_single_side();
        quit_single_side();
    }

    else if (testnum->data_int == 2 && road_idx[road_num] == 2)
    {
        decline_to_stairs();
        judge_time_to_jump();
    }

    else if (testnum->data_int == 2 && road_idx[road_num] == 3)
    {
        quit_follow_stairs();
    }

    else if (testnum->data_int == 2 && road_idx[road_num] == 4)
    {
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
    case STATE_STAND_UP:
        control_state_stand_up();
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
    case STATE_JUMP_UP:
        control_state_jump_up();
        break;
    case STATE_FOLLOW_STAIRS:
        control_state_follow_stairs();
        break;
    case STATE_CLOSE_STAIRS:
        control_state_close_stairs();
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
    case STATE_TEST:
        control_state_test();
        break;
    case STATE_STOP:
        control_state_stop();
        break;
    default:
        control_state_forward();
        break;
    }
}
