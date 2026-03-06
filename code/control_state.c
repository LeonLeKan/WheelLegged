#include "control_state.h"
#include "control.h"
#include "image.h"

volatile control_state_t current_state = STATE_IDLE;

void control_state_switch(void)
{
    // 临时变量
    static uint32_t switch_from_single_side_to_forward = 0;
    static uint32_t switch_from_single_side_to_forward_count = 0;

    static uint32_t switch_from_forward_to_jump_up = 0;

    if (current_state == STATE_ACCELERATE && fabs(speed_loop.Target - speed_loop.Actual) < 0.1f)
    {
        current_state = STATE_JOLT;
    }

    if ((current_state != STATE_DECLINE_FRONT_SINGLE_SIDE &&
         current_state != STATE_STAND_FRONT_SINGLE_SIDE &&  
         current_state != STATE_SINGLE_SIDE) 
      && single_side_state != 0)
    {
        current_state = STATE_DECLINE_FRONT_SINGLE_SIDE;
    }

    if (current_state == STATE_DECLINE_FRONT_SINGLE_SIDE
      && single_side_state != 0
      && speed_loop.Actual < 2.5f + 0.2f)
    {
        current_state = STATE_STAND_FRONT_SINGLE_SIDE;
    }

    if ((current_state == STATE_DECLINE_FRONT_SINGLE_SIDE ||
         current_state == STATE_STAND_FRONT_SINGLE_SIDE || 
         current_state == STATE_SINGLE_SIDE) 
      && single_side_state == 0
      && switch_from_single_side_to_forward == 0)
    {
        switch_from_single_side_to_forward = 1;
    }

    if (switch_from_single_side_to_forward == 1)
    {
        switch_from_single_side_to_forward_count++;
        if (switch_from_single_side_to_forward_count >= 50)
        {
            current_state = STATE_FORWARD;
            switch_from_single_side_to_forward = 0;
            switch_from_single_side_to_forward_count = 0;
        }
    }

    if (current_state != STATE_JUMP_UP 
     && traverse_state != 0
     && switch_from_forward_to_jump_up == 0)
    {
        switch_from_forward_to_jump_up = 1;
    }

    if (switch_from_forward_to_jump_up == 1)
    {
        image_block = 0;
        error = 0;
        current_state = STATE_ACCUMULATE;
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
    case STATE_THUNDER:
        control_state_thunder();
        break;
    case STATE_ACCUMULATE:
        control_state_accumulate();
        break;
    case STATE_JUMP_UP:
        control_state_jump_up();
        break;
    case STATE_JOLT:
        control_state_jolt();
        break;
    case STATE_STOP:
        control_state_stop();
        break;
    default:
        control_state_forward();
        break;
    }
}
