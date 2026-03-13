#include "control_state.h"
#include "control.h"
#include "image.h"
#include "pid.h"

volatile control_state_t current_state = STATE_IDLE;

void control_state_switch(void)
{
    // 临时变量
    static uint32_t switch_from_single_side_to_forward = 0;
    static uint32_t switch_from_single_side_to_forward_count = 0;

    if (current_state == STATE_ACCELERATE && (speed_loop.Actual > speed_loop.Target))
    {
        current_state = STATE_FORWARD;
    }

    if ((current_state != STATE_DECLINE_FRONT_SINGLE_SIDE &&
         current_state != STATE_STAND_FRONT_SINGLE_SIDE &&  
         current_state != STATE_SINGLE_SIDE) 
      && single_side_state != 0)
    {
        current_state = STATE_DECLINE_FRONT_SINGLE_SIDE;
    }

    if (current_state == STATE_DECLINE_FRONT_SINGLE_SIDE
      && speed_loop.Actual < snspeed->data_float / 100.0f)
    {
        current_state = STATE_STAND_FRONT_SINGLE_SIDE;
    }

    if ((current_state == STATE_SINGLE_SIDE) 
      && single_side_state == 0
      && switch_from_single_side_to_forward == 0)
    {
        switch_from_single_side_to_forward = 1;
    }

    if (switch_from_single_side_to_forward == 1)
    {
        switch_from_single_side_to_forward_count++;
        if (switch_from_single_side_to_forward_count >= 100)
        {
            current_state = STATE_ACCELERATE;
            roll_base = 0.0f;
            switch_from_single_side_to_forward = 0;
            switch_from_single_side_to_forward_count = 0;
        }
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
