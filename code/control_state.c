#include "control_state.h"
#include "control.h"
#include "image.h"

volatile control_state_t current_state = STATE_IDLE;

void control_state_switch(void)
{
    if (current_state == STATE_ACCELERATE && fabs(speed_loop.Target - speed_loop.Actual) < 0.1f)
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
      && single_side_state != 0
      && speed_loop.Actual < 2.5f + 0.3f)
    {
        current_state = STATE_STAND_FRONT_SINGLE_SIDE;
    }

    if ((current_state == STATE_DECLINE_FRONT_SINGLE_SIDE ||
         current_state == STATE_STAND_FRONT_SINGLE_SIDE || 
         current_state == STATE_SINGLE_SIDE) 
      && single_side_state == 0)
    {
        current_state = STATE_FORWARD;
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
    case STATE_STOP:
        control_state_stop();
        break;
    default:
        control_state_forward();
        break;
    }
}
