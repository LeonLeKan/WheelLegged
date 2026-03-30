#include "control_state.h"
#include "control.h"
#include "image.h"
#include "pid.h"

volatile control_state_t current_state = STATE_IDLE;
volatile uint8_t test_state = 0;
volatile float yaw_memory = 0.0f;

static void accelerate_to_forward(void)
{
    if (current_state == STATE_ACCELERATE && (speed_loop.Actual > speed_loop.Target))
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
        speed_loop.Target = 1.0f;
    }
}

static void to_single_side_decline(void)
{
    if ((current_state != STATE_DECLINE_FRONT_SINGLE_SIDE &&
         current_state != STATE_STAND_FRONT_SINGLE_SIDE &&  
         current_state != STATE_SINGLE_SIDE) 
      && single_side_state != 0)
    {
        current_state = STATE_DECLINE_FRONT_SINGLE_SIDE;
    }
}

static void to_single_side_stand(void)
{
    if (current_state == STATE_DECLINE_FRONT_SINGLE_SIDE
      && speed_loop.Actual < speed_loop.Target / 100.0f)
    {
        current_state = STATE_STAND_FRONT_SINGLE_SIDE;
    }
}

static void quit_single_side(void)
{
    static uint32_t switch_from_single_side_to_forward = 0;
    static uint32_t switch_from_single_side_to_forward_count = 0;

    if ((current_state == STATE_SINGLE_SIDE) 
      && single_side_state == 0
      && switch_from_single_side_to_forward == 0)
    {
        switch_from_single_side_to_forward = 1;
    }

    if (switch_from_single_side_to_forward == 1)
    {
        switch_from_single_side_to_forward_count++;
        if (switch_from_single_side_to_forward_count >= 60)
        {
            current_state = STATE_ACCELERATE;
            roll_base = 0.0f;
            switch_from_single_side_to_forward = 0;
            switch_from_single_side_to_forward_count = 0;
        }
    }
}

void control_state_switch(void)
{
    accelerate_to_forward();
    deaccelerate_to_stop();
    deaccelerate_to_thunder();
    to_single_side_decline();
    to_single_side_stand();
    quit_single_side();
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
