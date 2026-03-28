#ifndef __CONTROL_STATE_H__
#define __CONTROL_STATE_H__

#include "stdint.h"

typedef enum {
    STATE_IDLE,
    STATE_STAND_UP,
    STATE_STAND_STILL,
    STATE_ACCELERATE,
    STATE_DEACCELERATE,
    STATE_DE_THUNDER,
    STATE_TRA_THUNDER,
    STATE_FORWARD,
    STATE_DECLINE_FRONT_SINGLE_SIDE,
    STATE_STAND_FRONT_SINGLE_SIDE,
    STATE_SINGLE_SIDE,
    STATE_THUNDER,
    STATE_JUMP_UP,
    STATE_JOLT,
    STATE_TEST,
    STATE_STOP
} control_state_t;

extern volatile control_state_t current_state;
extern volatile uint8_t test_state;
extern volatile float yaw_memory;

void control_state_switch(void);
void control_state_update(void);

#endif
