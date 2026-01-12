#ifndef __CONTROL_STATE_H__
#define __CONTROL_STATE_H__

typedef enum {
    STATE_IDLE,
    STATE_STAND_UP,
    STATE_STAND_STILL,
    STATE_ACCELERATE,
    STATE_FORWARD,
    STATE_STAND_FRONT_SINGLE_SIDE,
    STATE_SINGLE_SIDE,
    STATE_STOP
} control_state_t;

extern volatile control_state_t current_state;

void control_state_switch(void);
void control_state_update(void);

#endif
