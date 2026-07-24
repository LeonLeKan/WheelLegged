#ifndef __CONTROL_STATE_H__
#define __CONTROL_STATE_H__

#include "stdint.h"

typedef enum {
    STATE_IDLE,
    STATE_STAND_STILL,
    STATE_ACCELERATE,
    STATE_DEACCELERATE,
    STATE_DE_THUNDER,
    STATE_TRA_THUNDER,
    STATE_THUNDER,
    STATE_FORWARD,
    STATE_DECLINE_FRONT_SINGLE_SIDE,
    STATE_STAND_FRONT_SINGLE_SIDE,
    STATE_SINGLE_SIDE,
    STATE_DE_STAIRS,
    STATE_TRA_STAIRS,
    STATE_FIND_JUMP,
    STATE_SIMPLE_MAINTAIN,
    STATE_MAINTAIN_JUMP,
    STATE_SIMPLE_FIND,
    STATE_SIMPLE_JUMP,
    STATE_JUMP_UP,
    STATE_DE_FOLLOW,
    STATE_TRA_FOLLOW,
    STATE_FOLLOW_STAIRS,
    STATE_MAINTAIN_FOLLOW,
    STATE_DE_JOLT,
    STATE_JOLT,
    STATE_JOLT_TRAP,
    STATE_TEST,
    STATE_UTURN,
    STATE_MODIFY,
    STATE_STOP
} control_state_t;

extern volatile control_state_t current_state;
extern volatile uint8_t single_lock;
extern volatile uint32_t single_lock_count;
extern volatile uint8_t single_delay;
extern volatile uint8_t test_state;
extern volatile float yaw_thunder_memory;
extern volatile uint8_t jolt_lock;
extern volatile uint32_t jolt_lock_count;
extern volatile uint8_t speed_0_change;
extern volatile uint8_t speed_1_change; 
extern volatile uint8_t speed_2_change;
extern volatile uint8_t jump_lock;
extern volatile uint32_t jump_lock_count;
extern volatile uint8_t follow_lock;
extern volatile uint32_t follow_lock_count_1;
extern volatile uint32_t follow_lock_count_2;
extern volatile uint8_t follow_speed_change;

void control_state_switch(void);
void control_state_update(void);

#endif
