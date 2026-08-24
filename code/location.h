#ifndef __LOCATION_H
#define __LOCATION_H

#include "control.h"
#include "arm_math.h"
#include "small_driver_uart_control.h"
#include "config.h"
#include "imu.h"
#include "pid.h"
#include "save_manager.h" 

#define XYMIN_BIA_0 0.1f
#define XYMIN_BIA_1 0.1f
#define XYMIN_BIA_2 0.1f

extern waypoint_node_t *current_target;
extern uint8_t road_idx[5];
extern uint8_t modify_idx[5];
extern uint8_t uturn_3_idx[2];
extern uint8_t thun_3_idx[2];
extern uint8_t road_num;
extern uint8_t disturb_flag;
extern float last_raw_yaw;
extern float target_yaw;
extern uint8_t thunder_direction_flag;
extern double thunder_final_angle;

void location_update_2_0(void);
void location_update_2_1(void);
void location_update_2_2(void);
void location_update_2_3(void);
void location_update_2_4(void);

#endif