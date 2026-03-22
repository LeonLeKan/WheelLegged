#ifndef __LOCATION_H
#define __LOCATION_H

#include "control.h"
#include "arm_math.h"
#include "small_driver_uart_control.h"
#include "config.h"
#include "imu.h"
#include "pid.h"
#include "save_manager.h" 

#define XYMIN_BIAS 0.1f

extern waypoint_node_t *current_target;
extern float yaw_memory;

void location_update(void);

#endif