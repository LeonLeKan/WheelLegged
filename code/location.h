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
#define XYMIN_THUNDER 1.0f

extern waypoint_node_t *current_target;

void location_update_0(void);
void location_update_1(void);
void location_update_2(void);

#endif