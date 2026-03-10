#ifndef __LOCATION_H
#define __LOCATION_H

#include "control.h"
#include "arm_math.h"
#include "small_driver_uart_control.h"
#include "config.h"
#include "imu.h"
#include "pid.h"

#define XYMIN_BIAS 0.05f

static double direction_calculate(double x_target, double y_target, double x_now, double y_now, double current_yaw);

#endif