#include "location.h"

volatile PID_t direction_loop;

double direction_calculate(double x_target, double y_target, 
                           double x_now, double y_now, 
                           double current_yaw)
{
    double dx = x_target - x_now;
    double dy = y_target - y_now;

    if (dx == 0.0 && dy == 0.0) 
    {
        return 0.0;
    }

    double target_angle = atan2(dy, dx);
    target_angle = atan2(sin(target_angle), cos(target_angle));
    return target_angle;
}