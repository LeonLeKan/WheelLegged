#include "location.h"

volatile PID_t location_loop;

void yaw_error_calculate(double x_target, double y_target, 
                         double x_now, double y_now, 
                         double current_yaw, double *error)
{
    double dx = x_target - x_now;
    double dy = y_target - y_now;

    if (dx == 0.0 && dy == 0.0) 
    {
        *error = 0.0;
        return;
    }

    double target_angle = atan2(dy, dx);
    
    double angle_diff = target_angle - current_yaw;
    
    angle_diff = atan2(sin(angle_diff), cos(angle_diff));
    
    *error = angle_diff;
}

void location_update(double x_target, double y_target, 
                     double x_now, double y_now, 
                     double current_yaw, double *error)
{
    double yaw_error;
    float dis = sqrt(pow(x_target - x_now, 2) + pow(y_target - y_now, 2));
    yaw_error_calculate(x_target, y_target, x_now, y_now, current_yaw, &yaw_error);
    
    location_loop.Target = 0.0;
    location_loop.Actual = yaw_error;
}
