#include "location.h"
#include "pid.h"
#include "waypoint.h"

static double direction_calculate(double x_target, double y_target, double x_now, double y_now, double current_yaw)
{
    double dx = x_target - x_now;
    double dy = y_target - y_now;

    if (dx == 0.0 && dy == 0.0) 
    {
        return 0.0;
    }

    double target_angle = atan2(dy, dx);
    return target_angle;
}

// void pos_update([现在x]，[现在y]，[当前航向角])
// {
//     yaw_loop.Target = direction_calculate([目标x], [目标y], [现在x]，[现在y]，[当前航向角]);
//     if (fabs([现在x] - [目标x]) < XYMIN_BIAS && fabs([现在y] - [目标y]) < XYMIN_BIAS)
//     {
//         指向下一个路径节点;
//     }
// }