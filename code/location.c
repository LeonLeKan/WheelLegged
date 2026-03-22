#include "location.h"
#include "pid.h"
#include "waypoint.h"
#include "save_manager.h"
#include "control_state.h"

waypoint_node_t *current_target = NULL;
float yaw_memory = 0.0f;

static double direction_calculate(double x_target, double y_target, double x_now, double y_now)
{
    static float last_raw_yaw = 0.0f, target_yaw = 0.0f;
    double dx = x_target - x_now;
    double dy = y_target - y_now;

    if (dx == 0.0 && dy == 0.0)
    {
        return 0.0;
    }

    double yaw = atan2(dy, dx) * RAD_TO_DEG;

    double delta = yaw - last_raw_yaw;
    if(delta > 180.f) delta -= 360;
    if(delta < -180.f) delta += 360;

    last_raw_yaw = yaw;

    target_yaw += delta;
    return target_yaw;
}

static void advance_to_next_waypoint(void)
{
    if (current_target->next)
    {
        current_target = current_target->next;
    }
    else
    {
        current_target = NULL;
    }
}

void location_update(void)
{
    waypoint_list_t *list = save_manager_get_active_save();
    
    if (!current_target)
    {
        current_target = list->head->next;
    }
        
    if (!current_target)
    {
        return;
    }

    double x_now = imu.output.position_x;
    double y_now = imu.output.position_y;
    float yaw_now = imu.output.euler[2];

    double x_tgt = current_target->waypoint.x;
    double y_tgt = current_target->waypoint.y;

    double target_deg = direction_calculate(x_tgt, y_tgt, x_now, y_now);

    yaw_loop.Target = target_deg;
    yaw_loop.Actual = imu.output.euler[2];

    double dx = x_tgt - x_now;
    double dy = y_tgt - y_now;
    if (fabs(dx) < XYMIN_BIAS && fabs(dy) < XYMIN_BIAS)
    {
        yaw_memory = yaw_loop.Actual;
        current_state = STATE_THUNDER;
        advance_to_next_waypoint();
    }

    if (!current_target) 
    {
        current_state = STATE_DEACCELERATE;
    }
}
