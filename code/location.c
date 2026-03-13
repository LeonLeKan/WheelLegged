#include "location.h"
#include "pid.h"
#include "waypoint.h"
#include "save_manager.h"
#include "control_state.h"

static waypoint_node_t *current_target = NULL;

static double direction_calculate(double x_target, double y_target, double x_now, double y_now, double current_yaw)
{
    double dx = x_target - x_now;
    double dy = y_target - y_now;

    if (dx == 0.0 && dy == 0.0)
    {
        return 0.0;
    }

    double target_angle = atan2(dy, dx);
    return target_angle * RAD_TO_DEG;
}

static float normalize_deg(float ang)
{
    while (ang > 180.0f)
    {
        ang -= 360.0f;
    }
    while (ang <= -180.0f)
    {
        ang += 360.0f;
    }
    return ang;
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

void location_reset(void)
{
    current_target = NULL;
}

void location_update(void)
{
    waypoint_list_t *list = save_manager_get_active_save();
    
    if (!current_target)
    {
        current_target = list->head->next;
    }
        
    if (!current_target || !current_target->waypoint)
    {
        return;
    }

    double x_now = imu.output.position_x;
    double y_now = imu.output.position_y;
    float yaw_now = imu.output.euler[2];

    double x_tgt = current_target->waypoint->x;
    double y_tgt = current_target->waypoint->y;

    double target_deg = direction_calculate(x_tgt, y_tgt, x_now, y_now, yaw_now);
    target_deg = normalize_deg(target_deg);

    yaw_loop.Target = target_deg;
    yaw_loop.Actual = normalize_deg(yaw_now);

    double dx = x_tgt - x_now;
    double dy = y_tgt - y_now;
    if (sqrt(dx * dx + dy * dy) < XYMIN_BIAS)
    {
        advance_to_next_waypoint();
    }

    if (!current_target) 
    {
        current_state = STATE_DEACCELERATE;
    }
}
