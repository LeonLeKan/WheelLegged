#include "ins.h"
#include "imu.h"

double x, y;

bool stopped = false;

// ====================== 接口函数 ======================
void ins_init(void)
{
    x = 0, y = 0;
}

void ins_update(double yaw, double v)
{
    if(stopped) return;
    // 简单的积分计算位置（仅供示例，实际应用中需要更复杂的算法）
    x += v * cosf(yaw) * (IMU_UPDATE_INTERVAL_MS / 1000.0f);
    y += v * sinf(yaw) * (IMU_UPDATE_INTERVAL_MS / 1000.0f);
}

void ins_get_data(double *x_pos, double *y_pos)
{
    *x_pos = x;
    *y_pos = y;
}

void ins_reset(void)
{
    stopped = true;
    x = 0;
    y = 0;
    imu_reset_yaw();
    stopped = false;
}

void ins_stop(void)
{
    stopped = true;
}

void ins_start(void)
{
    stopped = false;
}


// ====================== 点操作函数 ======================
void ins_plane_to_waypoint(double x, double y, waypoint_t* waypoint)
{
    waypoint->x = x;
    waypoint->y = y;
}

void ins_get_current_position(double* x, double* y)
{
    *x = imu.output.position_x;
    *y = imu.output.position_y;
}

waypoint_node_t* ins_add_current(waypoint_list_t* list, waypoint_node_t* node, waypoint_t waypoint)
{
    waypoint_screen_param_t screen_param;
    screen_param.plane_x = waypoint.x, screen_param.plane_y = waypoint.y;
    screen_param.screen_x = 0, screen_param.screen_y = 0;
    return waypoint_list_add_next(list, node, waypoint, screen_param);
}

// ====================== 辅助函数 ======================
waypoint_node_t* ins_add_next(waypoint_list_t* list, waypoint_node_t* pNode, waypoint_t waypoint)
{
    waypoint_screen_param_t screen_param;
    screen_param.plane_x = waypoint.x, screen_param.plane_y = waypoint.y;
    screen_param.screen_x = 0, screen_param.screen_y = 0;
    waypoint_node_t* pNew = waypoint_list_add_next(list, pNode, waypoint, screen_param);
    if (pNew == NULL) return NULL;
    return pNew;
}

