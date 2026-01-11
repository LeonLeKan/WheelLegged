#include "ahrs_ui.h"
#include "imu.h"
#include "screen_interface.h"

// 回调函数
void ahrs_plane_to_waypoint(double x, double y, waypoint_t* waypoint);
void ahrs_get_current_position(double* x, double* y);
waypoint_node_t* ahrs_add_current(waypoint_list_t* list, waypoint_node_t* node, waypoint_t waypoint);
// 辅助函数
waypoint_node_t* ahrs_add_next(waypoint_list_t* list, waypoint_node_t* pNode, waypoint_t waypoint);

// ====================== 回调函数(由save_manager使用) =======================
void ahrs_plane_to_waypoint(double x, double y, waypoint_t* waypoint)
{
    waypoint->x = x;
    waypoint->y = y;
}

void ahrs_get_current_position(double* x, double* y)
{
    *x = imu.output.position_x;
    *y = imu.output.position_y;
}

waypoint_node_t* ahrs_add_current(waypoint_list_t* list, waypoint_node_t* node, waypoint_t waypoint)
{
    waypoint_screen_param_t screen_param;
    screen_param.plane_x = waypoint.x, screen_param.plane_y = waypoint.y;
    screen_param.screen_x = 0, screen_param.screen_y = 0;
    return waypoint_list_add_next(list, node, waypoint, screen_param);
}

// ====================== 辅助函数 ======================
waypoint_node_t* ahrs_add_next(waypoint_list_t* list, waypoint_node_t* pNode, waypoint_t waypoint)
{
    waypoint_screen_param_t screen_param;
    screen_param.plane_x = waypoint.x, screen_param.plane_y = waypoint.y;
    screen_param.screen_x = 0, screen_param.screen_y = 0;
    waypoint_node_t* pNew = waypoint_list_add_next(list, pNode, waypoint, screen_param);
    if (pNew == NULL) return NULL;
    return pNew;
}