#ifndef __INS_H__
#define __INS_H__

#include "config.h"
#include "waypoint.h"

void ins_init(void);

/// @brief 更新位置估计
/// @param yaw 弧度
/// @param v m/s
void ins_update(float yaw, float v);

void ins_get_data(float *x_pos, float *y_pos);

void ins_plane_to_waypoint(double x, double y, waypoint_t* waypoint);
void ins_get_current_position(double* x, double* y);
waypoint_node_t* ins_add_current(waypoint_list_t* list, waypoint_node_t* node, waypoint_t waypoint);
waypoint_node_t* ins_add_next(waypoint_list_t* list, waypoint_node_t* pNode, waypoint_t waypoint);

#endif // INS_H