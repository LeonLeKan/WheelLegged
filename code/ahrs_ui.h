#ifndef __AHRS_UI_H__
#define __AHRS_UI_H__

#include "zf_common_headfile.h"
#include "points_display.h"
#include "waypoint.h"


void ahrs_plane_to_waypoint(double x, double y, waypoint_t* waypoint);
void ahrs_get_current_position(double* x, double* y);
waypoint_node_t* ahrs_add_current(waypoint_list_t* list, waypoint_node_t* node, waypoint_t waypoint);
waypoint_node_t* ahrs_add_next(waypoint_list_t* list, waypoint_node_t* pNode, waypoint_t waypoint);

#endif