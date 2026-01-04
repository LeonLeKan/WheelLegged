#ifndef __WAYPOINT_H__
#define __WAYPOINT_H__

#include "zf_common_headfile.h"

typedef struct
{
    double x, y;  
} waypoint_t;

typedef struct
{
    double plane_x, plane_y;       // 平面坐标（米）
    int screen_x, screen_y;        // 屏幕坐标
} waypoint_screen_param_t;

typedef struct _waypoint_node
{
    waypoint_t* waypoint;               // 点数据
    waypoint_screen_param_t* screen_param; // 显示相关属性
    struct _waypoint_node* next, * prev; // 前后指针
} waypoint_node_t;

typedef struct 
{
    waypoint_node_t* head;  // 特殊节点，存储开机时的坐标，无法被删除或修改
    waypoint_node_t* tail;  // 最后一个节点，用于快速在尾部加点
    int size;               // 点的数量（包括head）
}waypoint_list_t;


waypoint_list_t* waypoint_list_create(waypoint_t waypoint, waypoint_screen_param_t screen_param);
waypoint_node_t* waypoint_list_add_next(waypoint_list_t* list, waypoint_node_t* pNode,
                                waypoint_t waypoint, waypoint_screen_param_t screen_param);
void waypoint_list_remove(waypoint_list_t* list, waypoint_node_t* pNode);
void waypoint_list_clear(waypoint_list_t* list);

void waypoint_list_save_to_flash(waypoint_list_t *list, uint32 flash_page);
bool waypoint_list_load_from_flash(waypoint_list_t *list, uint32 flash_page);


#endif