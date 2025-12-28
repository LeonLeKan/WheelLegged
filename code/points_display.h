#ifndef __POINTS_DISPLAY_H__
#define __POINTS_DISPLAY_H__

#include "zf_common_headfile.h"
#include "waypoint.h"


typedef struct
{
    char* name;
    void (*func)(void);
} func_menu_t;

typedef struct
{
    waypoint_list_t* waypoint_list; // 点链表
    void (*get_current_position_callback)(double* x, double* y); // 获取当前定位点回调
    void (*plane_to_waypoint_callback)(double x, double y, waypoint_t* waypoint);
    void (*add_current_callback)(waypoint_list_t* list, waypoint_node_t* node);
    void (*save_callback)(void);
    void (*quit_callback)(void);
}points_display_create_info_t;


typedef struct
{
    waypoint_list_t* waypoint_list; // 点链表
    // 回调函数
    void (*get_current_position_callback)(double* x, double* y);
    void (*plane_to_waypoint_callback)(double x, double y, waypoint_t* waypoint);
    void (*add_current_callback)(waypoint_list_t* list, waypoint_node_t* node);
    void (*save_callback)(void);
    void (*quit_callback)(void);
    // 显示参数
    double offset_x;      // 平面坐标偏移量X（米）
    double offset_y;      // 平面坐标偏移量Y（米）
    double scale_factor;  // 屏幕缩放比例（像素/米）
    double selected_node_idx; // 选中节点索引
    double min_x, max_x, min_y, max_y; // 点集边界
    // 状态变量
    waypoint_node_t* selected_node; // 选中节点指针
    bool moving_point; // 是否处于移动点状态
    // 菜单功能
    func_menu_t funcs[10];
    int total_funcs;
    int active_func_idx;
}points_display_info_t;


typedef struct
{
    points_display_info_t* info;
    waypoint_t current_position; // 当前定位点
    int screen_x, screen_y; // 当前定位点屏幕坐标
}points_display_instance_t;

// 全局点显示实例(单例)
extern points_display_instance_t points_display_instance;


// 创建点显示信息结构体
points_display_info_t* points_display_create(points_display_create_info_t* create_info);
void points_display_show(points_display_info_t* info);
void points_display_update_current_position(void);

// 按键功能函数
void points_display_key_down_pressed(void);
void points_display_key_up_pressed(void);
void points_display_key_left_pressed(void);
void points_display_key_right_pressed(void);
void points_display_key_ok_pressed(void);


#endif
