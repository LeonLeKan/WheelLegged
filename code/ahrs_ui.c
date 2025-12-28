#include "ahrs_ui.h"
#include "imu.h"
#include "screen_interface.h"

// 回调函数
void ahrs_plane_to_waypoint(double x, double y, waypoint_t* waypoint);
void ahrs_get_current_position(double* x, double* y);

waypoint_list_t* now_list;
points_display_info_t* info;

// ====================== 接口函数 ======================
// AHRS界面初始化
void ahrs_ui_init(void)
{
    now_list = waypoint_list_create();
    waypoint_list_add_next(now_list, now_list->root, (waypoint_t){0.1, 0});
    // 初始化点显示模块
    points_display_create_info_t create_info;
    create_info.waypoint_list = now_list;
    create_info.get_current_position_callback = ahrs_get_current_position; // 需要用户实现获取当前定位点的回调
    create_info.plane_to_waypoint_callback = ahrs_plane_to_waypoint;   // 需要用户实现平面坐标转换为航点的回调
    create_info.add_current_callback = NULL;          // 需要用户实现添加当前点的回调
    create_info.save_callback = ips200_clear;                 // 需要用户实现保存回调
    create_info.quit_callback = ips200_clear;                 // 需要用户实现退出回调

    info = points_display_create(&create_info);
    // points_display_show(info);
}

void ahrs_ui_show(void)
{
    // 显示点界面
    points_display_show(info);
}

// ====================== 回调函数 =======================
void ahrs_plane_to_waypoint(double x, double y, waypoint_t* waypoint)
{
    waypoint->x = x;
    waypoint->y = y;
}

void ahrs_get_current_position(double* x, double* y)
{
    *x = imu.ukf_filter.output.position_x;
    *y = imu.ukf_filter.output.position_y;
}