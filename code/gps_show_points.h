#ifndef __GPS_SHOW_POINTS_H__
#define __GPS_SHOW_POINTS_H__

#include "gps_core.h"

// 功能菜单项
typedef struct
{
    char* name;
    void (*func)(void);
} func_menu_t;

void gps_show_points_init(void (*quit_callback)(void), void (*save_callback)(void));

void gps_show_points_main(void);

void gps_show_points_down_pressed(void);

void gps_show_points_up_pressed(void);

void gps_show_points_left_pressed(void);

void gps_show_points_right_pressed(void);

void gps_show_points_ok_pressed(void);

#endif