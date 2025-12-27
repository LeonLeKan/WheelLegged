#include "gps_save_manager.h"
#include "gps_show_points.h"

int total_saves;
int active_save;
bool showing_points;


void show_saves()
{
    char text[35];
    for(int i = 0; i < total_saves; i++)
    {
        sprintf(text, "Save %02d", i);
        if(i == active_save)
            ips200_set_color(RGB565_BLACK, RGB565_CYAN);
        else
            ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        ips200_show_string(0, 16 * (1 + i), text);
    }
}

void gps_save_manager_init(void)
{
    // 添加测试数据
    // gps_add_point(last_node, 31.2304, 121.4737);
    // gps_add_point(last_node, 31.2305, 121.4738);
    // gps_add_point(last_node, 31.2306, 121.4739);
    // gps_add_point(last_node, 31.2307, 121.4740);
    // gps_add_point(last_node, 31.2308, 121.4741);
    active_save = 0;
    total_saves = 5;
    showing_points = 0;
}

void gps_show_save_manager(void)
{
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_clear();
    showing_points = 0;
    ips200_show_string(16, 0, "Saves:");
    show_saves();
}

void save_current(void)
{
    gps_save_points(GPS_FLASH_START_PAGE + active_save);
}

// ==================== 按键处理函数 ======================
void gps_save_manager_up_pressed(void)
{
    if(showing_points)
    {
        gps_show_points_up_pressed();
        return;
    }
    active_save--;
    if(active_save < 0)
        active_save = total_saves - 1;
    
    show_saves();
}

void gps_save_manager_down_pressed(void)
{
    if(showing_points)
    {
        gps_show_points_down_pressed();
        return;
    }
    active_save++;
    if(active_save >= total_saves)
        active_save = 0;
    show_saves();
}

void gps_save_manager_ok_pressed(void)
{
    if(showing_points)
    {
        gps_show_points_ok_pressed();
        return;
    }
    bool ret = gps_load_points(GPS_FLASH_START_PAGE + active_save);
    if(!ret)
    {
        ips200_set_color(RGB565_BLACK, RGB565_RED);
        ips200_show_string(0, 160, "Failed");
        return;
    }
    showing_points = 1;
    gps_show_points_init(gps_show_save_manager, save_current);
    gps_show_points_main();
}

void gps_save_manager_left_pressed(void)
{
    if(showing_points)
        gps_show_points_left_pressed();
}

void gps_save_manager_right_pressed(void)
{
    if(showing_points)
        gps_show_points_right_pressed();
}