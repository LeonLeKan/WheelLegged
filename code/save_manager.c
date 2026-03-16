#include "save_manager.h"
#include "points_display.h"
#include "screen_interface.h"
#include "ahrs_ui.h"

// 屏幕显示函数
void show_saves();
void show_ahrs_points(void);
// 回调函数
void ahrs_save_callback(void);

save_manager_t save_manager_instance;


// ==================== 接口函数 ======================
void save_manager_init(void (*exit_callback)(void))
{
    save_manager_instance.active_save = waypoint_list_create(
        (waypoint_t) {0, 0},
        (waypoint_screen_param_t) {0, 0, 0, 0}
    );
    // 默认先加载0号存档
    bool ret = waypoint_list_load_from_flash(
        save_manager_instance.active_save,
        AHRS_FLASH_START_PAGE
    );
    save_manager_instance.active_save_idx = 0;
    save_manager_instance.total_saves = 5;
    save_manager_instance.showing_points = 0;
    save_manager_instance.exit_callback = exit_callback;
}

void save_manager_show(void)
{
    ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    ips200_clear();
    save_manager_instance.showing_points = 0;
    screen_printf(0, 0, "AHRS Saves:");
    show_saves();
}

waypoint_list_t* save_manager_get_active_save(void)
{
    return save_manager_instance.active_save;
}

// ==================== 屏幕显示函数 ======================
// 显示存档管理器主界面
void show_saves()
{
    int i;
    for (i = 0; i < save_manager_instance.total_saves; i++)
    {
        if(i == save_manager_instance.active_save_idx)
            ips200_set_color(RGB565_BLACK, RGB565_CYAN);
        else
            ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        screen_printf(16, 16 * (i + 1), "Save %2d", i);
    }
    if(save_manager_instance.active_save_idx == save_manager_instance.total_saves)
        ips200_set_color(RGB565_BLACK, RGB565_CYAN);
    else 
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
    screen_printf(16, 16 * (i + 1), "Exit");
}

// 启动ahrs显点
void show_ahrs_points(void)
{
    // 初始化点显示模块
    points_display_config_t config;
    config.waypoint_list = save_manager_instance.active_save;
    config.get_current_position_callback = ahrs_get_current_position; // 获取当前定位点的回调
    config.plane_to_waypoint_callback = ahrs_plane_to_waypoint;   // 平面坐标转换为航点的回调
    config.add_point_next_callback = ahrs_add_current;          // 添加当前点的回调
    config.save_callback = ahrs_save_callback;                 // 保存回调
    config.quit_callback = save_manager_show;                 // 退出回调

    points_display_configure(&config);

    points_display_show();
}

// ==================== 辅助函数 ========================
void ahrs_save_callback(void)
{
    waypoint_list_save_to_flash(
        save_manager_instance.active_save,
        AHRS_FLASH_START_PAGE + save_manager_instance.active_save_idx
    );
}
// ==================== 按键处理函数 ======================
void save_manager_key_up_pressed(void)
{
    if(save_manager_instance.showing_points)
    {
        points_display_key_up_pressed();
        return;
    }
    save_manager_instance.active_save_idx--;
    if(save_manager_instance.active_save_idx < 0)
        save_manager_instance.active_save_idx = save_manager_instance.total_saves - 1;
    
    show_saves();
}

void save_manager_key_down_pressed(void)
{
    if(save_manager_instance.showing_points)
    {
        points_display_key_down_pressed();
        return;
    }
    save_manager_instance.active_save_idx++;
    // 最后一个是Exit菜单
    if(save_manager_instance.active_save_idx > save_manager_instance.total_saves)
        save_manager_instance.active_save_idx = 0;
    show_saves();
}

void save_manager_key_ok_pressed(void)
{
    if(save_manager_instance.showing_points)
    {
        points_display_key_ok_pressed();
        return;
    }
    if(save_manager_instance.active_save_idx == save_manager_instance.total_saves)
    {
        // 当前选中的是Exit项
        // 先改回画笔颜色
        ips200_set_color(RGB565_BLACK, RGB565_WHITE);
        save_manager_instance.exit_callback();
        return;
    }
    // todo: 根据类型加载
    bool ret = waypoint_list_load_from_flash(
        save_manager_instance.active_save,
        AHRS_FLASH_START_PAGE + save_manager_instance.active_save_idx
    );
    if (!ret)
    {
        // 加载失败，清空当前数据
        waypoint_list_clear(save_manager_instance.active_save);
    }

    save_manager_instance.showing_points = 1;
    show_ahrs_points();
}

void save_manager_key_left_pressed(void)
{
    if(save_manager_instance.showing_points)
        points_display_key_left_pressed();
}

void save_manager_key_right_pressed(void)
{
    if(save_manager_instance.showing_points)
        points_display_key_right_pressed();
}