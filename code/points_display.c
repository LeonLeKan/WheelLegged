#include "points_display.h"
#include "screen_interface.h"
#include "config.h"

// 屏幕尺寸定义
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 240
#define MAX_POINTS    50

// 颜色定义
#define COLOR_BACKGROUND  RGB565_WHITE // 白色背景
#define COLOR_PATH        RGB565_RED   // 红色路径
#define COLOR_POINTS      RGB565_BLUE  // 蓝色点
#define COLOR_SELECTED    RGB565_RED   // 红色选中标识
#define COLOR_CURRENT     RGB565_BLUE  // 蓝色定位标识
#define COLOR_MENU_TEXT   RGB565_BLUE
#define COLOR_MENU_BACKGROUND 0xbffa
#define COLOR_MENU_HIGHLIGHT RGB565_CYAN

// 移动步长（米）
#define MOVE_STEP    0.05

// 全局点显示实例(单例)
points_display_instance_t points_display_instance;

// 坐标转换函数
void update_display_params(void);
void plane_to_screen(double plane_x, double plane_y, int* screen_x, int* screen_y);
void update_screen_coordinates(void);
// 绘图辅助函数
void draw_point_marker(int x, int y, uint16 color);
void draw_cross_marker(int x, int y, uint16 color);
void draw_all_points_and_path(void);
void clear_previous_display(waypoint_node_t* node);
void clear_all_previous_display(void);
void update_point_display(waypoint_node_t* node, uint16 color);
void show_path(void);
// 点操作函数
void move_selected_point(int dx, int dy);
void select_next_point(void);
void select_prev_point(void);
// 底部菜单
void new_func(char* name, void (*func)(void));
void funcs_init(void);
void show_func(void);
void show_next_func(void);
void show_prev_func(void);
// 菜单功能函数
void func_move_point(void);
void func_add_current_point_next(void);
void func_remove_point(void);
void func_save_and_quit(void);
void func_quit(void);
// 坐标显示
void show_selected_cordinates(void);
void show_current_cordinates(void);


// ===================== 接口函数 =====================
// 创建点显示信息结构体
void points_display_configure(points_display_config_t* create_info)
{
    points_display_info_t* info = &points_display_instance.info;
    // if(!info) return NULL;

    info->waypoint_list = create_info->waypoint_list;
    // 回调函数
    info->get_current_position_callback = create_info->get_current_position_callback;
    info->plane_to_waypoint_callback = create_info->plane_to_waypoint_callback;
    info->add_point_next_callback = create_info->add_point_next_callback;
    info->save_callback = create_info->save_callback;
    info->quit_callback = create_info->quit_callback;
    // 显示参数
    info->offset_x = 0.0, info->offset_y = 0.0;
    info->scale_factor = 1.0;
    info->min_x = 0.0, info->max_x = 0.0;
    info->min_y = 0.0, info->max_y = 0.0;
    // 状态变量
    info->selected_node = NULL;
    info->selected_node_idx = 0;
    info->state = NONE;
    info->moving_dx = 0.0;
    info->moving_dy = 0.0;
    // 菜单功能
    info->total_funcs = 0;
    info->active_func_idx = 0;
}

void points_display_show(void)
{
    points_display_info_t* info = &points_display_instance.info;
    ips200_set_color(COLOR_MENU_TEXT, COLOR_BACKGROUND);
    ips200_clear();

    // 默认选择第一个点
    info->selected_node = info->waypoint_list->head;
    info->selected_node_idx = 1;
    // 先获取一次当前定位点
    info->get_current_position_callback(&points_display_instance.current_position.x,
                                        &points_display_instance.current_position.y);
    // 显示路径图
    show_path();
    // 显示下方菜单
    funcs_init();
    show_func();
    // 显示选中点和当前定位点坐标
    show_selected_cordinates();

    pit_enable(SCREEN_UPDATE_PIT); // 开启100ms中断刷新当前定位点
}

void points_display_update_current_position(void)
{
    points_display_info_t* info = &points_display_instance.info;
    // 擦除之前定位点的十字标识
    draw_cross_marker(points_display_instance.screen_x, points_display_instance.screen_y, COLOR_BACKGROUND);
    // 绘制选中点的十字标识（防止被一起擦除）
    draw_cross_marker(info->selected_node->screen_param.screen_x, info->selected_node->screen_param.screen_y, COLOR_SELECTED);
    // 获取当前定位点
    info->get_current_position_callback(&points_display_instance.current_position.x,
        &points_display_instance.current_position.y);
    // 计算屏幕坐标
    plane_to_screen(points_display_instance.current_position.x, points_display_instance.current_position.y,
        &points_display_instance.screen_x, &points_display_instance.screen_y);

    // 如果非常靠近边界，刷新显示缩放
    if (points_display_instance.screen_x < 8 || points_display_instance.screen_x > SCREEN_WIDTH - 8 ||
        points_display_instance.screen_y < 8 || points_display_instance.screen_y > SCREEN_HEIGHT - 8)
    {
        clear_all_previous_display();
        show_path();
    }

    // 绘制新的定位点十字标识
    draw_cross_marker(points_display_instance.screen_x, points_display_instance.screen_y, COLOR_CURRENT);
    show_current_cordinates();
}


// ====================== 坐标转换函数 ======================
void update_display_params(void)
{
    points_display_info_t* info = &points_display_instance.info;
    if (info->waypoint_list->size < 1) return;
    
    // 找到点集的边界
    waypoint_node_t* current = info->waypoint_list->head;
    // 包括当前实时定位点
    info->min_x = points_display_instance.current_position.x;
    info->max_x = points_display_instance.current_position.x;
    info->min_y = points_display_instance.current_position.y;
    info->max_y = points_display_instance.current_position.y;

    while (current)
    {
        if (current->screen_param.plane_x < info->min_x)
            info->min_x = current->screen_param.plane_x;
        if (current->screen_param.plane_x > info->max_x)
            info->max_x = current->screen_param.plane_x;
        if (current->screen_param.plane_y < info->min_y)
            info->min_y = current->screen_param.plane_y;
        if (current->screen_param.plane_y > info->max_y)
            info->max_y = current->screen_param.plane_y;
        current = current->next;
    }

    // 计算点集的宽度和高度
    double data_width = info->max_x - info->min_x;
    double data_height = info->max_y - info->min_y;
    
    // 添加20%边距
    if (data_width == 0) data_width = 10.0;
    if (data_height == 0) data_height = 10.0;
    
    data_width *= 1.2;
    data_height *= 1.2;
    
    // 计算缩放比例
    double scale_x = (SCREEN_WIDTH * 0.9) / data_width;
    double scale_y = (SCREEN_HEIGHT * 0.9) / data_height;
    info->scale_factor = (scale_x < scale_y) ? scale_x : scale_y;
    
    // 计算偏移量（居中显示）
    info->offset_x = (SCREEN_WIDTH - (info->max_x - info->min_x) * info->scale_factor) / 2 - info->min_x * info->scale_factor;
    info->offset_y = (SCREEN_HEIGHT - (info->max_y - info->min_y) * info->scale_factor) / 2 - info->min_y * info->scale_factor;
}


// 将一个点的平面坐标转换为屏幕坐标
void plane_to_screen(double plane_x, double plane_y, int* screen_x, int* screen_y)
{
    points_display_info_t* info = &points_display_instance.info;
    *screen_x = (int)(plane_x * info->scale_factor + info->offset_x);
    *screen_y = (int)(SCREEN_HEIGHT - (plane_y * info->scale_factor + info->offset_y));
    // 边界检查
    if (*screen_x < 0) *screen_x = 0;
    if (*screen_x >= SCREEN_WIDTH) *screen_x = SCREEN_WIDTH - 1;
    if (*screen_y < 0) *screen_y = 0;
    if (*screen_y >= SCREEN_HEIGHT) *screen_y = SCREEN_HEIGHT - 1;
}


// 更新所有节点的屏幕坐标
void update_screen_coordinates(void)
{
    points_display_info_t* info = &points_display_instance.info;
    waypoint_node_t* current = info->waypoint_list->head;

    while (current)
    {
        plane_to_screen(current->screen_param.plane_x, current->screen_param.plane_y,
                        &current->screen_param.screen_x, &current->screen_param.screen_y);
        
        current = current->next;
    }

    // 更新当前定位点屏幕坐标
    plane_to_screen(points_display_instance.current_position.x, points_display_instance.current_position.y,
                    &points_display_instance.screen_x, &points_display_instance.screen_y);
}


// ====================== 绘图辅助函数 ======================

// 绘制点标记
void draw_point_marker(int x, int y, uint16 color)
{
    // 绘制3x3方块
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            screen_draw_point(x + dx, y + dy, color);
        }
    }
}

// 绘制十字标记
void draw_cross_marker(int x, int y, uint16 color)
{
    int size = 10;
    
    // 水平线
    screen_draw_line(x - size, y, x - 3, y, color);
    screen_draw_line(x + 3, y, x + size, y, color);
    
    // 垂直线
    screen_draw_line(x, y - size, x, y - 3, color);
    screen_draw_line(x, y + 3, x, y + size, color);
    
    // 中心交叉
    // screen_draw_line(x-1, y-1, x+1, y+1, color);
    // screen_draw_line(x-1, y+1, x+1, y-1, color);
}

// 绘制所有点和路径
void draw_all_points_and_path(void)
{
    points_display_info_t* info = &points_display_instance.info;
    // 绘制连接线
    waypoint_node_t* current = info->waypoint_list->head;
    while (current && current->next) {
        screen_draw_line(current->screen_param.screen_x, current->screen_param.screen_y,
                           current->next->screen_param.screen_x, current->next->screen_param.screen_y,
                           COLOR_PATH);
        current = current->next;
    }
    
    // 绘制所有点
    current = info->waypoint_list->head;
    while (current) {
        if (current == info->selected_node) {
            draw_cross_marker(current->screen_param.screen_x, current->screen_param.screen_y, COLOR_SELECTED);
            draw_point_marker(current->screen_param.screen_x, current->screen_param.screen_y, COLOR_POINTS);
        } else {
            draw_point_marker(current->screen_param.screen_x, current->screen_param.screen_y, COLOR_POINTS);
        }
        current = current->next;
    }
}

// 清除节点的旧显示
void clear_previous_display(waypoint_node_t* node)
{
    if (!node) return;
    
    // 清除点标记
    draw_point_marker(node->screen_param.screen_x, node->screen_param.screen_y, COLOR_BACKGROUND);
    
    // 清除十字标记
    draw_cross_marker(node->screen_param.screen_x, node->screen_param.screen_y, COLOR_BACKGROUND);

    // 清除连接线（前一条）
    if (node->prev) {
        screen_draw_line(node->prev->screen_param.screen_x, node->prev->screen_param.screen_y,
                           node->screen_param.screen_x, node->screen_param.screen_y, COLOR_BACKGROUND);
    }
    
    // 清除连接线（后一条）
    if (node->next) {
        screen_draw_line(node->screen_param.screen_x, node->screen_param.screen_y,
                           node->next->screen_param.screen_x, node->next->screen_param.screen_y, COLOR_BACKGROUND);
    }
}

// 清除之前显示的实时定位点和路径图
void clear_all_previous_display(void)
{
    // 清除之前实时定位点的标识
    draw_cross_marker(points_display_instance.screen_x, points_display_instance.screen_y, COLOR_BACKGROUND);
    // 清除之前显示的路径图
    points_display_info_t* info = &points_display_instance.info;
    waypoint_node_t* current = info->waypoint_list->head;
    while (current)
    {
        clear_previous_display(current);
        current = current->next;
    }
}

// 更新节点显示（可以指定颜色）
void update_point_display(waypoint_node_t* node, uint16 color)
{
    points_display_info_t* info = &points_display_instance.info;
    if (!node) return;
    
    // 绘制点标记
    draw_point_marker(node->screen_param.screen_x, node->screen_param.screen_y, color);
    
    // 绘制连接线（前一条）
    if (node->prev) {
        screen_draw_line(node->prev->screen_param.screen_x, node->prev->screen_param.screen_y,
                           node->screen_param.screen_x, node->screen_param.screen_y, COLOR_PATH);
    }
    
    // 绘制连接线（后一条）
    if (node->next) {
        screen_draw_line(node->screen_param.screen_x, node->screen_param.screen_y,
                           node->next->screen_param.screen_x, node->next->screen_param.screen_y, COLOR_PATH);
    }
    
    // 如果是选中节点，绘制十字标记
    if (node == info->selected_node) {
        draw_cross_marker(node->screen_param.screen_x, node->screen_param.screen_y, COLOR_SELECTED);
        show_selected_cordinates();
    }
}

void show_path(void)
{
    // 更新显示参数
    update_display_params();
    update_screen_coordinates();
    // 绘制路径
    draw_all_points_and_path();
}

// ====================== 点操作函数实现 ======================

// 移动选中的点
void move_selected_point(int dx, int dy) {
    points_display_info_t* info = &points_display_instance.info;
    if (!info->selected_node) return;
    
    // 清除旧显示
    clear_previous_display(info->selected_node);
    if (info->selected_node->prev) clear_previous_display(info->selected_node->prev);
    if (info->selected_node->next) clear_previous_display(info->selected_node->next);
    
    // 更新平面坐标
    info->selected_node->screen_param.plane_x += dx * MOVE_STEP;
    info->selected_node->screen_param.plane_y += dy * MOVE_STEP;

    // 累计相对移动的偏移量
    info->moving_dx += dx * MOVE_STEP;
    info->moving_dy += dy * MOVE_STEP;

    // 更新点坐标
    info->plane_to_waypoint_callback(info->selected_node->screen_param.plane_x, 
                info->selected_node->screen_param.plane_y,
                &info->selected_node->waypoint);

    // 更新屏幕坐标
    info->selected_node->screen_param.screen_x =
        (int)(info->selected_node->screen_param.plane_x * info->scale_factor + info->offset_x);
    info->selected_node->screen_param.screen_y =
        (int)(SCREEN_HEIGHT - (info->selected_node->screen_param.plane_y * info->scale_factor + info->offset_y));

    // 边界检查
    if (info->selected_node->screen_param.screen_x < 0)
        info->selected_node->screen_param.screen_x = 0;
    if (info->selected_node->screen_param.screen_x >= SCREEN_WIDTH)
        info->selected_node->screen_param.screen_x = SCREEN_WIDTH - 1;
    if (info->selected_node->screen_param.screen_y < 0)
        info->selected_node->screen_param.screen_y = 0;
    if (info->selected_node->screen_param.screen_y >= SCREEN_HEIGHT)
        info->selected_node->screen_param.screen_y = SCREEN_HEIGHT - 1;

    // 重新绘制
    if (info->selected_node->prev)
        update_point_display(info->selected_node->prev, COLOR_POINTS);
    
    update_point_display(info->selected_node, COLOR_SELECTED);

    if (info->selected_node->next)
        update_point_display(info->selected_node->next, COLOR_POINTS);
}

// 选择下一个点
void select_next_point(void)
{
    points_display_info_t* info = &points_display_instance.info;
    if (!info->selected_node->next) return;
    
    // 清除十字标记
    // draw_cross_marker(info->selected_node->screen_x, info->selected_node->screen_y, COLOR_BACKGROUND);
    clear_previous_display(info->selected_node);
    
    // 选择下一个点
    info->selected_node = info->selected_node->next;
    info->selected_node_idx++;
    
    // 绘制新选中点
    update_point_display(info->selected_node->prev, COLOR_POINTS);
    update_point_display(info->selected_node, COLOR_POINTS);
}

// 选择上一个点
void select_prev_point(void)
{
    points_display_info_t* info = &points_display_instance.info;
    if (!info->selected_node->prev) return;
    
    // 清除十字标记
    // draw_cross_marker(info->selected_node->screen_x, info->selected_node->screen_y, COLOR_BACKGROUND);
    clear_previous_display(info->selected_node);
    
    info->selected_node = info->selected_node->prev;
    info->selected_node_idx--;

    update_point_display(info->selected_node->next, COLOR_POINTS);
    update_point_display(info->selected_node, COLOR_POINTS);
}



// ====================== 坐标显示 =========================
void show_selected_cordinates(void)
{
    points_display_info_t* info = &points_display_instance.info;
    ips200_set_color(COLOR_MENU_TEXT, COLOR_MENU_BACKGROUND);
    screen_printf(0, 15 * 16, "Point Index: %d", (int)info->selected_node_idx);
    
    screen_printf(0, 17 * 16, "X/LAT %.6f", info->selected_node->waypoint.x);
    screen_printf(0, 18 * 16, "Y/LON %.6f", info->selected_node->waypoint.y);
}

void show_current_cordinates(void)
{
    points_display_info_t* info = &points_display_instance.info;
    ips200_set_color(COLOR_MENU_TEXT, COLOR_MENU_BACKGROUND);
    screen_printf(120, 15 * 16, "Current:", (int)info->selected_node_idx);    
    screen_printf(120, 17 * 16, "X/LAT %.6f", points_display_instance.current_position.x);
    screen_printf(120, 18 * 16, "Y/LON %.6f", points_display_instance.current_position.y);
}

// ====================== 菜单功能函数实现 ======================
void func_move_point(void)
{
    points_display_info_t* info = &points_display_instance.info;
    info->state = MOVING_POINT;
    info->moving_dx = 0.0;
    info->moving_dy = 0.0;
    ips200_set_color(COLOR_MENU_TEXT, COLOR_MENU_HIGHLIGHT);
    ips200_show_string(0, 304, "Moving Point...");
}

void func_move_relatively(void)
{
    points_display_info_t* info = &points_display_instance.info;
    info->state = MOVING_RELATIVELY;
    info->moving_dx = 0.0;
    info->moving_dy = 0.0;
    ips200_set_color(COLOR_MENU_TEXT, COLOR_MENU_HIGHLIGHT);
    ips200_show_string(0, 304, "Moving Relatively...");
}

void func_add_current_point_next(void)
{
    clear_all_previous_display();
    points_display_info_t* info = &points_display_instance.info;
    waypoint_t current_position = points_display_instance.current_position;
    // 添加点并修改选中的点
    info->add_point_next_callback(info->waypoint_list, info->selected_node,
            (waypoint_t) { current_position.x, current_position.y });
    info->selected_node = info->selected_node->next;
    info->selected_node_idx++;
    show_path();
}

void func_remove_point(void)
{
    points_display_info_t* info = &points_display_instance.info;
    // 特殊的头节点不可删除
    if (info->selected_node == info->waypoint_list->head) return;

    waypoint_node_t* prev = info->selected_node->prev;
    clear_all_previous_display();  // 先清除之前路径的显示
    // 移除当前点，修改选中的点为被删除点的前一个点
    waypoint_list_remove(info->waypoint_list, info->selected_node);
    info->selected_node = prev;
    info->selected_node_idx--;
    show_path();    // 显示新路径
}

void func_save_and_quit(void)
{
    points_display_info_t* info = &points_display_instance.info;
    info->save_callback();
    func_quit();
}

void func_quit(void)
{
    points_display_info_t* info = &points_display_instance.info;
    // 关闭中断
    pit_disable(SCREEN_UPDATE_PIT);
    info->quit_callback();
}

// ====================== 底部菜单实现 ========================
void new_func(char* name, void(*func)(void))
{
    points_display_info_t* info = &points_display_instance.info;
    info->funcs[info->total_funcs].name = name;
    info->funcs[info->total_funcs].func = func;
    info->total_funcs++;
}

void funcs_init(void)
{
    points_display_info_t* info = &points_display_instance.info;
    info->total_funcs = 0;
    info->active_func_idx = 0;
    new_func("Move Point        ", func_move_point);
    new_func("Move Relatively   ", func_move_relatively);
    new_func("Add Current Point ", func_add_current_point_next);
    new_func("Remove Point      ", func_remove_point);
    new_func("Save and Quit     ", func_save_and_quit);
    new_func("Quit Without Save ", func_quit);
}

void show_func(void)
{
    points_display_info_t* info = &points_display_instance.info;
    ips200_set_color(COLOR_MENU_TEXT, COLOR_MENU_BACKGROUND);
    screen_printf(0, 16 * 19, info->funcs[info->active_func_idx].name);
}

void show_next_func(void)
{
    points_display_info_t* info = &points_display_instance.info;
    info->active_func_idx++;
    if(info->active_func_idx >= info->total_funcs)
        info->active_func_idx = 0;
    show_func();
}

void show_prev_func(void)
{
    points_display_info_t* info = &points_display_instance.info;
    info->active_func_idx--;
    if(info->active_func_idx < 0)
        info->active_func_idx = info->total_funcs - 1;
    show_func();
}


// ====================== 按键功能函数实现 ======================
static void handle_none_state_key_event(key_event_t event)
{
    points_display_info_t* info = &points_display_instance.info;
    switch(event)
    {
        case KEY_DOWN: select_next_point(); break;
        case KEY_UP: select_prev_point(); break;
        case KEY_LEFT: show_prev_func(); break;
        case KEY_RIGHT: show_next_func(); break;
        case KEY_OK: info->funcs[info->active_func_idx].func(); break;
    }
}

static void handle_moving_point_state_key_event(key_event_t event)
{
    points_display_info_t* info = &points_display_instance.info;
    switch(event)
    {
        case KEY_DOWN: move_selected_point(0, -1); break;
        case KEY_UP: move_selected_point(0, 1); break;
        case KEY_LEFT: move_selected_point(-1, 0); break;
        case KEY_RIGHT: move_selected_point(1, 0); break;
        case KEY_OK:
            info->state = NONE;
            show_func();
            break;
    }
}


static void handle_moving_relatively_state_key_event(key_event_t event)
{
    points_display_info_t* info = &points_display_instance.info;
    switch(event)
    {
        case KEY_DOWN: move_selected_point(0, -1);; break;
        case KEY_UP: move_selected_point(0, 1); break;
        case KEY_LEFT: move_selected_point(-1, 0); break;
        case KEY_RIGHT: move_selected_point(1, 0); break;
        case KEY_OK:
        {
            // 将后续点按照累计的相对偏移量进行更新
            waypoint_node_t* current = info->selected_node->next;
            while (current)
            {
                // 更新平面坐标
                current->screen_param.plane_x += info->moving_dx;
                current->screen_param.plane_y += info->moving_dy;
                // 更新点坐标
                info->plane_to_waypoint_callback(current->screen_param.plane_x, 
                    current->screen_param.plane_y,
                    &current->waypoint);
                current = current->next;
            }
            info->state = NONE;
            info->moving_dx = 0.0;
            info->moving_dy = 0.0;
            show_path(); // 重新显示路径
            show_func();
            break;
        }
    }
}

void points_display_handle_key_event(key_event_t event)
{
    points_display_info_t* info = &points_display_instance.info;
    switch(info->state)
    {
        case NONE:
            handle_none_state_key_event(event);
            break;
        case MOVING_POINT:
            handle_moving_point_state_key_event(event);
            break;
        case MOVING_RELATIVELY:
            handle_moving_relatively_state_key_event(event);
            break;
    }
}

void points_display_key_down_pressed(void)
{
    points_display_handle_key_event(KEY_DOWN);
}

void points_display_key_up_pressed(void)
{
    points_display_handle_key_event(KEY_UP);
}

void points_display_key_left_pressed(void)
{
    points_display_handle_key_event(KEY_LEFT);
}

void points_display_key_right_pressed(void)
{
    points_display_handle_key_event(KEY_RIGHT);
}

void points_display_key_ok_pressed(void)
{
    points_display_handle_key_event(KEY_OK);
}
