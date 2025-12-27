#include "gps_show_points.h"
#include "zf_common_headfile.h"

#define DEG_TO_RAD PI / 180.0
#define RAD_TO_DEG 180.0 / PI
// 地球半径（米）
#define EARTH_RADIUS 6371000.0


// 屏幕尺寸定义
#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 240
#define MAX_POINTS    50

// 颜色定义
#define COLOR_BACKGROUND  RGB565_WHITE // 白色背景
#define COLOR_PATH        RGB565_RED   // 红色路径
#define COLOR_POINTS      RGB565_BLUE  // 蓝色点
#define COLOR_SELECTED    RGB565_RED  // 红色选中标识
#define COLOR_MENU_TEXT   RGB565_BLUE
#define COLOR_MENU_BACKGROUND 0xbffa
#define COLOR_MENU_HIGHLIGHT RGB565_CYAN

// 移动步长（米）
#define MOVE_STEP    0.5


// 坐标转换函数
void gps_to_plane(double lat, double lon, double *x, double *y);
void plane_to_gps(double x, double y, double *lat, double *lon);
void update_all_coordinates();
void update_display_params();
void update_screen_coordinates();

// 显示函数
void draw_line_any_color(int x1, int y1, int x2, int y2, uint16 color);
void draw_point_marker(int x, int y, uint16 color);
void draw_cross_marker(int x, int y, uint16 color);
void draw_all_points_and_path();
void clear_previous_display(gps_waypoint_node_t* node);
void update_point_display(gps_waypoint_node_t* node, uint16 color);

// 点操作函数
void move_selected_point(int dx, int dy);
void select_next_point();
void select_prev_point();
void add_new_point_at_gps(gps_waypoint_node_t* node, double lat, double lon);
void add_new_point_at_screen(gps_waypoint_node_t* node, int screen_x, int screen_y);

// 节点信息展示
void show_cordinates(void);

// 下方菜单功能函数
void func_move_point(void);
void func_new_point(void);
void func_add_new_point_next(void);
void func_add_current_point_next(void);
void func_remove_point(void);
void func_save_and_quit(void);
void func_quit(void);

// 功能函数菜单
func_menu_t funcs[10];
int total_funcs;
int active_func_idx;

// 全局变量
void (*show_points_quit_callback)(void); // 退出回调函数
void (*show_points_save_callback)(void); // 保存回调函数
gps_waypoint_node_t *head;  // 第一个显示的点
gps_waypoint_node_t *selected_node; // 当前选中的节点
int selected_node_idx;
bool moving_point;              // 当前是否在移动点
// 显示变换参数
double origin_lat, origin_lon; // 原点经纬度
double scale_factor;
double offset_x, offset_y = 0.0;
double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;


// ====================== 坐标转换函数实现 ======================

void gps_to_plane(double lat, double lon, double *x, double *y) {
    double distance = get_two_points_distance(origin_lat, origin_lon, lat, lon);
    double bearing = get_two_points_azimuth(origin_lat, origin_lon, lat, lon) * DEG_TO_RAD;
    
    *x = distance * sin(bearing);
    *y = distance * cos(bearing);
}

void plane_to_gps(double x, double y, double *lat, double *lon) {
    double distance = sqrt(x*x + y*y);
    double bearing = atan2(x, y);
    
    if (distance < 0.0001) {
        *lat = origin_lat;
        *lon = origin_lon;
        return;
    }
    
    double ang_dist = distance / EARTH_RADIUS;
    double lat1 = origin_lat * DEG_TO_RAD;
    double lon1 = origin_lon * DEG_TO_RAD;
    
    double lat2 = asin(sin(lat1) * cos(ang_dist) +
                      cos(lat1) * sin(ang_dist) * cos(bearing));
    
    double lon2 = lon1 + atan2(sin(bearing) * sin(ang_dist) * cos(lat1),
                              cos(ang_dist) - sin(lat1) * sin(lat2));
    
    *lat = lat2 * RAD_TO_DEG;
    *lon = lon2 * RAD_TO_DEG;
}

// 更新所有节点的坐标
void update_all_coordinates() {
    int count = total_points;
    if (count == 0) return;
    
    // 第一个点作为原点
    gps_waypoint_node_t* first = head;
    if (first) {
        origin_lat = first->waypoint->latitude;
        origin_lon = first->waypoint->longitude;
    }
    
    // 更新所有点的平面坐标
    gps_waypoint_node_t* current = head;
    while (current) {
        gps_to_plane(current->waypoint->latitude, 
                     current->waypoint->longitude,
                     &current->plane_x, 
                     &current->plane_y);
        current = current->next;
    }
    
    update_display_params();
    update_screen_coordinates();
}

// 更新显示参数（缩放和偏移）
void update_display_params() {
    int count = total_points;
    if (count < 1) return;
    
    // 找到点集的边界
    gps_waypoint_node_t* current = head;
    min_x = current->plane_x;
    max_x = current->plane_x;
    min_y = current->plane_y;
    max_y = current->plane_y;
    
    while (current) {
        if (current->plane_x < min_x) min_x = current->plane_x;
        if (current->plane_x > max_x) max_x = current->plane_x;
        if (current->plane_y < min_y) min_y = current->plane_y;
        if (current->plane_y > max_y) max_y = current->plane_y;
        current = current->next;
    }
    
    // 计算点集的宽度和高度
    double data_width = max_x - min_x;
    double data_height = max_y - min_y;
    
    // 添加20%边距
    if (data_width == 0) data_width = 10.0;
    if (data_height == 0) data_height = 10.0;
    
    data_width *= 1.2;
    data_height *= 1.2;
    
    // 计算缩放比例
    double scale_x = (SCREEN_WIDTH * 0.9) / data_width;
    double scale_y = (SCREEN_HEIGHT * 0.9) / data_height;
    scale_factor = (scale_x < scale_y) ? scale_x : scale_y;
    
    // 计算偏移量（居中显示）
    offset_x = (SCREEN_WIDTH - (max_x - min_x) * scale_factor) / 2 - min_x * scale_factor;
    offset_y = (SCREEN_HEIGHT - (max_y - min_y) * scale_factor) / 2 - min_y * scale_factor;
}

// 更新所有节点的屏幕坐标
void update_screen_coordinates() {
    gps_waypoint_node_t* current = head;
    
    while (current) {
        current->screen_x = (int)(current->plane_x * scale_factor + offset_x);
        current->screen_y = (int)(SCREEN_HEIGHT - (current->plane_y * scale_factor + offset_y));
        
        // 边界检查
        if (current->screen_x < 0) current->screen_x = 0;
        if (current->screen_x >= SCREEN_WIDTH) current->screen_x = SCREEN_WIDTH - 1;
        if (current->screen_y < 0) current->screen_y = 0;
        if (current->screen_y >= SCREEN_HEIGHT) current->screen_y = SCREEN_HEIGHT - 1;
        
        current = current->next;
    }
}

// ====================== 显示函数实现 ======================
// 空行
char empty_line[31] = "                             ";

// 统一绘制线函数
void draw_line_any_color(int x1, int y1, int x2, int y2, uint16 color) {
    ips200_draw_line(x1, y1, x2, y2, color);
    // printf("Line: (%d,%d)->(%d,%d) color:0x%04X\n", x1, y1, x2, y2, color);
}

void clear_line(int line)
{
    ips200_show_string(0, line * 16, empty_line);
}

// 绘制点标记
void draw_point_marker(int x, int y, uint16 color) {
    // 绘制3x3方块
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            ips200_draw_point(x + dx, y + dy, color);
        }
    }
}

// 绘制十字标记
void draw_cross_marker(int x, int y, uint16 color) {
    int size = 10;
    
    // 水平线
    draw_line_any_color(x - size, y, x - 3, y, color);
    draw_line_any_color(x + 3, y, x + size, y, color);
    
    // 垂直线
    draw_line_any_color(x, y - size, x, y - 3, color);
    draw_line_any_color(x, y + 3, x, y + size, color);
    
    // 中心交叉
    // draw_line_any_color(x-1, y-1, x+1, y+1, color);
    // draw_line_any_color(x-1, y+1, x+1, y-1, color);
}

// 绘制所有点和路径
void draw_all_points_and_path() {
    // 默认选择第一个点
    selected_node = head;
    selected_node_idx = 1;
    // 绘制连接线
    gps_waypoint_node_t* current = head;
    while (current && current->next) {
        draw_line_any_color(current->screen_x, current->screen_y,
                           current->next->screen_x, current->next->screen_y,
                           COLOR_PATH);
        current = current->next;
    }
    
    // 绘制所有点
    current = head;
    while (current) {
        if (current == selected_node) {
            draw_cross_marker(current->screen_x, current->screen_y, COLOR_SELECTED);
            draw_point_marker(current->screen_x, current->screen_y, COLOR_POINTS);
        } else {
            draw_point_marker(current->screen_x, current->screen_y, COLOR_POINTS);
        }
        current = current->next;
    }
}

// 清除节点的旧显示
void clear_previous_display(gps_waypoint_node_t* node) {
    if (!node || !node->waypoint) return;
    
    // 清除点标记
    draw_point_marker(node->screen_x, node->screen_y, COLOR_BACKGROUND);
    
    // 清除十字标记
    draw_cross_marker(node->screen_x, node->screen_y, COLOR_BACKGROUND);
    
    // 清除连接线（前一条）
    if (node->prev && node->prev->waypoint) {
        draw_line_any_color(node->prev->screen_x, node->prev->screen_y,
                           node->screen_x, node->screen_y, COLOR_BACKGROUND);
    }
    
    // 清除连接线（后一条）
    if (node->next) {
        draw_line_any_color(node->screen_x, node->screen_y,
                           node->next->screen_x, node->next->screen_y, COLOR_BACKGROUND);
    }
}

// 更新节点显示（可以指定颜色）
void update_point_display(gps_waypoint_node_t* node, uint16 color) {
    if (!node || !node->waypoint) return;
    
    // 绘制点标记
    draw_point_marker(node->screen_x, node->screen_y, color);
    
    // 绘制连接线（前一条）
    if (node->prev && node->prev->waypoint) {
        draw_line_any_color(node->prev->screen_x, node->prev->screen_y,
                           node->screen_x, node->screen_y, COLOR_PATH);
    }
    
    // 绘制连接线（后一条）
    if (node->next) {
        draw_line_any_color(node->screen_x, node->screen_y,
                           node->next->screen_x, node->next->screen_y, COLOR_PATH);
    }
    
    // 如果是选中节点，绘制十字标记
    if (node == selected_node) {
        draw_cross_marker(node->screen_x, node->screen_y, COLOR_SELECTED);
        show_cordinates();
    }
}

// ====================== 点操作函数实现 ======================

// 移动选中的点
void move_selected_point(int dx, int dy) {
    if (!selected_node) return;
    
    // 清除旧显示
    clear_previous_display(selected_node);
    if (selected_node->prev) clear_previous_display(selected_node->prev);
    if (selected_node->next) clear_previous_display(selected_node->next);
    
    // 更新平面坐标
    selected_node->plane_x += dx * MOVE_STEP;
    selected_node->plane_y += dy * MOVE_STEP;
    
    // 更新GPS坐标
    plane_to_gps(selected_node->plane_x, 
                selected_node->plane_y,
                &selected_node->waypoint->latitude,
                &selected_node->waypoint->longitude);
    
    // 更新屏幕坐标
    selected_node->screen_x = (int)(selected_node->plane_x * scale_factor + offset_x);
    selected_node->screen_y = (int)(SCREEN_HEIGHT - (selected_node->plane_y * scale_factor + offset_y));
    
    // 边界检查
    if (selected_node->screen_x < 0) selected_node->screen_x = 0;
    if (selected_node->screen_x >= SCREEN_WIDTH) selected_node->screen_x = SCREEN_WIDTH - 1;
    if (selected_node->screen_y < 0) selected_node->screen_y = 0;
    if (selected_node->screen_y >= SCREEN_HEIGHT) selected_node->screen_y = SCREEN_HEIGHT - 1;
    
    // 重新绘制
    if (selected_node->prev) update_point_display(selected_node->prev, COLOR_POINTS);
    update_point_display(selected_node, COLOR_SELECTED);
    if (selected_node->next) update_point_display(selected_node->next, COLOR_POINTS);
}

// 选择下一个点
void select_next_point() {
    if (!selected_node->next) return;
    
    // 清除十字标记
    // draw_cross_marker(selected_node->screen_x, selected_node->screen_y, COLOR_BACKGROUND);
    clear_previous_display(selected_node);
    
    // 选择下一个点
    selected_node = selected_node->next;
    selected_node_idx++;
    
    // 绘制新选中点
    update_point_display(selected_node->prev, COLOR_POINTS);
    update_point_display(selected_node, COLOR_POINTS);
}

// 选择上一个点
void select_prev_point() {
    if (!selected_node->prev) return;
    if (!selected_node->prev->waypoint) return;
    
    // 清除十字标记
    // draw_cross_marker(selected_node->screen_x, selected_node->screen_y, COLOR_BACKGROUND);
    clear_previous_display(selected_node);
    
    selected_node = selected_node->prev;
    selected_node_idx--;
    
    update_point_display(selected_node->next, COLOR_POINTS);
    update_point_display(selected_node, COLOR_POINTS);
}

// 添加新点（通过经纬度）
void add_new_point_at_gps(gps_waypoint_node_t* node, double lat, double lon) {
    gps_add_point(node, lat, lon);
}

// 在屏幕位置添加新点
void add_new_point_at_screen(gps_waypoint_node_t* node, int screen_x, int screen_y) {
    // 将屏幕坐标转换为平面坐标
    double plane_x = (screen_x - offset_x) / scale_factor;
    double plane_y = (SCREEN_HEIGHT - screen_y - offset_y) / scale_factor;
    
    // 将平面坐标转换为GPS坐标
    double lat, lon;
    plane_to_gps(plane_x, plane_y, &lat, &lon);
    
    // 创建新点
    add_new_point_at_gps(node, lat, lon);
}

// ====================== 节点信息展示 =========================
void show_cordinates(void)
{
    char text[30];
    ips200_set_color(COLOR_MENU_TEXT, COLOR_MENU_BACKGROUND);
    sprintf(text, "Point%02d ", selected_node_idx);
    ips200_show_string(0, 16 * 16, text);
    char str_lat[30], str_lon[30];
    if(selected_node->waypoint)
    {
        sprintf(str_lat, "Lat. %.6f", selected_node->waypoint->latitude);
        sprintf(str_lon, "Lon. %.6f", selected_node->waypoint->longitude);
    }
    else
    {
        sprintf(str_lat, "Lat. N/A");
        sprintf(str_lon, "Lon. N/A");
    }
    ips200_show_string(16, 17 * 16, str_lat);
    ips200_show_string(16, 18 * 16, str_lon);
}

// ====================== 底部菜单实现 =========================
void new_func(char* name, void(*func)(void))
{
    funcs[total_funcs].name = name;
    funcs[total_funcs].func = func;
    total_funcs++;
}

void funcs_init(void)
{
    new_func("Move Point", func_move_point);
    new_func("New Point", func_new_point);
    new_func("Add New Point Next", func_add_new_point_next);
    new_func("Add Current Point", func_add_current_point_next);
    new_func("Remove Point", func_remove_point);
    new_func("Save and Quit", func_save_and_quit);
    new_func("Quit Without Save", func_quit);
}

void show_func(void)
{
    ips200_set_color(COLOR_MENU_TEXT, COLOR_MENU_BACKGROUND);
    char text[35];
    sprintf(text, "%-30s", funcs[active_func_idx].name);
    ips200_show_string(0, 19*16, text);
}

void show_next_func(void)
{
    active_func_idx++;
    if(active_func_idx >= total_funcs)
        active_func_idx = 0;
    show_func();
}

void show_prev_func(void)
{
    active_func_idx--;
    if(active_func_idx < 0)
        active_func_idx = total_funcs - 1;
    show_func();
}

// ====================== 菜单功能函数实现 ======================
void func_move_point(void)
{
    moving_point = 1;
    ips200_set_color(COLOR_MENU_TEXT, COLOR_MENU_HIGHLIGHT);
    ips200_show_string(0, 304, "Moving Point...");
}

void func_new_point(void)
{
    add_new_point_at_screen(last_node, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
    selected_node = last_node;
    gps_show_points_main();
}

void func_add_new_point_next(void)
{
    add_new_point_at_screen(selected_node, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
    selected_node = selected_node->next;
    gps_show_points_main();
}

void func_add_current_point_next(void)
{
    add_new_point_at_gps(selected_node, gnss.latitude, gnss.longitude);
    gps_show_points_main();
}

void func_remove_point(void)
{
    gps_waypoint_node_t* temp = selected_node;
    gps_remove_point(temp);
    gps_show_points_main();
}

void func_save_and_quit(void)
{
    show_points_save_callback();
    func_quit();
}

void func_quit(void)
{
    show_points_quit_callback();
}


// ====================== 按键功能函数实现 ======================
void gps_show_points_down_pressed(void)
{
    if(!moving_point) 
        select_next_point();
    else
        move_selected_point(0, -1);
}

void gps_show_points_up_pressed(void)
{
    if(!moving_point)
        select_prev_point();
    else
        move_selected_point(0, 1);
}

void gps_show_points_left_pressed(void)
{
    if(moving_point)
        move_selected_point(-1, 0);
    else
        show_prev_func();

}

void gps_show_points_right_pressed(void)
{
    if(moving_point)
        move_selected_point(1, 0);
    else
        show_next_func();
}

void gps_show_points_ok_pressed(void)
{
    if(moving_point)
    {
        moving_point = 0;
        show_func();
    }
        
    else
    {
        funcs[active_func_idx].func();
    }
}

void gps_show_points_init(void (*quit_callback)(void), void (*save_callback)(void))
{
    head = root.next;
    selected_node = head;
    selected_node_idx = 1;
    total_funcs = 0;
    active_func_idx = 0;
    funcs_init();
    
    show_points_quit_callback = quit_callback;
    show_points_save_callback = save_callback;
}


// ====================== 主函数实现 ======================
void gps_show_points_main(void)
{
    // 设置第一个显示的点
    head = root.next;
    
    // 计算屏幕坐标和显示参数
    update_all_coordinates();
    
    ips200_set_color(COLOR_MENU_TEXT, COLOR_BACKGROUND);
    ips200_clear();
    
    draw_all_points_and_path();
    
    show_func();
    show_cordinates();
}