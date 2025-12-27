#ifndef __GPS_CORE_H__
#define __GPS_CORE_H__

#include "zf_common_headfile.h"


typedef struct {
    double latitude; // 纬度
    double longitude; // 经度
}gps_waypoint_t;

// GPS点链表节点
struct gps_waypoint_node {
    gps_waypoint_t* waypoint;      // GPS坐标
    struct gps_waypoint_node *next, *prev; // 前后指针
    
    // 显示相关属性
    double plane_x, plane_y;       // 平面坐标（米）
    int screen_x, screen_y;        // 屏幕坐标
};

typedef struct gps_waypoint_node gps_waypoint_node_t;

// 现在一共有几个点
extern int total_points;
// 根节点
extern gps_waypoint_node_t root;
// 最后一个节点
extern gps_waypoint_node_t* last_node;


void gps_init(void);

void gps_add_point(gps_waypoint_node_t *pNode, double latitude, double longitude);

void gps_add_current_point(gps_waypoint_node_t *pNode);

void gps_remove_point(gps_waypoint_node_t *pNode);

// 存储所有GPS点
void gps_save_points(uint32 flash_page);

// 加载所有GPS点
bool gps_load_points(uint32 flash_page);

void gps_clear(void);

#endif