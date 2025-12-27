#include "gps_core.h"

gps_waypoint_node_t root; // 特殊根节点，id为0，无具体数据

gps_waypoint_node_t* last_node; // 指向最后一个节点

int total_points;  // 现在一共有多少个点


void gps_init(void)
{
    // 初始化GPS
    gnss_init(TAU1201);
    // GPS 解析周期中断 100ms
    pit_ms_init(PIT_CH10, 100);
    
    root.prev = NULL;
    root.next = NULL;
    root.waypoint = NULL;
    
    last_node = &root;
}

void gps_add_point(gps_waypoint_node_t *pNode, double latitude, double longitude)
{
    gps_waypoint_node_t *newNode = (gps_waypoint_node_t*)malloc(sizeof(gps_waypoint_node_t));
    
    newNode->waypoint = (gps_waypoint_t*)malloc(sizeof(gps_waypoint_t));
    newNode->prev = pNode;
    newNode->next = pNode->next;
    
    // 存入当前点信息
    newNode->waypoint->latitude = latitude;
    newNode->waypoint->longitude = longitude;
    
    if(newNode->next == NULL)
    {
        // 新加入的点为最后一点，修改last_node
        last_node = newNode;
    }
    else newNode->next->prev = newNode;
    pNode->next = newNode;
    total_points++;
}

void gps_add_current_point(gps_waypoint_node_t *pNode)
{
    // 添加当前位置
    gps_add_point(pNode, gnss.latitude, gnss.longitude);
}

void gps_remove_point(gps_waypoint_node_t *pNode)
{
    pNode->prev->next = pNode->next;
    pNode->next->prev = pNode->prev;
    if(pNode->next == NULL)
    {
        // 移除的点为最后一点，修改last_node
        last_node = pNode->prev;
    }
    
    free(pNode->waypoint);
    free(pNode);
    total_points--;
}

void gps_clear(void)
{
    gps_waypoint_node_t* now_node = &root;
    if(!now_node->next) return;
    now_node = now_node->next;
    while(now_node)
    {
        gps_waypoint_node_t* temp = now_node;
        now_node = now_node->next;
        gps_remove_point(temp);
    }
    last_node = &root;
}


union double_converter
{
    double d;
    uint32_t buf[2];
};


void gps_save_points(uint32 flash_page)
{
    flash_buffer_clear();
    gps_waypoint_node_t *now_node = root.next;
    
    int index = 1; // flash_union_buffer[0] : 一共有几个点
    union double_converter converter;
    
    while(now_node)
    {
        // double 占用两个uint32
        converter.d = now_node->waypoint->latitude;
        flash_union_buffer[index++].uint32_type = converter.buf[0];
        flash_union_buffer[index++].uint32_type = converter.buf[1];
        
        converter.d = now_node->waypoint->longitude;
        flash_union_buffer[index++].uint32_type = converter.buf[0];
        flash_union_buffer[index++].uint32_type = converter.buf[1];
        now_node = now_node->next;
    }
    flash_union_buffer[0].int32_type = index;
    
    flash_erase_page(0, flash_page);
    flash_write_page_from_buffer(0, flash_page, index);
    
}


bool gps_load_points(uint32 flash_page)
{
    gps_clear();
    flash_buffer_clear();
    flash_read_page_to_buffer(0, flash_page, FLASH_PAGE_LENGTH);
    // 先读取一共有几个点
    int points_cnt = (flash_union_buffer[0].int32_type - 1) / 4;
    if(points_cnt <= 0 || total_points > 200) // 点数非法，失败
        return false;
    // 设置总点数
    total_points = points_cnt;
    double t_latitude, t_longitude;
    int index = 1;
    union double_converter converter;
    for(int i = 1; i <= points_cnt; i++)
    {
        converter.buf[0] = flash_union_buffer[index++].uint32_type;
        converter.buf[1] = flash_union_buffer[index++].uint32_type;
        t_latitude = converter.d;
        
        converter.buf[0] = flash_union_buffer[index++].uint32_type;
        converter.buf[1] = flash_union_buffer[index++].uint32_type;
        t_longitude = converter.d;
        
        gps_add_point(last_node, t_latitude, t_longitude);
    }
    return true;
}
