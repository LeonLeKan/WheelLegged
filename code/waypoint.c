#include "waypoint.h"

waypoint_list_t* waypoint_list_create(waypoint_t waypoint, waypoint_screen_param_t screen_param)
{
    waypoint_list_t* list = (waypoint_list_t*)malloc(sizeof(waypoint_list_t));
    list->head = (waypoint_node_t*)malloc(sizeof(waypoint_node_t));
    list->head->prev = NULL;
    list->head->next = NULL;
    list->head->waypoint = (waypoint_t*)malloc(sizeof(waypoint_t));
    list->head->screen_param = (waypoint_screen_param_t*)malloc(sizeof(waypoint_screen_param_t));
    
    *(list->head->waypoint) = waypoint;
    *(list->head->screen_param) = screen_param;

    list->size = 1;
    list->tail = list->head;
    return list;
}

waypoint_node_t* waypoint_list_add_next(waypoint_list_t* list, waypoint_node_t* pNode,
                                        waypoint_t waypoint, waypoint_screen_param_t screen_param)
{
    waypoint_node_t *newNode = (waypoint_node_t*)malloc(sizeof(waypoint_node_t));
    
    newNode->waypoint = (waypoint_t*)malloc(sizeof(waypoint_t));
    newNode->screen_param = (waypoint_screen_param_t*)malloc(sizeof(waypoint_screen_param_t));
    *(newNode->waypoint) = waypoint;
    *(newNode->screen_param) = screen_param;
    newNode->prev = pNode;
    newNode->next = pNode->next;


    if(newNode->next == NULL)
    {
        // 新加入的点为最后一点，修改last_node
        list->tail = newNode;
    }
    else newNode->next->prev = newNode;

    pNode->next = newNode;
    list->size++;

    return newNode;
}

void waypoint_list_remove(waypoint_list_t *list, waypoint_node_t *pNode)
{
    if(pNode->prev) pNode->prev->next = pNode->next;
    if(pNode->next) pNode->next->prev = pNode->prev;
    if(pNode->next == NULL)
    {
        // 移除的点为最后一点，修改tail
        list->tail = pNode->prev;
    }
    
    free(pNode->waypoint);
    free(pNode->screen_param);
    free(pNode);
    list->size--;
}

void waypoint_list_clear(waypoint_list_t *list)
{
    waypoint_node_t* now_node = list->head;
    if(!now_node->next) return;
    now_node = now_node->next;
    while(now_node)
    {
        waypoint_node_t* temp = now_node;
        now_node = now_node->next;
        waypoint_list_remove(list, temp);
    }
    list->tail = list->head;
    list->size = 1;
}

union double_converter
{
    double d;
    uint32_t buf[2];
};


void waypoint_list_save_to_flash(waypoint_list_t *list, uint32 flash_page)
{
    flash_buffer_clear();
    waypoint_node_t *now_node = list->head->next;
    
    int index = 1; // 从1开始, 0用来存一共有几个uint32的数据
    union double_converter converter;
    
    while(now_node)
    {
        // double 占用两个uint32
        converter.d = now_node->waypoint->x;
        flash_union_buffer[index++].uint32_type = converter.buf[0];
        flash_union_buffer[index++].uint32_type = converter.buf[1];
        
        converter.d = now_node->waypoint->y;
        flash_union_buffer[index++].uint32_type = converter.buf[0];
        flash_union_buffer[index++].uint32_type = converter.buf[1];
        now_node = now_node->next;
    }
    // flash_union_buffer[0] : 一共有几个uint32的数据
    flash_union_buffer[0].int32_type = index;
    
    flash_erase_page(0, flash_page);
    flash_write_page_from_buffer(0, flash_page, index);
}


bool waypoint_list_load_from_flash(waypoint_list_t *list, uint32 flash_page)
{
    waypoint_list_clear(list);
    flash_buffer_clear();
    flash_read_page_to_buffer(0, flash_page, FLASH_PAGE_LENGTH);
    // 先读取一共有几个点
    int points_cnt = (flash_union_buffer[0].int32_type - 1) / 4;
    if(points_cnt < 0 || points_cnt > 10000) // 点数非法，失败
        return false;
    double t_x, t_y;
    int index = 1;
    union double_converter converter;
    for(int i = 1; i <= points_cnt; i++)
    {
        converter.buf[0] = flash_union_buffer[index++].uint32_type;
        converter.buf[1] = flash_union_buffer[index++].uint32_type;
        t_x = converter.d;
        
        converter.buf[0] = flash_union_buffer[index++].uint32_type;
        converter.buf[1] = flash_union_buffer[index++].uint32_type;
        t_y = converter.d;
        
        waypoint_list_add_next(list, list->tail,
            (waypoint_t) { t_x, t_y }, (waypoint_screen_param_t) { t_x, t_y, 0, 0 });
        // ips200_clear();
    }
    // ips200_clear();
    return true;
}

