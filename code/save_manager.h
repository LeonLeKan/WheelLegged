#ifndef __SAVE_MANAGER_H__
#define __SAVE_MANAGER_H__

#include "waypoint.h"

#define INS_FLASH_START_PAGE 2
#define GPS_FLASH_START_PAGE 10

typedef struct
{
    int total_saves;
    int active_save_idx;
    waypoint_list_t* active_save;
    int showing_points;
    void (*exit_callback)(void);
}save_manager_t;

extern save_manager_t save_manager_instance;


void save_manager_init(void (*exit_callback)(void));

/// @brief 从Flash加载存档
/// @param save_idx 要加载的存档编号 0~4
/// @return true-加载成功 false-加载失败
bool save_manager_load(int save_idx);
waypoint_list_t* save_manager_get_active_save(void);

#endif
