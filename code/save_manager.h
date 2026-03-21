#ifndef __SAVE_MANAGER_H__
#define __SAVE_MANAGER_H__

#include "waypoint.h"

#define AHRS_FLASH_START_PAGE 2
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
bool save_manager_load(void);
waypoint_list_t* save_manager_get_active_save(void);

#endif
