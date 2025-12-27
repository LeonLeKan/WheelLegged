#ifndef __GPS_SAVE_MANAGER_H__
#define __GPS_SAVE_MANAGER_H__

#include "gps_core.h"

#define GPS_FLASH_START_PAGE 2

void gps_save_manager_init(void);
void gps_show_save_manager(void);

void gps_save_manager_up_pressed(void);
void gps_save_manager_down_pressed(void);
void gps_save_manager_ok_pressed(void);
void gps_save_manager_left_pressed(void);
void gps_save_manager_right_pressed(void);
#endif
