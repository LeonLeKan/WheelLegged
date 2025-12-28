#ifndef __SCREEN_INTERFACE_H__
#define __SCREEN_INTERFACE_H__

#include "zf_common_headfile.h"

void screen_init(void);
void screen_draw_point(int x, int y, uint16 color);
void screen_draw_line(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color);
void screen_printf(uint16_t x, uint16_t y, const char* format, ...);

#endif