#ifndef __SCREEN_INTERFACE_H__
#define __SCREEN_INTERFACE_H__

#include "zf_common_headfile.h"

void screen_init(void);
void screen_draw_point(int x, int y, uint16 color);
void screen_draw_line(int x1, int y1, int x2, int y2, uint16 color);
void screen_printf(uint16_t x, uint16_t y, const char* format, ...);

#endif