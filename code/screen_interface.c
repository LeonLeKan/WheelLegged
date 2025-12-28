#include "screen_interface.h"

#define IPS200_TYPE     (IPS200_TYPE_SPI)

void screen_init(void)
{
    ips200_init(IPS200_TYPE);
}

void screen_draw_point(int x, int y, uint16 color)
{
    ips200_draw_point(x, y, color);
}

// 统一绘制线函数
void screen_draw_line(int x1, int y1, int x2, int y2, uint16 color)
{
    ips200_draw_line(x1, y1, x2, y2, color);
}

void screen_printf(uint16_t x, uint16_t y, const char* format, ...)
{
    char buffer[40]; // 根据需要调整缓冲区大小
    va_list args;
    // 初始化可变参数列表
    va_start(args, format);
    // 使用 vsnprintf 安全地格式化字符串
    vsnprintf(buffer, sizeof(buffer), format, args);
    // 结束可变参数列表
    va_end(args);
    // 调用显示函数
    ips200_show_string(x, y, buffer);
}
