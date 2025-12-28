#include "screen_interface.h"

#define IPS200_TYPE     (IPS200_TYPE_SPI)

void screen_init(void)
{
    ips200_init(IPS200_TYPE);
}

void screen_draw_point(int x, int y, uint16 color)
{
    if(x < 240 && x >= 0 && y < 240 && y >= 0)
        ips200_draw_point(x, y, color);
}

void screen_draw_line(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color)
{
    int16 x_dir = (x_start < x_end ? 1 : -1);
    int16 y_dir = (y_start < y_end ? 1 : -1);
    float temp_rate = 0;
    float temp_b = 0;

    do
    {
        if(x_start != x_end)
        {
            temp_rate = (float)(y_start - y_end) / (float)(x_start - x_end);
            temp_b = (float)y_start - (float)x_start * temp_rate;
        }
        else
        {
            while(y_start != y_end)
            {
                screen_draw_point(x_start, y_start, color);
                y_start += y_dir;
            }
            screen_draw_point(x_start, y_start, color);
            break;
        }
        if(func_abs(y_start - y_end) > func_abs(x_start - x_end))
        {
            while(y_start != y_end)
            {
                screen_draw_point(x_start, y_start, color);
                y_start += y_dir;
                x_start = (int16)(((float)y_start - temp_b) / temp_rate);
            }
            screen_draw_point(x_start, y_start, color);
        }
        else
        {
            while(x_start != x_end)
            {
                screen_draw_point(x_start, y_start, color);
                x_start += x_dir;
                y_start = (int16)((float)x_start * temp_rate + temp_b);
            }
            screen_draw_point(x_start, y_start, color);
        }
    }while(0);
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
