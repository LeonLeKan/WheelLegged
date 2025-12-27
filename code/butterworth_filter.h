#ifndef __BUTTERWORTH_FILTER_H__
#define __BUTTERWORTH_FILTER_H__

#include "zf_common_headfile.h"

typedef struct {
    float a[3];  // 分母系数 [a0, a1, a2]
    float b[3];  // 分子系数 [b0, b1, b2]
    float x[3];  // 输入历史 [x(n), x(n-1), x(n-2)]
    float y[3];  // 输出历史 [y(n), y(n-1), y(n-2)]
} BiquadFilter;

typedef struct {
    BiquadFilter stage1;
    BiquadFilter stage2;
    int order;  // 滤波器阶数 (2或4)
} ButterworthFilter;

// 滤波器初始化函数
void butterworth_lowpass_init(ButterworthFilter* filter, float cutoff_freq, float sample_freq, int order);

// 滤波函数
float butterworth_filter(ButterworthFilter* filter, float input);

// 滤波器重置
void butterworth_reset(ButterworthFilter* filter);

#endif