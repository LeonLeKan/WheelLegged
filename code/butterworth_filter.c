#include "butterworth_filter.h"


// 辅助函数：计算Butterworth滤波器系数
static void calculate_butterworth_coeffs_lowpass(BiquadFilter* filter, float cutoff_freq, float sample_freq, int order) 
{
    float *b0 = &filter->b[0], *b1 = &filter->b[1], *b2 = &filter->b[2];
    float *a0 = &filter->a[0], *a1 = &filter->a[1], *a2 = &filter->a[2];
    float omega = 2.0f * PI * cutoff_freq / sample_freq;
    float tan_omega = tanf(omega / 2.0f);
    float tan_omega_sq = tan_omega * tan_omega;
    float sqrt2 = sqrtf(2.0f);
    
    if (order == 2) {
        // 二阶Butterworth低通滤波器系数
        float denominator = tan_omega_sq + sqrt2 * tan_omega + 1.0f;
        *b0 = tan_omega_sq / denominator;
        *b1 = 2.0f * *b0;
        *b2 = *b0;
        *a0 = 1.0f;
        *a1 = 2.0f * (tan_omega_sq - 1.0f) / denominator;
        *a2 = (tan_omega_sq - sqrt2 * tan_omega + 1.0f) / denominator;
    }
}

// 初始化低通滤波器
void butterworth_lowpass_init(ButterworthFilter* filter, float cutoff_freq, float sample_freq, int order) 
{
    if (order != 2 && order != 4) {
        order = 2;
    }
    
    filter->order = order;
    
    // 第一级
    calculate_butterworth_coeffs_lowpass(&filter->stage1, cutoff_freq, sample_freq, 2);
    
    // 第二级（如果是4阶）
    if (order == 4) 
    {
        calculate_butterworth_coeffs_lowpass(&filter->stage2, cutoff_freq, sample_freq, 2);
    }
    
    // 重置滤波器状态
    butterworth_reset(filter);
}

// 单个二阶滤波器的处理函数
static float biquad_filter(BiquadFilter* filter, float input) 
{
    // 直接II型实现
    float output = filter->b[0] * input + filter->b[1] * filter->x[0] + filter->b[2] * filter->x[1]
                  - filter->a[1] * filter->y[0] - filter->a[2] * filter->y[1];
    
    output /= filter->a[0];  // 归一化
    
    // 更新历史状态
    filter->x[1] = filter->x[0];
    filter->x[0] = input;
    filter->y[1] = filter->y[0];
    filter->y[0] = output;
    
    return output;
}

// Butterworth滤波函数
float butterworth_filter(ButterworthFilter* filter, float input) 
{
    float output = biquad_filter(&filter->stage1, input);
    
    if (filter->order == 4) 
    {
        output = biquad_filter(&filter->stage2, output);
    }
    
    return output;
}

// 重置滤波器状态
void butterworth_reset(ButterworthFilter* filter) 
{
    memset(&filter->stage1.x, 0, sizeof(filter->stage1.x));
    memset(&filter->stage1.y, 0, sizeof(filter->stage1.y));
    memset(&filter->stage2.x, 0, sizeof(filter->stage2.x));
    memset(&filter->stage2.y, 0, sizeof(filter->stage2.y));
}

