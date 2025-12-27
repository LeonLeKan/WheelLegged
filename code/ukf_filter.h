#ifndef __UKF_FILTER_H__
#define __UKF_FILTER_H__

#include "zf_common_headfile.h"
#include "ukfLib.h"
#include "ukfCfg1.h"

typedef struct
{
    double dt;   // 采样时间间隔
    double q[5]; // 过程噪声协方差对角线元素
    double r[3]; // 测量噪声协方差对角线元素
}ukf_filter_param_t;

typedef struct
{
    UKF_t ukfIO;
    ukf_filter_param_t params;
    struct
    {
        float position_x;   // x位置估计值
        float position_y;   // y位置估计值
        float speed;        // 速度估计值
        float yaw;          // 航向角估计值(deg)
        float yaw_rate;     // 角速度估计值(deg/s)
    }output;
}ukf_filter_t;


/// @brief ukf滤波器初始化函数
/// @param filter ukf滤波器实例
/// @param param ukf滤波器参数
/// @return 0-成功，1-失败
bool ukf_filter_init(ukf_filter_t* filter, ukf_filter_param_t* param);

/// @brief ukf滤波器更新函数（中断中调用）
/// @param filter ukf滤波器实例
/// @param measurement 测量值数组 [速度, 航向角, 角速度]
void ukf_filter_update(ukf_filter_t* filter, float measurement[3]);

#endif