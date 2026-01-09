#include "ukf_filter.h"
#include "imu.h"
#include "small_driver_uart_control.h"

static void set_ukf_param(ukf_filter_param_t* param);


bool ukf_filter_init(ukf_filter_t* filter, ukf_filter_param_t* param)
{
    // 设置UKF参数
    set_ukf_param(param);
    // 初始化UKF实例
    return ukf_init(&filter->ukfIO, &UkfMatrixCfg1);
}


void ukf_filter_update(ukf_filter_t* filter, float measurement[3])
{
    double *const py = filter->ukfIO.input.y.val;
    // 设置测量值
    py[0] = measurement[0]; // 速度观测值
    py[1] = measurement[1]; // 航向角观测值
    py[2] = measurement[2]; // 角速度观测值
    // 执行UKF步骤
    ukf_step(&filter->ukfIO);
    // 更新输出估计值
    filter->output.position_x = (float)filter->ukfIO.update.x.val[0];
    filter->output.position_y = (float)filter->ukfIO.update.x.val[1];
    filter->output.speed = (float)filter->ukfIO.update.x.val[2];
    filter->output.yaw = (float)filter->ukfIO.update.x.val[3] * RAD_TO_DEG;
    filter->output.yaw_rate = (float)filter->ukfIO.update.x.val[4] * RAD_TO_DEG;
}


// == 内部函数实现 ==

static void set_ukf_param(ukf_filter_param_t* param)
{
    // 设置采样时间间隔
    UkfMatrixCfg1.dT = param->dt;
    // 设置过程噪声协方差矩阵Q
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            if (i == j)
            {
                UkfMatrixCfg1.Qxx_process_noise_cov.val[i * 5 + j] = param->q[i];
            }
            else
            {
                UkfMatrixCfg1.Qxx_process_noise_cov.val[i * 5 + j] = 0.0;
            }
        }
    }
    // 设置测量噪声协方差矩阵R
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (i == j)
            {
                UkfMatrixCfg1.Ryy0_init_out_covariance.val[i * 3 + j] = param->r[i];
            }
            else
            {
                UkfMatrixCfg1.Ryy0_init_out_covariance.val[i * 3 + j] = 0.0;
            }
        }
    }
}