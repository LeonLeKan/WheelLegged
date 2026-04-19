#ifndef __IMU_H__
#define __IMU_H__

#include "zf_common_headfile.h"
#include "mahony_filter.h"
#include "config.h"

// imu状态枚举
typedef enum {
    IMU_STATE_UNINIT = 0,
    IMU_STATE_INITIAL_ESTIMATION,
    IMU_STATE_AUTO_GYRO_CALIBRATING,
    IMU_STATE_NORMAL,
    IMU_STATE_ROTATING,
    IMU_STATE_ERROR
} imu_state_t;

typedef struct {
    double rotation_thre;       // 判断旋转状态的yaw角速度阈值 (deg/s)
    double yaw_compensate_cw;   // 顺时针旋转时给yaw的补偿 (deg)
    double yaw_compensate_ccw;  // 逆时针旋转时给yaw的补偿 (deg)
} RotationCompensationParam;    // 旋转状态yaw角速度补偿参数


typedef struct {
    // 原始传感器数据（传感器坐标系）
    struct {
        double acc[3];   // 加速度计 (g)
        double gyro[3];  // 陀螺仪 (deg/s)
    } raw;
    
    // 校准后数据（机体FLU坐标系）
    struct {
        double acc[3];   // 转换后的加速度 (g)
        double gyro[3];  // 减去零偏后的角速度 (deg/s)
    } body;

    struct {
        double v_left, v_right;  // 左右轮线速度 (m/s)
        double speed;            // 前进速度 (m/s)
        double yaw_rate;         // 编码器计算的yaw角速度 (deg/s)
    } encoder;

    // 校准参数
    struct {
        double gyro_bias[3];      // 陀螺仪零偏
        uint32_t sample_count;   // 已采集样本数
        uint32_t target_samples; // 目标样本数
        bool is_calibrated;      // 校准完成标志
    } calib;

    struct {
        double acc_sum[3];        // 加速度校准期间的累积和，用于计算初始姿态
        uint32_t sample_count;   // 已采集样本数
        uint32_t target_samples; // 目标样本数
        bool is_estimated;       // 初始估计完成标志
    } initial_estimation; // 初始姿态估计相关数据

    struct {
        int enter_cnt, exit_cnt;
        RotationCompensationParam param;
    } rot_comp;

    // 输出
    struct {
        double euler[3];     // 欧拉角 [roll, pitch, yaw] (deg)
        double gyro[3];      // 角速度 (deg/s)
        double position_x;   // x位置坐标 (m)
        double position_y;   // y位置坐标 (m)
        double speed;        // 速度 (m/s)
    } output;

    imu_state_t state;  // 当前状态
    
    // 算法实例
    MahonyFilter mahony_filter;
} imu_t;


extern imu_t imu;


void imu_init(MahonyFilterParam mahony_param, RotationCompensationParam rot_comp_param);
void imu_reset_mahony_filter(MahonyFilterParam param);
void imu_start_initial_estimation(uint32_t target_samples);
void imu_set_gyro_bias(double bias_x, double bias_y, double bias_z);
void imu_start_auto_gyro_calibration(uint32_t target_samples);
void imu_update(void);  // 在定时中断中调用
void imu_reset_yaw(void);


#endif