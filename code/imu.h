#ifndef __IMU_H__
#define __IMU_H__

#include "zf_common_headfile.h"
#include "mahony_filter.h"
#include "ukf_filter.h"

#define IMU_UPDATE_INTERVAL_MS 2  // IMU更新间隔，单位毫秒

// 车辆尺寸参数
#define WHEEL_D     0.065   // 轮胎直径
#define WHEELBASE   0.185   // 轮距


// imu状态枚举
typedef enum {
    IMU_STATE_UNINIT = 0,
    IMU_STATE_CALIBRATING,
    IMU_STATE_RUNNING,
    IMU_STATE_ERROR
} imu_state_t;

typedef struct {
    // 原始传感器数据（传感器坐标系）
    struct {
        float acc[3];   // 加速度计 (g)
        float gyro[3];  // 陀螺仪 (deg/s)
    } raw;
    
    // 校准后数据（机体FLU坐标系）
    struct {
        float acc[3];   // 转换后的加速度 (g)
        float gyro[3];  // 减去零偏后的角速度 (deg/s)
    } body;

    struct {
        float v_left, v_right;  // 左右轮线速度 (m/s)
        float speed;            // 前进速度 (m/s)
        float yaw_rate;         // 编码器计算的yaw角速度 (deg/s)
    } encoder;

    // 校准参数
    struct {
        float gyro_bias[3];      // 陀螺仪零偏
        uint32_t sample_count;   // 已采集样本数
        uint32_t target_samples; // 目标样本数
        bool is_calibrated;      // 校准完成标志
    } calib;

    // 输出
    struct {
        float euler[3];     // 欧拉角 [roll, pitch, yaw] (deg)
        float gyro[3];      // 角速度 (deg/s)
        float position_x;   // x位置坐标 (m)
        float position_y;   // y位置坐标 (m)
        float speed;        // 速度 (m/s)
    } output;

    imu_state_t state;  // 当前状态
    
    // 算法实例
    MahonyFilter mahony_filter;
    ukf_filter_t ukf_filter;
} imu_t;


extern imu_t imu;


void imu_init(MahonyFilterParam* param, ukf_filter_param_t* ukf_param);
void imu_reset_mahony_filter(MahonyFilterParam* param);
void imu_reset_ukf_filter(ukf_filter_param_t* param);
void imu_start_calibration(uint32_t target_samples);
bool imu_is_calibrated(void);
void imu_update(void);  // 在定时中断中调用


#endif