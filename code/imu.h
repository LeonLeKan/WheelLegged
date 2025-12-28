#ifndef __IMU_H__
#define __IMU_H__

#include "zf_common_headfile.h"
#include "mahony_filter.h"
#include "ukf_filter.h"

#define IMU_UPDATE_INTERVAL_MS 2  // IMU更新间隔，单位毫秒


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
        float speed;    // 速度（m/s）
    } body;
    
    // 校准参数
    struct {
        float gyro_bias[3];      // 陀螺仪零偏
        uint32_t sample_count;   // 已采集样本数
        uint32_t target_samples; // 目标样本数
        bool is_calibrated;      // 校准完成标志
    } calib;
    
    // 输出
    float euler[3];     // 欧拉角 [roll, pitch, yaw] (度)
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