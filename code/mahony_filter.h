// mahony_filter.h
#ifndef __MAHONY_FILTER_H__
#define __MAHONY_FILTER_H__


#include "zf_common_headfile.h"

#define RAD_TO_DEG 180.0 / PI
#define DEG_TO_RAD PI / 180.0

typedef struct {
    double Kp, Ki, dt;
} MahonyFilterParam;

typedef struct {
    double q[4];           // 四元数 [q0, q1, q2, q3]
    double integralFB[3];  // 误差积分项
    MahonyFilterParam param;

    double last_yaw; // 保存上一次yaw角，用于unwrap

    // 欧拉角输出
    struct {
        double euler[3];   // 欧拉角 [roll, pitch, yaw] (deg)
    } output;
} MahonyFilter;


void Mahony_Init(MahonyFilter* filter, MahonyFilterParam param);
void Mahony_Update(MahonyFilter* filter, double gyro[3], double acc[3]);
void Mahony_Reset_Yaw(MahonyFilter* filter);
void Mahony_Stop(MahonyFilter* filter);


#endif