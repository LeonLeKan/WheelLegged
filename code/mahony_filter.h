// mahony_filter.h
#ifndef __MAHONY_FILTER_H__
#define __MAHONY_FILTER_H__


#include "zf_common_headfile.h"

#define RAD_TO_DEG 180.0 / PI
#define DEG_TO_RAD PI / 180.0

typedef struct {
    float Kp, Ki, dt;
} MahonyFilterParam;

typedef struct {
    float q[4];           // 四元数 [q0, q1, q2, q3]
    float integralFB[3];  // 误差积分项
    MahonyFilterParam param;
} MahonyFilter;


void Mahony_Init(MahonyFilter* filter, MahonyFilterParam* param);
void Mahony_Update(MahonyFilter* filter, float gyro[3], float acc[3]);
void Mahony_GetEuler(MahonyFilter* filter, float* roll, float* pitch, float* yaw);


#endif