// mahony_filter.c
#include "mahony_filter.h"

// 初始化
void Mahony_Init(MahonyFilter* filter, MahonyFilterParam* param) {
    filter->q[0] = 1.0f; filter->q[1] = 0.0f; 
    filter->q[2] = 0.0f; filter->q[3] = 0.0f;
    filter->integralFB[0] = filter->integralFB[1] = filter->integralFB[2] = 0.0f;
    filter->param.Kp = param->Kp;
    filter->param.Ki = param->Ki;
    filter->param.dt = param->dt;
    filter->last_yaw = 0;
}

// 核心更新函数
void Mahony_Update(MahonyFilter* filter, float gyro[3], float acc[3]) {
    float gx = gyro[0] * DEG_TO_RAD, gy = gyro[1] * DEG_TO_RAD, gz = gyro[2] * DEG_TO_RAD;
    float ax = acc[0], ay = acc[1], az = acc[2];
    float norm, vx, vy, vz, ex, ey, ez;
    
    // 1. 归一化加速度计数据
    norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm == 0.0f) return; // 避免除零
    ax /= norm; ay /= norm; az /= norm;
    
    // 2. 将重力向量从导航坐标系转换到机体坐标系（使用当前四元数估计值）
    vx = 2.0f*(filter->q[1]*filter->q[3] - filter->q[0]*filter->q[2]);
    vy = 2.0f*(filter->q[0]*filter->q[1] + filter->q[2]*filter->q[3]);
    vz = filter->q[0]*filter->q[0] - filter->q[1]*filter->q[1] 
         - filter->q[2]*filter->q[2] + filter->q[3]*filter->q[3];
    
    // 3. 计算加速度计测量值与估计值的误差（叉积）
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
    
    // 4. 误差积分（消除陀螺仪零偏）
    if (filter->param.Ki > 0.0f) {
        filter->integralFB[0] += filter->param.Ki * ex * filter->param.dt;
        filter->integralFB[1] += filter->param.Ki * ey * filter->param.dt;
        filter->integralFB[2] += filter->param.Ki * ez * filter->param.dt;
        
        // 应用积分校正到角速度
        gx += filter->integralFB[0];
        gy += filter->integralFB[1];
        gz += filter->integralFB[2];
    }
    
    // 5. 应用比例校正到角速度
    gx += filter->param.Kp * ex;
    gy += filter->param.Kp * ey;
    gz += filter->param.Kp * ez;
    
    // 6. 使用校正后的角速度积分更新四元数
    float q0 = filter->q[0], q1 = filter->q[1], q2 = filter->q[2], q3 = filter->q[3];
    float half_dt = 0.5f * filter->param.dt;
    
    q0 += (-q1*gx - q2*gy - q3*gz) * half_dt;
    q1 += ( q0*gx + q2*gz - q3*gy) * half_dt;
    q2 += ( q0*gy - q1*gz + q3*gx) * half_dt;
    q3 += ( q0*gz + q1*gy - q2*gx) * half_dt;
    
    // 7. 归一化四元数
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm == 0.0f) return;
    filter->q[0] = q0 / norm;
    filter->q[1] = q1 / norm;
    filter->q[2] = q2 / norm;
    filter->q[3] = q3 / norm;

    // 从四元数获取欧拉角
    q0 = filter->q[0], q1 = filter->q[1], q2 = filter->q[2], q3 = filter->q[3];
    
    filter->output.euler[0] = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * RAD_TO_DEG;
    filter->output.euler[1] = asinf(2.0f * (q0 * q2 - q1 * q3)) * RAD_TO_DEG;
    float yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * RAD_TO_DEG;
    
    // yaw unwrap
    float delta = yaw - filter->last_yaw;
    if(delta > 180.f) delta -= 360;
    if(delta < -180.f) delta += 380;

    filter->output.euler[2] += delta;
    filter->last_yaw = yaw;
}
