#include "ins.h"
#include "imu.h"

float x, y;

void ins_init(void)
{
    x = 0, y = 0;
}

void ins_update(float yaw, float v)
{
    // 简单的积分计算位置（仅供示例，实际应用中需要更复杂的算法）
    x += v * cosf(yaw) * (IMU_UPDATE_INTERVAL_MS / 1000.0f);
    y += v * sinf(yaw) * (IMU_UPDATE_INTERVAL_MS / 1000.0f);
}

void ins_get_data(float *x_pos, float *y_pos)
{
    *x_pos = x;
    *y_pos = y;
}