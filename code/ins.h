#ifndef INS_H
#define INS_H

#include "config.h"

void ins_init(void);

/// @brief 更新位置估计
/// @param yaw 弧度
/// @param v m/s
void ins_update(float yaw, float v);

void ins_get_data(float *x_pos, float *y_pos);

#endif // INS_H