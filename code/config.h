#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "zf_common_headfile.h"

#define IMU_UPDATE_PIT PIT_CH0
#define IMU_UPDATE_INTERVAL_MS 2  // IMU更新间隔，单位毫秒

#define SCREEN_UPDATE_PIT PIT_CH15
#define SCREEN_UPDATE_INTERVAL_MS 100 // 显点屏幕更新间隔，单位毫秒

// 车辆尺寸参数
#define WHEEL_D     0.065   // 轮胎直径
#define WHEELBASE   0.185   // 轮距

#endif