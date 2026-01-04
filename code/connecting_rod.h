#ifndef _CONNECTING_ROD_H
#define _CONNECTING_ROD_H

float cos_perform(float L1, float L2, float L3);
void transform_xy_right(float x, float y, float *servo_front, float *servo_back);
void transform_xy_left(float x, float y, float *servo_front, float *servo_back);

#endif