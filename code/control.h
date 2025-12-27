#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "zf_common_headfile.h"

typedef struct
{
    float Actual;
    float Target;
    float Out;
    float Kp;
    float Ki;
    float Kd;
    float Integral;
}PID_t;

extern PID_t ang1_loop;



#endif