#include "connecting_rod.h"
#include <arm_math.h>

//front is for x axis and down is for y axis for each side

float cos_perform(float L1, float L2, float L3)
{
    float angel;
    angel=(L1*L1+L2*L2-L3*L3)/(2*L1*L2);
    return acos(angel)*180.0f/PI;
}

void transform_xy_right(float x, float y, float volatile *servo_front, float volatile *servo_back)
{
    //L1 is for middle, L2 is for front and L3 is for back
    float L1 = sqrt(x*x + y*y);
    float L2 = sqrt((x-1.9)*(x-1.9) + y*y);
    float L3 = sqrt((x+1.9)*(x+1.9) + y*y);
    float angle1 = cos_perform(1.9, L2, L1);
    float angle2 = cos_perform(L2, 6.0, 9.0);
    float angle3 = cos_perform(1.9, L3, L1);
    float angle4 = cos_perform(L3, 6.0, 9.0);
    
    *servo_front = angle1 + angle2 - 90.0f;
    *servo_back  = 270.0f - angle3 - angle4;
}

void transform_xy_left(float x, float y, float volatile *servo_front, float volatile *servo_back)
{
    //L1 is for middle, L2 is for front and L3 is for back
    float L1 = sqrt(x*x + y*y);
    float L2 = sqrt((x-1.9)*(x-1.9) + y*y);
    float L3 = sqrt((x+1.9)*(x+1.9) + y*y);
    float angle1 = cos_perform(1.9, L2, L1);
    float angle2 = cos_perform(L2, 6.0, 9.0);
    float angle3 = cos_perform(1.9, L3, L1);
    float angle4 = cos_perform(L3, 6.0, 9.0);
    
    *servo_front = 270.0f - angle1 - angle2;
    *servo_back  = angle3 + angle4 - 90.0f;
}
