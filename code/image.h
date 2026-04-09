#ifndef _IMAGE_H
#define _IMAGE_H

#define ONBOARD 1

#if ONBOARD

#include "zf_common_headfile.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
extern uint32 out_check_flag;
#else

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#endif



//注意在VS中头文件是上述写法，因为使用到了opencv库的"cv.h"  "highgui.h"
//在ADS中，要去掉 "cv.h"  "highgui.h"，加入逐飞的头文件"zf_common_headfile.h"，如下




#define MISS 255
#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define NEW_CAMERA_H 188
#define My_Max(x, y) ((x) > (y) ? (x) : (y))
#define My_abs(x)    ((x)>0 ? (x):-(x))
#define FAR_LINE 1//图像处理上边界
#define NEAR_LINE 95//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 255
#define white_num_MAX 13//每行最多允许白条数
#define JUDGE_END 20//判断状态的截至行

/////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
///////////////////////////

///////////////////////////
#define strai            0
#define cross            1
#define zebra            2
#define right_roundabout 3
#define left_roundabout  4
#define right_roadblock  5
#define left_roadblock   6
#define traverse         7
#define single_side      8
#define ramp             9
////////////////////////////

typedef enum {
    NORM,
    SINGLE_SIDE,
    FRONT_STAIRS,
    BACK_STAIRS,
    JOLT
}element_t;

//////////////////////// 传给控制 ////////////////////
// 1-单边桥 2-三级跳 3-下三级 4-颠簸路段
extern element_t now_element;
extern int stair_distance;     // 到下一级台阶的距离
extern bool front_stairs_end_flag;
extern bool back_stairs_end_flag;
extern bool jolt_end_flag;
extern int error;
extern int error_calc_near_line;
extern int error_calc_far_line;
extern uint8_t threshold; //阈值
extern uint8_t single_side_state;
extern uint8_t mid_update;
extern int jolt_end_judge_cnt;
//////////////////////// ////////// //////////////////

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
//extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//指向灰度图的首地址
extern uint8_t mid_line[CAMERA_H];
extern uint8_t left_line[CAMERA_H], right_line[CAMERA_H];


//属于赛道的每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    uint8_t   width;//宽度
}road_range;
//每行属于赛道的每个白条子
typedef struct {
    uint8_t   white_num;
    road_range   connected[white_num_MAX];
}road;

void head_clear(void);
void THRE(void);
int find_f(int a);
void search_white_range1(void);//获取白条子
road_range  search_white_range(uint8_t line, uint8_t col);
void adaptive_threshold_2(uint8_t y, uint8_t x);
void find_all_connect(void);
void find_road(void);
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
void get_mid_line(void);
void image_main(void);
//==================================================其他函数======================================================
uint8_t mid(uint8_t i);
void find_all_connect_easy();
void get_line_easy();
void print_f(void);//打印数值的函数
void datas_init(void);
void get_white_threshold_avg(void);
void threshold_decrease(void);
road_range search_white_range_begin(uint8_t line, uint8_t col);
uint8_t is_connect_continue(road_range* new_white, road_range* pre_line);
uint8_t is_connect(road_range* new_white, road* pre_line, uint8_t continue_id);

//====================================================判断状态函数====================================================

void state_judge(void);
void straight_judge(uint8_t start, uint8_t end);
void single_side_bridge_judge2(uint8_t start);
void single_side_bridge_judge(uint8_t start, uint8_t end);
void bend_judge(void);

//===================================================各元素状态机=================================================

void general_judgement(void);
void single_side_bridge_state_machine(void);
void straight(void);
void bend(uint8_t start);

//====================================================选道====================================================
void renew_road(void);                                                                //按中间选道
void single_side_road_choose(void);                                                   //单边桥选两边
void single_side_bridge_two_line(uint8_t* road_continue);
void find_road_state(void);

//====================================================一些找点的函数====================================================

uint8_t get_left_line_x_max_y(uint8_t* line, uint8_t start, uint8_t end);
uint8_t get_right_line_x_min_y(uint8_t* line, uint8_t start, uint8_t end);
uint8_t get_line_change(uint8_t* line, uint8_t start, uint8_t end, uint8_t change_limit);
uint8_t get_line_f2_max_y(uint8_t* line, uint8_t start, uint8_t end);
uint8_t get_line_f2_forward_y(uint8_t* line, uint8_t start, uint8_t end, uint8_t change_limit, uint8_t dis);
uint8_t get_line_f2_forward_y_down(uint8_t* line, uint8_t start, uint8_t end, uint8_t change_limit);
uint8_t get_white_width_small(uint8_t start, uint8_t end, uint8_t change_limit);
uint8_t get_white_width_larger(uint8_t start, uint8_t end, uint8_t change_limit);
uint8_t get_line_side(uint8_t* line, uint8_t start, uint8_t end, uint8_t limit);
uint8_t judge_white_width_charge(uint8_t y1, uint8_t y2, uint8_t charge);
uint8_t get_white_num_more_y(uint8_t start, uint8_t end);
uint8_t get_white_num_less_y(uint8_t start, uint8_t end, uint8_t white_num);
uint8_t get_white_num_most_y(uint8_t start, uint8_t end);
uint8_t get_straight_line(uint8_t* line, uint8_t start, uint8_t end, uint8_t change);
int8_t get_line_monotony(uint8_t* line, uint8_t start, uint8_t end, uint8_t change);
uint8_t last_one_left_single_check(void);
uint8_t last_one_right_single_check(void);

//====================================================一些有关补线的函数======================================================

uint8_t get_least_square_limit(uint8_t* line, uint8_t outset, uint8_t end, uint8_t change, uint8_t start);
void least_square(uint8_t* line, uint8_t outset, uint8_t limit, float input[]);
void two_dots_line(uint8_t dots1_H, uint8_t dots1_W, uint8_t dots2_H, uint8_t dots2_W, uint8_t* line);
void least_square_get_line(uint8_t* line, uint8_t outset, uint8_t limit, float input[], uint8_t start, uint8_t end);
void get_line_b(uint8_t* line, uint8_t x, float input[]);
void get_line_b_and_k(uint8_t* line, uint8_t start, uint8_t end, float input[]);
void get_line(uint8_t* line, uint8_t start, uint8_t end, float k, float b);
void define_mid_255(void);
#endif //
