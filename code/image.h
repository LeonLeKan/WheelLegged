#ifndef _IMAGE_H
#define _IMAGE_H

#define ADS 1

#if ADS

#include "zf_common_headfile.h"
#include <stdio.h>
#include <stdlib.h>
#include <arm_math.h>
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
#define NEAR_LINE 85//图像处理下边界
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

extern int error;
extern int error_calc_near_line;
extern int error_calc_far_line;

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
//extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//指向灰度图的首地址
extern uint8_t mid_line[CAMERA_H];
extern uint8_t left_line[CAMERA_H], right_line[CAMERA_H];
extern uint8_t threshold_1;
extern uint8_t threshold_2;
extern uint8_t top_road;
extern uint8_t roundabout_state;
extern uint8_t traverse_state;
extern uint8_t traverse_near_road;
extern uint8_t bend_state;
//菜单可调数据
extern uint8_t cross_judge_end;
extern uint8_t s_state;
extern uint8_t black_check;
extern uint8_t traverse_on;
extern uint8_t traverse_flag;
extern float traverse_angle;
extern uint8_t mid_update;
extern uint8_t gray_dis;
extern uint8_t single_place;
extern uint8_t right_pianyi;
extern uint8_t left_pianyi;
//元素计数
extern uint8_t traverse_num;
extern uint8_t cross_num;
extern uint8_t roundabout_num;
extern uint8_t ramp_num;
extern uint8_t single_num;
extern uint8_t zebra_num;

extern uint8_t single_straight_flag;
extern uint8_t single_block_d;
extern uint8_t single_round_d;//在第几个环岛后开始判单边桥
extern uint8_t single_cross_d;//在过环岛后第几个十字后开始判单边桥
extern uint8_t single_u;//最多判几次单边桥
extern uint8_t single_side_state;
extern uint8_t traverse_round_d;//在第几个环岛后开始判横断桥
extern uint8_t traverse_cross_d;//在过环岛后第几个十字后开始判横断桥
extern uint8_t traverse_u;//最多判几次横断桥
extern uint8_t ramp_round_d;//在经过第几个环岛后开始判坡道
extern uint8_t ramp_cross_d;//在经过环岛后第几个十字后开始判坡道
extern uint8_t ramp_u;//坡道上限

extern uint8_t threshold_mode;
extern int8_t threshold_minus1;
extern int8_t threshold_minus2;
extern uint8_t traverse_mode;

//新增参数
extern uint8_t zebra_round_d;
extern uint8_t zebra_cross_d;
extern uint8_t aboutround_fps_up;
extern uint8_t right_guai_H;
extern uint8_t ramp_end;
extern uint8_t cross_state;

//环岛
extern float round_k;//表示向右移动的距离倍数
extern uint8_t round_b;
extern uint8_t round_b2;

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
uint8_t GetOTSU_part(uint8_t* imageInput, uint8_t height_far, uint8_t height_near, uint8_t widthleft, uint8_t widthright);
uint8_t Iteration_Threshould(uint8_t* imageInput, uint8_t BlackThres, uint8_t height_far, uint8_t height_near, uint8_t width_left, uint8_t width_right);
void sobel(uint8_t* imageInput, uint8_t* imageOut, uint8_t Theshold, uint8_t height_far, uint8_t height_near, uint8_t width_left, uint8_t width_right);
uint8_t Get_threshold_part(uint8_t* imageInput, uint8_t height_far, uint8_t height_near, uint8_t width_left, uint8_t width_right);
void print_f(void);//打印数值的函数
void datas_init(void);
void get_white_threshold_avg(void);
void threshold_decrease(void);
void threshold_add_gray(void);
road_range search_white_range_begin(uint8_t line, uint8_t col);
uint8_t is_connect_continue(road_range* new_white, road_range* pre_line);
uint8_t is_connect(road_range* new_white, road* pre_line, uint8_t continue_id);

//====================================================判断状态函数====================================================

void state_judge(void);
void zebra_judge(uint8_t start, uint8_t end);
void roundabout_judge(uint8_t start, uint8_t end);
void crossing_judge(uint8_t start, uint8_t end);
void straight_judge(uint8_t start, uint8_t end);
void roadblock_judge(uint8_t start, uint8_t end);
void single_side_bridge_judge2(uint8_t start);
void single_side_bridge_judge(uint8_t start, uint8_t end);
void traverse_brige_judge(void);
void ramp_judge(uint8_t start, uint8_t end);
void out_check();
void state_light(uint8_t num);
void bend_judge(void);

//===================================================各元素状态机=================================================

void general_judgement(void);
void cross_state_machine(void);
void zebra_crossing_state_machine(void);
void right_roundabout_state_machine(void);
void left_roundabout_state_machine(void);
void right_roadblock_state_machine(void);
void left_roadblock_state_machine(void);
void traverse_bridge_state_machine(void);
void single_side_bridge_state_machine(void);
void ramp_state_machine(void);
void straight(void);
void bend(uint8_t start);

//====================================================选道====================================================

void left_roundabout_road_choose(uint8_t* road_continue);                             //左环岛选道
void right_roundabout_road_choose(uint8_t* road_continue);                            //右环岛选道
void renew_road(void);                                                                //按中间选道
void traverse_bridge_road_choose(void);                                               //横断桥选赛道
void single_side_road_choose(void);                                                   //单边桥选两边
void traverse_bridge_get_twp_line(uint8_t* road_continue);                            //横断桥选两边
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
extern uint8_t step;
