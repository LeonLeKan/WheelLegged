#include "image.h"

#if ONBOARD
uint8_t* fullBuffer = &mt9v03x_image[0][0];
#endif

int f[10 * CAMERA_H];//考察连通域联通性
//每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    int   connect_num;//连通标记
}range;


//每行的所有白条子
typedef struct {
    uint8_t   num;//每行白条数量
    range   area[white_num_MAX];//该行各白条区域
}all_range;



//////////////////////// 传给控制 ////////////////////
// 1-单边桥 2-三级跳 3-下三级 4-颠簸路段
element_t now_element;
int stair_distance;     // 到下一级台阶的距离
bool front_stairs_end_flag = 0;
bool back_stairs_end_flag = 0;
bool jolt_end_flag = 0;
int error = 0;
int error_calc_near_line = 42;
int error_calc_far_line = 40;
uint8_t threshold = 230;    //阈值
bool on_single_side_road = 0;   // 是否进入单边桥之前的白色赛道
//////////////////////// ////////// //////////////////



uint8_t road_start = NEAR_LINE;
uint8_t road_end = FAR_LINE;
all_range white_range[CAMERA_H];//所有白条子
road my_road[CAMERA_H];//赛道
road_range myroad_continue[NEW_CAMERA_H];
uint8_t j_continue[CAMERA_H];
uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
uint8_t boundaries[CAMERA_H][CAMERA_W];
uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//赛道的左右边界
uint8_t mid_line[CAMERA_H];
float left_line_datas[2] = { 0 };//左边线的斜率和截距
float right_line_datas[2] = { 0 };//右边线的斜率和截距
int all_connect_num = 0;//所有白条子数
uint8_t top_road;//赛道最高处所在行数
uint8_t mid_choose = 0;
uint8_t white_num_max = 0;
uint8_t block = 13;
uint8_t last_midline = 94;
uint8_t double_thre = 0;
uint8_t extract_bound = 1;

//赛道判断所需要的一些数据

//直道
uint8_t left_f2_y = 0;//左边线二阶导拐点
uint8_t right_f2_y = 0;//右边线二阶导拐点

//单边桥
uint8_t single_side_left_f2_y = 0;//左边线突变点
uint8_t single_side_right_f2_y = 0;//右边线突变点
uint8_t single_side_white_smller = 0;
uint8_t single_left_side_y = 0;
uint8_t single_right_side_y = 0;
uint8_t single_place = 0;
uint8_t left_nb = 0;
uint8_t right_nb = 0;
uint8_t single_start_fps = 0;
uint8_t single_start_fps_up;
float single_dis = 0;

int front_stairs_end_judge_cnt = 0;
int back_stairs_end_judge_cnt = 0;

// 颠簸路段
int jolt_end_judge_cnt = 0;


//补线需要的数据
uint8_t left_guai_H = 0;//左下边界拐点
uint8_t right_guai_H = 0;//右下边界拐点
uint8_t left_guai_H2 = 0;//左上边界拐点
uint8_t right_guai_H2 = 0;//右上边界拐点

//判断状态的参数
uint8_t step = 0;
uint8_t pre_step = 0;
uint8_t yuansu_state = 0;
uint8_t cross_state = 0;
uint8_t zebra_state = 0;
uint8_t roundabout_state = 0;
uint8_t roadblock_state = 0;
uint8_t traverse_state = 0;
uint8_t single_side_state = 0;
uint8_t ramp_state = 0;
uint8_t s_state = 0;
uint8_t single_side_fps = 0;
uint8_t single_straight_flag = 0;
uint8_t bend_state = 0;
//控制横断桥判断的开关
uint8_t traverse_on = 0;



//元素计数
uint8_t traverse_num = 0;
uint8_t cross_num = 0;
uint8_t roundabout_num = 0;
uint8_t ramp_num = 0;
uint8_t single_num = 0;
uint8_t zebra_num = 0;
uint8_t bend_fps = 0;
uint8_t mid_update = 0;
uint8_t roadblock_num = 0;

//菜单可调数据
uint8_t single_round_d = 0;//在第几个环岛后开始判单边桥
uint8_t single_cross_d = 0;//在过环岛后第几个十字后开始判单边桥
uint8_t single_block_d = 0;//在经过第几个避障后判坡道
uint8_t single_u = 100;//最多判几次单边桥
uint8_t traverse_round_d = 0;//在第几个环岛后开始判横断桥
uint8_t traverse_cross_d = 0;//在过环岛后第几个十字后开始判横断桥
uint8_t traverse_u = 100;//最多判几次横断桥
uint8_t ramp_round_d = 0;//在经过第几个环岛后开始判坡道
uint8_t ramp_cross_d = 0;//在经过环岛后第几个十字后开始判坡道
uint8_t ramp_u = 5;//坡道上限
uint8_t bend_judge_start = 60;
uint8_t round_out_k = 40;//摄像头越远该数字越大
uint8_t round_out_b = 30;//摄像头越近该数字越大
//新增参数
uint8_t zebra_round_d = 0;
uint8_t zebra_cross_d = 0;
//究极人机交互，预防单边桥和避障误判
uint32_t element_sort = 0;//1代表单边桥，2代表避障，3代表横断桥
uint8_t element_ok = 0;
uint8_t element_round_d = 0;
uint8_t element_cross_d = 0;


//////////////////////////////////
// 通用检测
// 检测各种状态
// 是否进入状态机
//////////////////////////////////
void general_judgement() {
    state_judge();
    uint8_t bend_judge_start = (road_start < 60) ? 60 : road_start;
    switch (step)
    {
    case strai: break;
    case single_side: single_side_bridge_state_machine(); break;
    }
}


//单边桥状态机
void single_side_bridge_state_machine()
{
    switch (single_side_state)
    {
        //左单边桥在前
    case 1: {
        right_guai_H = NEAR_LINE - 10;
        left_guai_H = NEAR_LINE - 10;
        while (left_guai_H = get_line_f2_forward_y(left_line, left_guai_H, top_road + 5, 7, 5))
        {
            if (2 * left_line[left_guai_H] - left_line[left_guai_H - 5] - left_line[left_guai_H + 5] < -7 && 2 * left_line[left_guai_H] - left_line[left_guai_H - 5] - left_line[left_guai_H + 5] > -60 && left_line[left_guai_H] - left_line[left_guai_H - 5] < -7)
            {
                if (left_guai_H > 50)
                {
                    if (left_line[left_guai_H] > LEFT_SIDE + 15)
                    {
                        break;
                    }
                    else
                    {
                        left_guai_H = left_guai_H - 1;
                    }
                }
                else
                {
                    break;
                }
            }
            else
            {
                left_guai_H = left_guai_H - 1;
            }
        }
        while (1)
        {
            right_guai_H = get_line_f2_forward_y(right_line, right_guai_H, top_road + 5, 10, 5);
            if (right_guai_H == 0)
            {
                break;
            }
            if (right_line[right_guai_H] - right_line[right_guai_H + 5] < 5 && right_line[right_guai_H] - right_line[right_guai_H + 5] > -20 &&
                right_line[right_guai_H] - right_line[right_guai_H - 5] > 7 && right_line[right_guai_H] - right_line[right_guai_H - 5] < 40)
            {
                if (right_guai_H > 50)
                {
                    if (right_line[right_guai_H] < RIGHT_SIDE - 15)
                    {
                        break;
                    }
                    else
                    {
                        right_guai_H = right_guai_H - 1;
                    }
                }
                else
                {
                    break;
                }
            }
            else
            {
                right_guai_H = right_guai_H - 1;
            }
        }
        if (right_guai_H == 0)
        {
            right_guai_H = top_road + 5;
        }
        if (left_guai_H == 0)
        {
            left_guai_H = top_road + 5;
        }
        //找单边桥的标志点
        for (uint8_t i = left_guai_H; i > top_road + 1; i--)
        {
            if (left_line[i] - left_line[i - 1] < -10 || (left_line[i] >= left_line[i - 1] && left_line[i] >= left_line[i + 1] && left_line[i] > left_line[i + 2]))
            {
                single_place = i;
                break;
            }
        }
        if (single_place == 0)
        {
            single_place = left_guai_H - 3;
        }
        //算距离单边桥的直线距离
        single_dis = sqrt(((NEAR_LINE - single_place) * (NEAR_LINE - single_place) + (mid_line[NEAR_LINE] - left_line[single_place]) * (mid_line[NEAR_LINE] - left_line[single_place])));
        least_square_get_line(right_line, right_guai_H + 10, 10, right_line_datas, right_guai_H, FAR_LINE);
        least_square_get_line(left_line, left_guai_H + 10, 10, left_line_datas, left_guai_H, FAR_LINE);
        if (left_guai_H >= 65)
        {
            single_side_state = 3;
            single_place = 0;
            single_dis = 0;
        }
    }break;
          //右单边桥在前
    case 2: {
        right_guai_H = NEAR_LINE - 10;
        left_guai_H = NEAR_LINE - 10;
        while (right_guai_H = get_line_f2_forward_y(right_line, right_guai_H, top_road + 5, 7, 5))
        {
            if (2 * right_line[right_guai_H] - right_line[right_guai_H - 5] - right_line[right_guai_H + 5] > 7 && 2 * right_line[right_guai_H] - right_line[right_guai_H - 5] - right_line[right_guai_H + 5] < 60 && right_line[right_guai_H] - right_line[right_guai_H - 5] > 7)
            {
                if (right_guai_H > 50)
                {
                    if (right_line[right_guai_H] < RIGHT_SIDE - 15)
                    {
                        break;
                    }
                    else
                    {
                        right_guai_H = right_guai_H - 1;
                    }
                }
                else
                {
                    break;
                }
            }
            else
            {
                right_guai_H = right_guai_H - 1;
            }
        }
        while (1)
        {
            left_guai_H = get_line_f2_forward_y(left_line, left_guai_H, top_road + 5, 10, 5);
            if (left_guai_H == 0)
            {
                break;
            }
            if (left_line[left_guai_H] - left_line[left_guai_H + 5] > -5 && left_line[left_guai_H] - left_line[left_guai_H + 5] < 20 &&
                left_line[left_guai_H] - left_line[left_guai_H - 5] < -7 && left_line[left_guai_H] - left_line[left_guai_H - 5] > -40)
            {
                if (left_guai_H > 50)
                {
                    if (left_line[left_guai_H] > LEFT_SIDE + 15)
                    {
                        break;
                    }
                    else
                    {
                        left_guai_H = left_guai_H - 1;
                    }
                }
                else
                {
                    break;
                }
            }
            else
            {
                left_guai_H = left_guai_H - 1;
            }
        }
        if (left_guai_H == 0)
        {
            left_guai_H = top_road + 5;
        }
        if (right_guai_H == 0)
        {
            right_guai_H = top_road + 5;
        }
        //找单边桥的标志点
        for (uint8_t i = right_guai_H; i > top_road + 1; i--)
        {
            if (right_line[i] - right_line[i - 1] > 10 || (right_line[i] <= right_line[i - 1] && right_line[i] <= right_line[i + 1] && right_line[i] < right_line[i + 2]))
            {
                single_place = i;
                break;
            }
        }
        if (single_place == 0)
        {
            single_place = right_guai_H - 3;
        }
        single_dis = sqrt(((NEAR_LINE - single_place) * (NEAR_LINE - single_place) + (mid_line[NEAR_LINE] - right_line[single_place]) * (mid_line[NEAR_LINE] - right_line[single_place])));
        least_square_get_line(right_line, right_guai_H + 10, 10, right_line_datas, right_guai_H, FAR_LINE);
        least_square_get_line(left_line, left_guai_H + 10, 10, left_line_datas, left_guai_H, FAR_LINE);
        if (right_guai_H >= 65)
        {
            single_side_state = 4;
            single_place = 0;
            single_dis = 0;
        }
    }break;
          //正在过左单边桥
    case 3: {
        uint8_t NEAR_LINE_white_width = right_line[NEAR_LINE] - left_line[NEAR_LINE];
        uint8_t left_white_small = 0;
        uint8_t left_white_larger = 0;
        uint8_t no_small = 0;
        uint8_t no_larger = 0;
        left_guai_H = NEAR_LINE;
        //判断是否右边为最后一个单边桥
        right_guai_H = last_one_right_single_check();
        if (right_guai_H == 0)
        {
            //那就说明是最后一个单边桥了，这时候，左边线从NEAR_LINE开始找，找拐点
            left_white_small = NEAR_LINE - 5;
            while (left_white_small = get_line_f2_forward_y(left_line, left_white_small, top_road + 10, 10, 5))
            {
                if (2 * left_line[left_white_small] - left_line[left_white_small - 5] - left_line[left_white_small + 5] < -10 &&
                    left_line[left_white_small] - left_line[left_white_small - 5] < -8)
                {
                    break;
                }
                else
                {
                    left_white_small = left_white_small - 1;
                }
            }
            if (left_white_small == 0 || left_white_small < 40)
            {
                left_white_small = NEAR_LINE;
                no_small = 1;
            }
            left_white_larger = left_white_small - 5;
            while (left_white_larger = get_line_f2_forward_y(left_line, left_white_larger, top_road + 10, 10, 5))
            {
                if (2 * left_line[left_white_larger] - left_line[left_white_larger - 5] - left_line[left_white_larger + 5] > 5 &&
                    left_line[left_white_larger] - left_line[left_white_larger - 5] > 8)
                {
                    break;
                }
                else
                {
                    left_white_larger = left_white_larger - 1;
                }
            }
            if (left_white_larger == 0)
            {
                left_white_larger = left_white_small;
                no_larger = 1;
            }
            if ((no_small == 1 && no_larger == 1) || (no_small == 1 && no_larger == 0 && left_white_larger <= 40) || (no_small == 0 && no_larger == 1))
            {
                left_guai_H = NEAR_LINE - 10;
            }
            else if ((no_small == 1 && no_larger == 0 && left_white_larger > 40) || (no_small == 0 && no_larger == 0))
            {
                for (uint8_t i = left_white_larger; i > top_road + 7; i--)
                {
                    if (left_line[i] < left_line[i - 1] && left_line[i - 1] <= left_line[i - 2] && left_line[i] < left_line[i - 3] && left_line[i - 1] < left_line[i - 4]
                        && left_line[i - 2] < left_line[i - 5])
                    {
                        left_guai_H = i;
                        break;
                    }
                }
                if (left_guai_H == 0)
                {
                    left_guai_H = left_white_larger - 10;
                }

            }
            right_guai_H = 35;
        }
        else
        {
            for (int i = right_guai_H; i < NEAR_LINE; i++)
            {
                if (left_line[i] > left_line[i - 1])
                {
                    left_guai_H = i - 1;
                    break;
                }
            }
        }
        least_square_get_line(left_line, left_guai_H, 10, left_line_datas, NEAR_LINE, FAR_LINE);
        least_square_get_line(right_line, right_guai_H + 10, 10, right_line_datas, right_guai_H, FAR_LINE);
        if (left_guai_H >= 68 && right_guai_H <= 40 && NEAR_LINE_white_width > 65 && left_nb == 1)
        {
            single_side_state = 0;
            single_side_fps = 0;
            step = 0;
            single_num++;
            single_straight_flag = 0;
            left_nb = 0;
            right_nb = 0;
        }
        else if (right_guai_H >= 65 || left_guai_H >= 70)
        {
            single_side_state = 4;
            right_nb = 1;
        }

    }break;
          //正在过右单边桥
    case 4: {
        uint8_t NEAR_LINE_white_width = right_line[NEAR_LINE] - left_line[NEAR_LINE];
        uint8_t right_white_small = 0;
        uint8_t right_white_larger = 0;
        uint8_t no_small = 0;
        uint8_t no_larger = 0;
        right_guai_H = NEAR_LINE;
        //判断是否为最后一个单边桥
        left_guai_H = last_one_left_single_check();
        if (left_guai_H == 0)
        {
            //那就说明是最后一个单边桥了，这时候，右边线从NEAR_LINE开始找，找拐点
            right_white_small = NEAR_LINE - 5;
            while (right_white_small = get_line_f2_forward_y(right_line, right_white_small, top_road + 10, 10, 5))
            {
                if (2 * right_line[right_white_small] - right_line[right_white_small - 5] - right_line[right_white_small + 5] > 10 &&
                    right_line[right_white_small] - right_line[right_white_small - 5] > 8)
                {
                    break;
                }
                else
                {
                    right_white_small = right_white_small - 1;
                }
            }

            if (right_white_small == 0 || right_white_small < 40)
            {
                right_white_small = NEAR_LINE;
                no_small = 1;
            }
            right_white_larger = right_white_small - 5;
            while (right_white_larger = get_line_f2_forward_y(right_line, right_white_larger, top_road + 10, 10, 5))
            {
                if (2 * right_line[right_white_larger] - right_line[right_white_larger - 5] - right_line[right_white_larger + 5] < -5 &&
                    right_line[right_white_larger] - right_line[right_white_larger - 5] < -8)
                {
                    break;
                }
                else
                {
                    right_white_larger = right_white_larger - 1;
                }
            }
            if (right_white_larger == 0)
            {
                right_white_larger = right_white_small;
                no_larger = 1;
            }
            if ((no_small == 1 && no_larger == 1) || (no_small == 1 && no_larger == 0 && right_white_larger <= 40) || (no_small == 0 && no_larger == 1))
            {
                right_guai_H = NEAR_LINE - 10;
            }
            else if ((no_small == 1 && no_larger == 0 && right_white_larger > 40) || (no_small == 0 && no_larger == 0))
            {
                for (uint8_t i = right_white_larger; i > top_road + 7; i--)
                {
                    if (right_line[i] > right_line[i - 1] && right_line[i - 1] >= right_line[i - 2] && right_line[i - 1] > right_line[i - 4] && right_line[i - 2] > right_line[i - 5]
                        && right_line[i] > right_line[i - 3])
                    {
                        right_guai_H = i;
                        break;
                    }
                }
                if (right_guai_H == 0)
                {
                    right_guai_H = right_white_larger - 10;
                }
            }
            left_guai_H = 35;

        }
        else
        {
            for (int i = left_guai_H; i < NEAR_LINE; i++)
            {
                if (right_line[i] < right_line[i - 1])
                {
                    right_guai_H = i - 1;
                    break;
                }
            }
        }

        least_square_get_line(left_line, left_guai_H + 10, 10, left_line_datas, left_guai_H, FAR_LINE);
        least_square_get_line(right_line, right_guai_H, 10, right_line_datas, NEAR_LINE, FAR_LINE);
        if (right_guai_H >= 68 && left_guai_H <= 40 && NEAR_LINE_white_width > 65 && right_nb == 1)
        {
            single_side_fps = 0;
            single_side_state = 0;
            step = 0;
            single_num++;
            single_straight_flag = 0;
            left_nb = 0;
            right_nb = 0;
        }
        else if (left_guai_H >= 65 || right_guai_H >= 70)
        {
            single_side_state = 3;
            left_nb = 1;
        }

    }break;
          break;
    }
    //printf("single_dis = %f\n", single_dis);
}

//直道补线
void straight()
{
    uint8_t limit = 40;
    if (limit + top_road > NEAR_LINE)
    {
        limit = NEAR_LINE - top_road - 1;
    }
    uint8_t outset = NEAR_LINE;
    least_square_get_line(left_line, outset, limit, left_line_datas, outset, FAR_LINE);
    least_square_get_line(right_line, outset, limit, right_line_datas, outset, FAR_LINE);
}


//判断进入直道
void straight_judge(uint8_t start, uint8_t end)
{
    left_f2_y = get_line_f2_forward_y(left_line, start, end, 5, 5);
    right_f2_y = get_line_f2_forward_y(right_line, start, end, 5, 5);
}


//单边桥判断
void single_side_bridge_judge(uint8_t start, uint8_t end)
{
    uint8_t single_side_left_f2_y2 = 0;//左边线突变点2
    uint8_t single_side_right_f2_y2 = 0;//右边线突变点2
    uint8_t single_side_left_f2_y3 = 0;//左边线突变点3
    uint8_t single_side_right_f2_y3 = 0;//右边线突变点3
    uint8_t single_right_ok2 = 0;
    uint8_t single_left_ok2 = 0;
    uint8_t single_left_side_y2 = get_line_side(left_line, NEAR_LINE - 30, top_road, 5);
    uint8_t single_right_side_y2 = get_line_side(right_line, NEAR_LINE - 30, top_road, 5);
    int8_t left_inde_dis = get_line_monotony(left_line, start, top_road + 4, 2);
    int8_t right_inde_dis = get_line_monotony(right_line, start, top_road + 4, 2);
    uint8_t single_white_num = 0;
    uint8_t right_side_num = 0;
    uint8_t left_side_num = 0;
    uint8_t white_number = 0;
    uint8_t white_width_num = 0;
    float right_y1 = 0;
    float left_y1 = 0;
    float right_y2 = 0;
    float left_y2 = 0;
    float left_k1 = 0;
    float left_k2 = 0;
    float right_k1 = 0;
    float right_k2 = 0;
    //先判断左右两边二阶导大于某个值，然后再判断二阶导上方直线的斜率在某个范围
    //然后继续在上面判断另一个拐角
    single_side_left_f2_y = start;
    single_side_right_f2_y = start;
    //白条子缩短，两边线均存在二阶导，并且两边线二阶导纵坐标距离大于某个值，小于某个值
    if (left_inde_dis > 1 && right_inde_dis < -1)
    {
        single_side_left_f2_y = 0;
        single_side_right_f2_y = 0;
    }
    else
    {
        if (right_inde_dis < 0)
        {
            while (1)
            {
                single_side_left_f2_y = get_line_f2_forward_y(left_line, single_side_left_f2_y, top_road + 4, 6, 5);
                if (single_side_left_f2_y == 0 || (left_line[single_side_left_f2_y + 1] - left_line[single_side_left_f2_y] > 10))
                {
                    single_side_left_f2_y = 0;
                    break;
                }
                else
                {
                    if (left_line[single_side_left_f2_y] - left_line[single_side_left_f2_y - 1] < -20)
                    {
                        left_y2 = (float)(left_line[single_side_left_f2_y - 2] - left_line[single_side_left_f2_y]);
                        left_k2 = atan2(left_y2, -1);
                    }
                    else
                    {
                        least_square(left_line, single_side_left_f2_y, 5, left_line_datas);
                        left_k2 = atan2(-left_line_datas[0], -1);
                    }
                    left_y1 = (float)(left_line[single_side_left_f2_y + 10] - left_line[single_side_left_f2_y]);
                    left_k1 = atan2(left_y1, 10);
                    if (left_k2 - left_k1 < 2.4 && left_k2 - left_k1 > 1.15)
                    {
                        break;
                    }
                    else
                    {
                        single_side_left_f2_y = single_side_left_f2_y - 1;
                    }
                }

            }
        }
        else
        {
            single_side_left_f2_y = 0;
        }
        if (left_inde_dis > 0)
        {
            while (1)
            {
                single_side_right_f2_y = get_line_f2_forward_y(right_line, single_side_right_f2_y, top_road + 4, 6, 5);
                if (single_side_right_f2_y == 0 || (right_line[single_side_right_f2_y + 1] - right_line[single_side_right_f2_y]) < -10)
                {
                    single_side_right_f2_y = 0;
                    break;
                }
                else
                {
                    if (right_line[single_side_right_f2_y] - right_line[single_side_right_f2_y - 1] > 20)
                    {
                        right_y2 = (float)(right_line[single_side_right_f2_y - 2] - right_line[single_side_right_f2_y]);
                        right_k2 = atan2(right_y2, -1);
                    }
                    else
                    {
                        least_square(right_line, single_side_right_f2_y, 5, right_line_datas);
                        right_k2 = atan2(-right_line_datas[0], -1);
                    }
                    right_y1 = (float)(right_line[single_side_right_f2_y + 10] - right_line[single_side_right_f2_y]);
                    right_k1 = atan2(right_y1, 10);
                    if (right_k2 - right_k1 > -2.4 && right_k2 - right_k1 < -1.15)
                    {
                        break;
                    }
                    else
                    {
                        single_side_right_f2_y = single_side_right_f2_y - 1;
                    }
                }
            }
        }
        else
        {
            single_side_right_f2_y = 0;
        }
    }

    if (single_side_right_f2_y != 0 && single_side_left_f2_y != 0)
    {
        for (uint8_t i = single_side_right_f2_y - 3; i >= top_road; i--)
        {
            if (right_line[single_side_right_f2_y] <= right_line[i])
            {
                single_side_right_f2_y = 0;
                break;
            }
        }
    }
    if (single_side_right_f2_y != 0 && single_side_left_f2_y != 0)
    {
        for (uint8_t i = single_side_left_f2_y - 3; i >= top_road; i--)
        {
            if (left_line[single_side_left_f2_y] >= left_line[i])
            {
                single_side_left_f2_y = 0;
                break;
            }
        }
    }
    if (single_side_left_f2_y == 0 && single_side_right_f2_y != 0 && get_straight_line(left_line, 30, top_road + 2, 10) == 0)
    {
        //找到单边桥右边的第二个拐点
        for (uint8_t i = single_side_right_f2_y - 5; i > top_road; i--)
        {
            if (right_line[i] <= right_line[i - 1] && right_line[i] <= right_line[i - 2] && right_line[i] < right_line[i - 3])
            {
                if (single_side_right_f2_y > 25)
                {
                    if (right_line[single_side_right_f2_y] - right_line[i] > 15)
                    {
                        single_side_right_f2_y2 = i;
                    }
                }
                else
                {
                    single_side_right_f2_y2 = i;
                }
                break;
            }
        }
        //找到单边桥右边的第三个拐点
        for (uint8_t i = top_road + 3; i < single_side_right_f2_y2; i++)
        {
            if ((right_line[i] > right_line[i - 3] && right_line[i] >= right_line[i + 1] && right_line[i + 1] >= right_line[i + 2] && right_line[i] - right_line[i + 1] < 7) ||
                (right_line[i + 1] - right_line[i] > 10 && right_line[i + 1] >= right_line[i] && right_line[i + 2] >= right_line[i + 1]))
            {
                for (uint8_t j = i; j < single_side_right_f2_y2; j++)
                {
                    if ((right_line[j + 1] - right_line[j] > 0 || right_line[j + 1] - right_line[j] < -5) && single_right_ok2 == 2)
                    {
                        single_side_right_f2_y3 = 0;
                        break;
                    }
                    if ((right_line[j + 1] - right_line[j] > 0 || right_line[j + 1] - right_line[j] < -5) && single_right_ok2 == 0)
                    {
                        single_right_ok2 = 1;
                    }
                    if (single_right_ok2 == 1 && right_line[j + 1] - right_line[j] < -7)
                    {
                        single_right_ok2 = 2;
                        single_side_right_f2_y3 = j + 1;
                    }
                    if (j == single_side_right_f2_y2 - 1 && single_right_ok2 == 0)
                    {
                        single_side_right_f2_y3 = i;
                    }
                    if (j == single_side_right_f2_y2 - 1 && single_right_ok2 == 1)
                    {
                        single_side_right_f2_y3 = 0;
                    }
                }
                break;
            }
        }
    }
    if (single_side_right_f2_y3 != 0 && single_side_right_f2_y2 - single_side_right_f2_y3 > 2)
    {

        left_y2 = (float)(right_line[single_side_right_f2_y3] - right_line[single_side_right_f2_y2]);
        left_k2 = atan2(left_y2, single_side_right_f2_y3 - single_side_right_f2_y2);
        left_y1 = (float)(right_line[single_side_right_f2_y] - right_line[single_side_right_f2_y2]);
        left_k1 = atan2(left_y1, single_side_right_f2_y - single_side_right_f2_y2);
        if (left_k2 - left_k1 < 2.4 && left_k2 - left_k1 > 1.05)
        {
            single_side_left_f2_y = top_road + 2;
        }
    }
    if (single_side_left_f2_y != 0 && single_side_right_f2_y == 0 && get_straight_line(right_line, 30, top_road + 2, 10) == 0)
    {
        //找到单边桥左边的第二个拐点、
        for (uint8_t i = single_side_left_f2_y - 5; i > top_road; i--)
        {
            if (left_line[i] >= left_line[i - 1] && left_line[i] >= left_line[i - 2] && left_line[i] > left_line[i - 3])
            {
                if (single_side_left_f2_y > 25)
                {
                    if (left_line[i] - left_line[single_side_left_f2_y] > 15)
                    {
                        single_side_left_f2_y2 = i;
                    }
                }
                else
                {
                    single_side_left_f2_y2 = i;
                }
                break;
            }
        }
        //找到单边桥左边的第三个拐点
        for (uint8_t i = top_road + 3; i < single_side_left_f2_y2; i++)
        {
            if ((left_line[i] < left_line[i - 3] && left_line[i] <= left_line[i + 1] && left_line[i + 1] <= left_line[i + 2] && left_line[i] - left_line[i + 1] > -7) ||
                (left_line[i + 1] - left_line[i] < -10 && left_line[i + 1] <= left_line[i] && left_line[i + 2] <= left_line[i + 1]))
            {
                for (uint8_t j = i; j < single_side_left_f2_y2; j++)
                {
                    if ((left_line[j + 1] - left_line[j] < 0 || left_line[j + 1] - left_line[j] > 5) && single_left_ok2 == 2)
                    {
                        single_side_left_f2_y3 = 0;
                        break;
                    }
                    if ((left_line[j + 1] - left_line[j] < 0 || left_line[j + 1] - left_line[j] > 5) && single_left_ok2 == 0)
                    {
                        single_left_ok2 = 1;
                    }
                    if (left_line[j + 1] - left_line[j] > 7 && single_left_ok2 == 1)
                    {
                        single_left_ok2 = 2;
                        single_side_left_f2_y3 = j + 1;
                    }
                    if (j == single_side_left_f2_y2 - 1 && single_left_ok2 == 0)
                    {
                        single_side_left_f2_y3 = i;
                    }
                    if (j == single_side_left_f2_y2 - 1 && single_left_ok2 == 1)
                    {
                        single_side_left_f2_y3 = 0;
                    }
                }
                break;
            }
        }
    }
    if (single_side_left_f2_y3 != 0 && single_side_left_f2_y2 - single_side_left_f2_y3 > 2)
    {
        right_y2 = (float)(left_line[single_side_left_f2_y3] - left_line[single_side_left_f2_y2]);
        right_k2 = atan2(right_y2, single_side_left_f2_y3 - single_side_left_f2_y2);
        right_y1 = (float)(left_line[single_side_left_f2_y] - left_line[single_side_left_f2_y2]);
        right_k1 = atan2(right_y1, single_side_left_f2_y - single_side_left_f2_y2);
        if (right_k2 - right_k1 > -2.4 && right_k2 - right_k1 < -1.05)
        {
            single_side_right_f2_y = top_road + 2;
        }
    }
    if (get_straight_line(right_line, 30, single_side_right_f2_y, 10) != 0)
    {
        single_side_right_f2_y = 0;
    }
    if (get_straight_line(left_line, 30, single_side_left_f2_y, 10) != 0)
    {
        single_side_left_f2_y = 0;
    }
    //如果左边还没找到横断桥拐点
    if (single_side_left_f2_y == 0 && single_side_right_f2_y != 0)
    {
        for (uint8_t i = single_side_right_f2_y; i > top_road; i--)
        {
            if (my_road[i].white_num > 1)
            {
                single_white_num++;
                if (single_white_num >= 3)
                {
                    break;
                }
            }
        }
        if (single_white_num >= 3)
        {
            for (uint8_t i = single_side_right_f2_y; i > top_road + 1; i--)
            {
                if (right_line[i] - right_line[i - 1] - left_line[i] + left_line[i - 1] < -10)
                {
                    break;
                }
                if (right_line[i] - right_line[i - 1] >= 15)
                {
                    if (get_straight_line(right_line, i - 5, top_road, 4) == 0)
                    {
                        single_side_left_f2_y = single_side_right_f2_y;
                        single_side_right_f2_y = single_side_left_f2_y - 6;
                    }
                    break;
                }
            }
            //防止多弯道判成单边桥,对top_road上方的白条子进行宽度和条数检测
            for (uint8_t i = top_road; i > FAR_LINE; i--)
            {
                if (my_road[i].white_num == 0)
                {
                    break;
                }
                if (my_road[i].white_num >= 2)
                {
                    white_number++;
                }
                for (uint8_t j = 1; j <= my_road[i].white_num; j++)
                {
                    if (my_road[i].connected[j].width > 10)
                    {
                        white_width_num++;
                    }
                }
            }
        }
        single_white_num = 0;
    }
    //如果右边还没找到右拐点
    if (single_side_right_f2_y == 0 && single_side_left_f2_y != 0)
    {
        for (uint8_t i = single_side_left_f2_y; i > top_road; i--)
        {
            if (my_road[i].white_num > 1)
            {
                single_white_num++;
            }
        }
        if (single_white_num >= 3)
        {
            for (uint8_t i = single_side_left_f2_y; i > top_road + 1; i--)
            {
                if (right_line[i] - right_line[i - 1] - left_line[i] + left_line[i - 1] < -10)
                {
                    break;
                }
                if (left_line[i] - left_line[i - 1] <= -15)
                {
                    if (get_straight_line(left_line, i - 1, top_road, 5) == 0)
                    {
                        single_side_right_f2_y = single_side_left_f2_y;
                        single_side_left_f2_y = single_side_right_f2_y - 6;
                    }
                    break;
                }
            }
            //防止多弯道判成单边桥,对top_road上方的白条子进行宽度和条数检测
            for (uint8_t i = top_road; i > FAR_LINE; i--)
            {
                if (my_road[i].white_num == 0)
                {
                    break;
                }
                if (my_road[i].white_num >= 2)
                {
                    white_number++;
                }
                for (uint8_t j = 1; j <= my_road[i].white_num; j++)
                {
                    if (my_road[i].connected[j].width > 10)
                    {
                        white_width_num++;
                    }
                }
            }
        }
        single_white_num = 0;
    }
    //贴边数
    for (uint8_t i = NEAR_LINE; i > top_road; i--)
    {
        if (right_line[i] > RIGHT_SIDE - 4)
        {
            right_side_num++;
        }
        if (left_line[i] < LEFT_SIDE + 4)
        {
            left_side_num++;
        }
    }

    if (((single_side_left_f2_y > 15 && single_side_right_f2_y > 15) || (single_side_left_f2_y > 23 || single_side_right_f2_y > 23) && (single_side_left_f2_y > 7 && single_side_right_f2_y > 7)) &&
        (single_side_left_f2_y - single_side_right_f2_y > 5 || single_side_left_f2_y - single_side_right_f2_y < -5) &&
        (single_left_side_y == 0 || single_right_side_y == 0) && right_line[top_road + 2] - left_line[top_road + 2] < 60
        && left_side_num < 20 && right_side_num < 20 && white_number < 11 && white_width_num < 10 &&
        roundabout_num >= single_round_d && cross_num >= single_cross_d && single_num < single_u && roadblock_num >= single_block_d)
    {
        single_side_fps++;
        if ((single_side_left_f2_y - single_side_right_f2_y) > 5 && single_side_fps == 3)
        {
            step = single_side;
            single_side_state = 1;
            single_side_fps = 0;
        }
        if ((single_side_left_f2_y - single_side_right_f2_y) < -5 && single_side_fps == 3)
        {
            step = single_side;
            single_side_state = 2;
            single_side_fps = 0;
        }
    }
    else
    {
        single_side_fps = 0;
    }
    /*printf("左拐点2 = %d,左拐点3 = %d, 左斜率1 = %f,左斜率2 = %f\n", single_side_left_f2_y2, single_side_left_f2_y3, right_k1, right_k2);
    printf("右拐点2 = %d,右拐点3 = %d, 右斜率1 = %f,右斜率2 = %f\n", single_side_right_f2_y2, single_side_right_f2_y3, left_k1, left_k2);*/
}
//单边桥判断2,直道判断单边桥
void single_side_bridge_judge2(uint8_t start)
{
    uint8_t single_right_guai = 0;
    uint8_t single_left_guai = 0;
    uint8_t single_left_ok = 0;
    uint8_t single_right_ok = 0;
    uint8_t singel_right_ok = 0;
    uint8_t ok1 = 0;
    uint8_t ok2 = 0;
    uint8_t ok3 = 0;
    uint8_t single_ok = 0;
    uint8_t single_white_num = 0;

    float black_duty = 0;
    for (uint8_t i = top_road; i < top_road + 10; i++)
    {
        if (my_road[i].white_num > 1)
        {
            single_white_num++;
        }
    }
    //白条子缩短，两边线均存在二阶导，并且两边线二阶导纵坐标距离大于某个值，小于某个值
    single_left_side_y = get_line_side(left_line, start, top_road, 10);
    single_right_side_y = get_line_side(right_line, start, top_road, 10);
    single_side_white_smller = get_white_width_small(start, top_road, 10);
    single_side_left_f2_y = start;
    single_side_right_f2_y = start;
    while (single_side_left_f2_y = get_line_f2_forward_y(left_line, single_side_left_f2_y, top_road + 2, 10, 3))
    {
        if (left_line[single_side_left_f2_y] - left_line[single_side_left_f2_y + 1] > -4 && left_line[single_side_left_f2_y] - left_line[single_side_left_f2_y + 1] < 3 && left_line[single_side_left_f2_y + 2] > LEFT_SIDE + 5 &&
            left_line[single_side_left_f2_y] - left_line[single_side_left_f2_y - 1] < -2 && 2 * left_line[single_side_left_f2_y] - left_line[single_side_left_f2_y + 3] - left_line[single_side_left_f2_y - 3] < -5)
        {
            if (left_line[single_side_left_f2_y] - left_line[single_side_left_f2_y - 1] < -2 && left_line[single_side_left_f2_y] - left_line[single_side_left_f2_y - 1] > -5)
            {
                if (left_line[single_side_left_f2_y - 1] - left_line[single_side_left_f2_y - 2] < -2 && left_line[single_side_left_f2_y - 2] - left_line[single_side_left_f2_y - 3] < -2)
                {
                    ok3 = 1;
                }
            }
            else
            {
                ok3 = 1;
            }
            if (ok3 == 1)
            {
                for (uint8_t i = single_side_left_f2_y; i > top_road + 3; i--)
                {
                    if (left_line[i] - left_line[i - 4] < 7 && left_line[i] - left_line[i - 4] > -7 && left_line[i] - left_line[i + 3] > 9 && left_line[i] > LEFT_SIDE + 5)
                    {
                        single_left_ok = 1;
                        break;
                    }
                }
                if (single_left_ok == 0)
                {
                    for (uint8_t i = single_side_left_f2_y; i > top_road + 1; i--)
                    {
                        if (left_line[i] - left_line[i - 2] > -2 && left_line[i] - left_line[i - 2] < 2 && left_line[i] - left_line[i + 3] > 6)
                        {
                            single_left_ok = 1;
                            break;
                        }
                    }
                }
                if (single_left_ok == 0)
                {
                    single_side_left_f2_y = 0;
                }
                break;
            }
        }
        if (left_line[single_side_left_f2_y] - left_line[single_side_left_f2_y - 3] > 20)
        {
            single_side_left_f2_y = 0;
            break;
        }
        else
        {
            single_side_left_f2_y = single_side_left_f2_y - 1;
        }
    }
    ok3 = 0;
    while (single_side_right_f2_y = get_line_f2_forward_y(right_line, single_side_right_f2_y, top_road + 2, 10, 3))
    {
        if (right_line[single_side_right_f2_y + 1] - right_line[single_side_right_f2_y] > -4 && right_line[single_side_right_f2_y + 1] - right_line[single_side_right_f2_y] < 3 && right_line[single_side_right_f2_y + 2] < RIGHT_SIDE - 5 &&
            right_line[single_side_right_f2_y - 1] - right_line[single_side_right_f2_y] < -2 && 2 * right_line[single_side_right_f2_y] - right_line[single_side_right_f2_y + 3] - right_line[single_side_right_f2_y - 3] > 5)
        {
            if (right_line[single_side_right_f2_y - 1] - right_line[single_side_right_f2_y] < -2 && right_line[single_side_right_f2_y - 1] - right_line[single_side_right_f2_y] > -5)
            {
                if (right_line[single_side_right_f2_y - 2] - right_line[single_side_right_f2_y - 1] < -2 && right_line[single_side_right_f2_y - 3] - right_line[single_side_right_f2_y - 2] < -2)
                {
                    ok3 = 1;
                }
            }
            else
            {
                ok3 = 1;
            }
            if (ok3 == 1)
            {
                for (uint8_t i = single_side_right_f2_y; i > top_road + 3; i--)
                {
                    if (right_line[i] - right_line[i - 4] < 7 && right_line[i] - right_line[i - 4] > -7 && right_line[i] - right_line[i + 3] < -10 && right_line[i] < RIGHT_SIDE - 5)
                    {
                        single_right_ok = 1;
                        break;
                    }
                }
                if (single_right_ok == 0)
                {
                    for (uint8_t i = single_side_right_f2_y; i > top_road + 1; i--)
                    {
                        if (right_line[i] - right_line[i - 2] < 2 && right_line[i] - right_line[i - 2] > -2 && right_line[i] - right_line[i + 3] < -6)
                        {
                            single_right_ok = 1;
                            break;
                        }
                    }
                }
                if (single_right_ok == 0)
                {
                    single_side_right_f2_y = 0;
                }
                break;
            }
        }
        ok3 = 0;
        if (right_line[single_side_right_f2_y] - right_line[single_side_right_f2_y - 3] < -20)
        {
            single_side_right_f2_y = 0;
            break;
        }
        else
        {
            single_side_right_f2_y = single_side_right_f2_y - 1;
        }
    }
    if (get_straight_line(right_line, NEAR_LINE - 30, single_side_right_f2_y, 5) != 0)
    {
        single_side_right_f2_y = 0;
    }
    if (get_straight_line(left_line, NEAR_LINE - 30, single_side_left_f2_y, 5) != 0)
    {
        single_side_left_f2_y = 0;
    }
    if (single_side_right_f2_y > 5)
    {
        for (int i = single_side_right_f2_y - 5; i > single_side_right_f2_y - 15; i--)
        {
            if (left_line[i] == MISS)
            {
                break;
            }
            else if (right_line[i] - right_line[i + 1] > 5 && right_line[i] > right_line[single_side_right_f2_y + 3] + 5)
            {
                ok1 = 1;
                break;
            }
        }
    }
    if (single_left_side_y > 5)
    {
        for (int i = single_side_left_f2_y - 5; i > single_side_left_f2_y - 15; i--)
        {
            if (left_line[i] == MISS)
            {
                break;
            }
            else if (left_line[i] - left_line[i + 1] < -5 && left_line[i] < left_line[single_side_left_f2_y + 3] + 5)
            {
                ok1 = 1;
                break;
            }
        }
    }

    //如果漫水法因为单边桥没有满上去
    if (single_side_right_f2_y >= 10 && single_side_left_f2_y == 0 && get_straight_line(left_line, NEAR_LINE - 50, top_road + 4, 10) == 0)
    {
        for (uint8_t i = single_side_right_f2_y - 3; i > top_road; i--)
        {
            if (right_line[i] - right_line[single_side_right_f2_y - 3] > 4 || right_line[i] - right_line[single_side_right_f2_y - 3] < -4)
            {
                single_ok = 1;
                break;
            }
        }
        if (single_ok == 1)
        {
            single_side_left_f2_y = 0;
        }
        else
        {
            single_side_left_f2_y = top_road;
        }
    }
    else if (single_side_left_f2_y >= 10 && single_side_right_f2_y == 0 && get_straight_line(right_line, NEAR_LINE - 50, top_road + 4, 10) == 0)
    {
        for (uint8_t i = single_side_left_f2_y - 3; i > top_road; i--)
        {
            if (left_line[i] - left_line[single_side_left_f2_y - 3] > 4 || left_line[i] - left_line[single_side_left_f2_y - 3] < -4)
            {
                single_ok = 1;
                break;
            }
        }
        if (single_ok == 1)
        {
            single_side_right_f2_y = 0;
        }
        else
        {
            single_side_right_f2_y = top_road;
        }
    }
    //防止避障误判成单边桥，增加判据
    //利用单边桥占赛道的百分比
    if (single_side_left_f2_y != 0 && single_side_right_f2_y != 0)
    {
        //右避障
        if (single_side_right_f2_y > single_side_left_f2_y)
        {
            for (uint8_t i = single_side_right_f2_y; i > top_road + 1; i--)
            {
                if (right_line[i] <= right_line[i - 1] && right_line[i] <= right_line[i + 1] &&
                    right_line[i] <= right_line[i - 2] && right_line[i] <= right_line[i + 2])
                {
                    single_right_guai = i;
                    break;
                }
            }
            black_duty = (float)(right_line[single_side_right_f2_y] - right_line[single_right_guai]) / (right_line[single_side_right_f2_y] - left_line[single_side_right_f2_y]);
        }
        //左避障
        if (single_side_left_f2_y > single_side_right_f2_y)
        {
            for (uint8_t i = single_side_left_f2_y; i > top_road + 1; i--)
            {
                if (left_line[i] >= left_line[i - 1] && left_line[i] >= left_line[i + 1] &&
                    left_line[i] >= left_line[i - 2] && left_line[i] >= left_line[i + 2])
                {
                    single_left_guai = i;
                    break;
                }
            }
            black_duty = (float)(left_line[single_left_guai] - left_line[single_side_left_f2_y]) / (right_line[single_side_left_f2_y] - left_line[single_side_left_f2_y]);
        }
    }
    if (single_side_right_f2_y != 0 && single_side_left_f2_y != 0)
    {
        if (single_side_left_f2_y < 20 || single_side_right_f2_y < 20)
        {
            if ((single_side_right_f2_y - single_side_left_f2_y) > 8 || (single_side_right_f2_y - single_side_left_f2_y) < -8)
            {
                ok2 = 1;
            }
        }
    }

    if (single_side_white_smller >= 6 && single_side_left_f2_y >= 5 && single_side_right_f2_y >= 5 &&
        single_left_side_y == 0 && single_right_side_y == 0 && ok1 == 0 && ok2 == 0 && black_duty > 0.38 &&
        ((single_side_left_f2_y - single_side_right_f2_y) >= 4 || (single_side_left_f2_y - single_side_right_f2_y) <= -4) &&
        roundabout_num >= single_round_d && cross_num >= single_cross_d && single_num < single_u)
    {
        if (step == cross)
        {
            yuansu_state = single_side;
        }
        else
        {
            step = single_side;
        }
        if ((single_side_left_f2_y - single_side_right_f2_y) >= 4)
        {
            single_side_state = 1;
        }
        if ((single_side_left_f2_y - single_side_right_f2_y) <= -4)
        {
            single_side_state = 2;
        }
    }
    //printf("ok2 = %d\n", ok2);
    //printf("single_side_white_smller = %d,single_side_left_f2_y = %d,single_side_right_f2_y = %d\nsingle_left_side_y = %d,single_right_side_y = %d,ok1 = %d, road_max = %d,black_duty = %f\n", 
    //    single_side_white_smller, single_side_left_f2_y, single_side_right_f2_y, single_left_side_y, single_right_side_y, ok1 ,road_max,black_duty);
}

//判断弯道，进入弯道状态机
void bend_judge(void)
{
    uint8_t bend_start = 0;
    uint8_t bend_num = 0;
    switch (bend_state)
    {
    case 0: {
        //判断是否为弯道
        //右转
        for (uint8_t i = NEAR_LINE; i > 4; i -= 4)
        {
            if (mid_line[i - 5] - mid_line[i] > 3 || mid_line[i] > RIGHT_SIDE - 5)
            {
                bend_start = i;
                bend_num++;
            }
            else
            {
                bend_start = 0;
                bend_num = 0;
            }
        }
        //左转
        if (bend_num < 3)
        {
            for (uint8_t i = NEAR_LINE; i > 4; i -= 4)
            {
                if (mid_line[i - 5] - mid_line[i] < -3 || mid_line[i] < LEFT_SIDE + 5)
                {
                    bend_start = i;
                    bend_num++;
                }
                else
                {
                    bend_start = 0;
                    bend_num = 0;
                }
            }
        }
        if (bend_num > 3)
        {
            bend_fps++;
        }
        else
        {
            bend_fps = 0;
        }
        if (bend_fps > 3)
        {
            bend_state = 1;
            bend_fps = 0;
        }
        else
        {
            if (roundabout_state == 2)
            {
                bend_state = 1;
                bend_fps = 1;
            }
        }
    }break;
    case 1: {
        uint8_t bend_ok = 0;
        for (uint8_t i = NEAR_LINE; i > 5; i -= 5)
        {
            if (mid_line[i] > RIGHT_SIDE - 20 || mid_line[i] < LEFT_SIDE + 20)
            {
                bend_ok = 1;
            }
        }
        if (bend_ok == 0)
        {
            if (get_straight_line(mid_line, NEAR_LINE - 20, 1, 7) == 0)
            {
                if (mid_line[20] > 54 && mid_line[20] < 134)
                {
                    bend_fps++;
                }
                else
                {
                    bend_fps = 0;;
                }
            }
            else
            {
                bend_fps = 0;
            }
        }
        if (bend_fps > 2)
        {
            bend_state = 0;
            bend_fps = 0;
        }
        else if ((roundabout_state <= 7 && (step == 3 || step == 4)))
        {
            bend_state = 1;
            bend_fps = 0;
        }
    }
          break;
    }
}
//总体判断
void state_judge()
{
    switch (yuansu_state)
    {
    case single_side: {
        if (step == 0)
        {
            step = single_side;
            yuansu_state = 0;
        }
    }break;
    }
    switch (step)
    {
    case strai:
    {
        if (element_ok == 0)
        {
            if (single_start_fps < single_start_fps_up)
            {
                single_start_fps++;
            }
            else
            {
                single_start_fps = single_start_fps_up;
            }
            single_side_bridge_judge2(NEAR_LINE - 20);
            if (step != single_side)
            {
                single_side_bridge_judge(50, top_road + 5);
            }
        }
        else
        {
            single_side_bridge_judge2(NEAR_LINE - 20);
            if (step != single_side && element_sort % 10 == 1)
            {
                single_side_bridge_judge(50, top_road + 5);
            }
        }
    }break;
    }
}
////////////////////////////////////////////
//功能：判断右边是否为最后一个单边桥
//输入：
//输出：如果是最后一个单边桥，输出右拐点为零，如果不是最后一个单边桥，输出右拐点
//备注：
///////////////////////////////////////////
uint8_t last_one_right_single_check(void)
{
    uint8_t right_x = NEAR_LINE - 10;
    while (1)
    {
        right_x = get_line_f2_forward_y(right_line, right_x, top_road + 10, 7, 5);
        if (right_x == 0)
        {
            break;
        }
        if ((right_line[right_x - 6] - right_line[right_x - 9]) > -4 && right_line[right_x] > right_line[right_x - 6] &&
            (right_line[right_x - 6] - right_line[right_x - 9]) < 4 && right_line[right_x] > 94 && right_line[right_x - 5] > 74 &&
            (right_line[right_x] - left_line[right_x] - right_line[right_x - 8] + left_line[right_x - 8]) > 13 && right_line[right_x] - right_line[right_x - 6] > 7
            && (right_line[right_x] - left_line[right_x] - right_line[right_x - 8] + left_line[right_x - 8]) < 60)
        {
            break;
        }
        else
        {
            right_x = right_x - 1;
        }
    }
    return right_x;
}
////////////////////////////////////////////
//功能：判断左边是否为最后一个单边桥
//输入：
//输出：如果是最后一个单边桥，输出左拐点为零，如果不是最后一个单边桥，输出左拐点
//备注：
///////////////////////////////////////////
uint8_t last_one_left_single_check(void)
{
    uint8_t left_x = NEAR_LINE - 10;
    while (left_x = get_line_f2_forward_y(left_line, left_x, top_road + 5, 7, 5))
    {
        if ((left_line[left_x - 6] - left_line[left_x - 9]) > -4 &&
            (left_line[left_x - 6] - left_line[left_x - 9]) < 4 && left_line[left_x] < 94 && left_line[left_x - 5] < 114 &&
            (right_line[left_x] - left_line[left_x] - right_line[left_x - 8] + left_line[left_x - 8]) > 13 && left_line[left_x - 6] - left_line[left_x] > 7
            && (right_line[left_x] - left_line[left_x] - right_line[left_x - 8] + left_line[left_x - 8]) < 60)
        {
            break;
        }
        else
        {
            left_x = left_x - 1;
        }
    }
    return left_x;
}
//计算白条子变多的函数
uint8_t get_white_num_more_y(uint8_t start, uint8_t end)
{
    uint8_t zebra_start = 0;
    for (int i = start; i > end; i--)
    {
        if (my_road[i].white_num > 1 && my_road[i - 1].white_num > 1 && my_road[i - 2].white_num > 1 && my_road[i - 3].white_num > 1)
        {
            zebra_start = (uint8_t)i;
            break;
        }
    }
    return zebra_start;
}
//计算白条子变少的函数
uint8_t get_white_num_less_y(uint8_t start, uint8_t end, uint8_t white_num)
{
    uint8_t zebra_end = 0;
    if (start == 0)
    {
        start = NEAR_LINE;
    }
    for (int i = start; i > end; i--)
    {
        if (my_road[i].white_num <= 1)
        {
            zebra_end = (uint8_t)i;
            break;
        }
        if (my_road[i].white_num == 2 && (my_road[i].connected[2].left - my_road[i].connected[1].right) > 20)
        {
            zebra_end = (uint8_t)i;
            break;
        }
    }
    return zebra_end;
}
//计算白条子最多的y坐标
uint8_t get_white_num_most_y(uint8_t start, uint8_t end)
{
    uint8_t white_max_y = 0;
    uint8_t white_num_1 = 0;
    for (int i = start; i > end; i--)
    {
        if (my_road[i].white_num > white_num_1)
        {
            white_max_y = (uint8_t)i;
            white_num_1 = my_road[i].white_num;
        }
    }
    return white_max_y;
}
////////////////////////////////////////////
//功能：计算边线突变点
//输入：line 边界线,start 开始点,end 结束点,change_limit 改变限制
//输出：边线突变点的y坐标
//备注：
///////////////////////////////////////////
uint8_t get_line_change(uint8_t* line, uint8_t start, uint8_t end, uint8_t change_limit)
{
    if (top_road > end)
    {
        end = top_road;
    }
    uint8_t line_change = 0;
    for (int i = start; i > end; i--)
    {
        if ((line[i] - line[i - 1]) > change_limit || (line[i] - line[i - 1]) < -change_limit)
        {
            line_change = (uint8_t)i;
            break;
        }
    }
    return line_change;
}
////////////////////////////////////////////
//功能：计算边线最大二阶导
//输入：line 边界线,start 开始点,end 结束点,change_limit 改变限制
//输出：边线突变点的y坐标
//备注：注意start不能大于NEAR_LINE - 5，end不能小于top_road + 5;
///////////////////////////////////////////
uint8_t get_line_f2_max_y(uint8_t* line, uint8_t start, uint8_t end)
{
    uint8_t line_f2_max_y = 0;
    int8_t line_f2_max = 0;
    for (int i = start; i > end; i--)
    {

        int8_t line_f2 = 2 * line[i] - line[i - 5] - line[i + 5];
        if (i == top_road)
        {
            break;
        }
        if (line_f2 > line_f2_max || line_f2 < -line_f2_max)
        {
            line_f2_max = line_f2;
            if (line_f2_max < 0)
            {
                line_f2_max = 0 - line_f2_max;
            }
            line_f2_max_y = (uint8_t)i;
        }
    }
    if (line_f2_max < 4)
    {
        line_f2_max_y = 0;
    }
    return line_f2_max_y;
}
////////////////////////////////////////////
//功能：计算边线二阶导大于change_limit的值
//输入：line 边界线,start 开始点,end 结束点,change_limit 改变限制
//输出：边线突变点的y坐标
//备注：注意start不能大于NEAR_LINE - 5，end不能小于top_road + 5;
///////////////////////////////////////////
uint8_t get_line_f2_forward_y(uint8_t* line, uint8_t start, uint8_t end, uint8_t change_limit, uint8_t dis)
{
    uint8_t line_y = 0;

    for (int i = start; i > end; i--)
    {
        if ((2 * line[i] - line[i - dis] - line[i + dis]) > change_limit || (2 * line[i] - line[i - dis] - line[i + dis]) < -change_limit)
        {
            if (start < 0 || start > 200)
            {
                break;
            }
            line_y = (uint8_t)i;
            break;
        }
    }
    return line_y;
}
////////////////////////////////////////////
//功能：从上到下巡线计算边线二阶导大于change_limit的值
//输入：line 边界线,start 开始点,end 结束点,change_limit 改变限制
//输出：边线突变点的y坐标
//备注：注意start不能大于NEAR_LINE - 5，end不能小于top_road + 5;
///////////////////////////////////////////
uint8_t get_line_f2_forward_y_down(uint8_t* line, uint8_t start, uint8_t end, uint8_t change_limit)
{
    uint8_t line_y = 0;
    for (int i = start; i < end; i++)
    {
        if ((2 * line[i] - line[i - 5] - line[i + 5]) > change_limit || (2 * line[i] - line[i - 5] - line[i + 5]) < -change_limit)
        {
            line_y = (uint8_t)i;
            break;
        }
    }
    return line_y;
}
////////////////////////////////////////////
//功能：计算白条子突然变小的y坐标
//输入：start 开始点,end 结束点,change_limit 改变限制
//输出：边线突变点的y坐标
//备注：注意start不能大于NEAR_LINE - 5，end不能小于top_road + 5;
///////////////////////////////////////////
uint8_t get_white_width_small(uint8_t start, uint8_t end, uint8_t change_limit)
{
    uint8_t width_shorten_y = 0;
    for (int i = start; i > end; i--)
    {
        if ((right_line[i] - left_line[i] - right_line[i - 1] + left_line[i - 1]) > change_limit)
        {
            int16_t width_1 = right_line[i - 1] - left_line[i - 1] - right_line[i - 2] + left_line[i - 2];
            if (width_1 - (right_line[i] - left_line[i] - right_line[i - 1] + left_line[i - 1]) < 10)
            {
                width_shorten_y = i - 1;
                break;
            }
        }
    }
    return width_shorten_y;
}
////////////////////////////////////////////
//功能：计算白条子突然变大的y坐标
//输入：start 开始点,end 结束点,change_limit 改变限制
//输出：边线突变点的y坐标
//备注：注意start不能大于NEAR_LINE - 5，end不能小于top_road + 5;
///////////////////////////////////////////
uint8_t get_white_width_larger(uint8_t start, uint8_t end, uint8_t change_limit)
{
    uint8_t width_larger_y = 0;
    for (int i = start; i > end; i--)
    {
        if ((right_line[i - 1] - left_line[i - 1] - right_line[i] + left_line[i]) >= change_limit)
        {
            width_larger_y = (uint8_t)i;
            break;
        }
    }
    return width_larger_y;
}
////////////////////////////////////////////
//功能：计算边线靠边的点
//输入：start 开始点,end 结束点
//输出：边线靠边的y坐标
//备注：end不能小于FAR_LINE + 1
///////////////////////////////////////////
uint8_t get_line_side(uint8_t* line, uint8_t start, uint8_t end, uint8_t limit)
{
    uint8_t side_y = 0;
    for (int i = start; i > end; i--)
    {
        if (i == top_road)
        {
            break;
        }
        if (line[i] >= RIGHT_SIDE - limit || line[i] <= LEFT_SIDE + limit)
        {
            if (line[i] == line[i - 1] || (line[i - 2] <= LEFT_SIDE + limit || line[i - 2] >= RIGHT_SIDE - limit))
            {
                side_y = (uint8_t)i + 1;
                break;
            }
        }
    }
    return side_y;
}
////////////////////////////////////////////
//功能：判断直线严格直
//输入：边线，开始点，结束点，严格值的范围
//输出：第一个不直的点
//备注：
///////////////////////////////////////////
uint8_t get_straight_line(uint8_t* line, uint8_t start, uint8_t end, uint8_t change)
{
    float line_datas[2] = { 0 };
    uint8_t f2_y = 0;
    get_line_b_and_k(line, start, end, line_datas);
    for (int i = start; i > end; i--)
    {
        uint8_t line_x = (uint8_t)(line_datas[0] * i + line_datas[1]);
        if ((line_x - line[i]) >= change || (line_x - line[i]) <= -change)
        {
            f2_y = (uint8_t)i;
            break;
        }
    }
    return f2_y;
}
////////////////////////////////////////////
//功能：判断直线的单调性
//输入：边线，开始点，结束点，判断为单调性的范围
//输出：第一个不直的点
//备注：
///////////////////////////////////////////
int8_t get_line_monotony(uint8_t* line, uint8_t start, uint8_t end, uint8_t change)
{
    int8_t line_sign = 0;//给线递增或递减的信号
    uint8_t line_increase = 0;//线递增
    uint8_t line_decrease = 0;//线递减
    for (uint8_t i = start; i > end; i--)
    {
        if (line[i] - line[i - 5] > change)
        {
            line_decrease = i;
            for (uint8_t j = i; j > end; j--)
            {
                if (line[j] - line[j - 5] < -1)
                {
                    line_increase = j;
                    break;
                }
            }
            break;
        }
        else if (line[i] - line[i - 5] < -change)
        {
            line_increase = i;
            for (uint8_t j = i; j > end; j--)
            {
                if (line[j] - line[j - 5] > 1)
                {
                    line_decrease = j;
                    break;
                }
            }
            break;
        }
    }
    if (line_decrease == 0 && line_increase != 0)
    {
        line_sign = 1;
    }
    else if (line_increase == 0 && line_decrease != 0)
    {
        line_sign = -1;
    }
    else
    {
        line_sign = line_increase - line_decrease;
    }
    return line_sign;
    //如果line_sign大于0，说明线先递增后递减，如果line_sign小于0，说明线先递减后递增，
}
////////////////////////////////////////////
//功能：坡道判断之判断赛道宽度
//输入：
//输出：
//备注：
///////////////////////////////////////////
uint8_t judge_white_width_charge(uint8_t y1, uint8_t y2, uint8_t charge)
{
    uint8_t answer = 0;
    uint8_t width_1 = right_line[y1] - left_line[y1];
    uint8_t width_2 = right_line[y2] - left_line[y2];
    if ((width_1 - width_2) < charge)
    {
        answer = 1;
    }
    return answer;
}

////////////////////////////////////////////
//功能：计算左边线x坐标最大值所对应的y坐标
//输入：start 开始计算点,end 结束计算点,change_limit 改变限制
//输出：左边线x坐标最大值所对应的y坐标
//备注：
///////////////////////////////////////////
uint8_t get_left_line_x_max_y(uint8_t* line, uint8_t start, uint8_t end)
{
    uint8_t line_x_max_y = 0;
    int8_t dis = 0;
    int8_t dis_i1 = 0;
    int8_t dis_i2 = 0;
    for (int i = start; i > end; i--)
    {
        if ((line[i] - line[start]) >= dis)
        {
            dis = line[i] - line[start];
            dis_i1 = line[i - 1] - line[start];
            dis_i2 = line[i - 2] - line[start];
            if (dis_i1 < dis)
            {
                if (dis_i1 >= dis_i2 || (left_line[i - 1] < LEFT_SIDE + 15 && left_line[i - 2] < LEFT_SIDE + 15) || left_line[i] - left_line[i - 1] > 20)
                {
                    line_x_max_y = (uint8_t)i;
                    break;
                }
                //做一个防抖
                else
                {

                }

            }
        }
        if (line[i] == MISS)
        {
            line_x_max_y = top_road;
            break;
        }
    }
    return line_x_max_y;
}
////////////////////////////////////////////
//功能：计算右边线x坐标最小值所对应的y坐标
//输入：start 开始计算点,end 结束计算点,change_limit 改变限制
//输出：右边线x坐标最小值所对应的y坐标
//备注：
///////////////////////////////////////////
uint8_t get_right_line_x_min_y(uint8_t* line, uint8_t start, uint8_t end)
{
    uint8_t line_x_min_y = 0;
    int8_t dis = 0;
    int8_t dis_i1 = 0;
    int8_t dis_i2 = 0;
    for (int i = start; i > end; i--)
    {
        if ((line[start] - line[i]) >= dis)
        {
            dis = line[start] - line[i];
            dis_i1 = line[start] - line[i - 1];
            dis_i2 = line[start] - line[i - 3];
            if (dis_i1 < dis)
            {
                //做一个防抖
                if (dis_i1 >= dis_i2 || (right_line[i - 1] > RIGHT_SIDE - 15 && right_line[i - 2] > RIGHT_SIDE - 15) || right_line[i - 1] - right_line[i] > 20)
                {
                    line_x_min_y = (uint8_t)i;
                    break;

                }
                else
                {

                }
            }
        }
    }
    return line_x_min_y;
}
////////////////////////////////////////////
//功能：用两点法求直线的截距和斜率
//输入：left 所选边线，start 开始计算点,end 结束计算点,k 斜率，b 斜率
//输出：直线的截距b
//备注：
///////////////////////////////////////////
void get_line_b_and_k(uint8_t* line, uint8_t start, uint8_t end, float input[])
{
    input[0] = (float)(line[start] - line[end]) / (start - end);
    input[1] = (float)line[start] - input[0] * start;
}
////////////////////////////////////////////
//功能：用斜率求直线的截距
//输入：left 所选边线，start 开始计算点,
//输出：直线的截距b
//备注：
///////////////////////////////////////////
void get_line_b(uint8_t* line, uint8_t x, float input[])
{
    input[1] = (float)line[x] - input[0] * x;
}
////////////////////////////////////////////
//功能：有时最小二乘法取得点的数量过多导致取到了255的点，故增加一函数来求其范围
//输入：line边线， outset输入拐点， change改变范围
//输出：二乘法所取的点数
//备注：
///////////////////////////////////////////
uint8_t get_least_square_limit(uint8_t* line, uint8_t outset, uint8_t end, uint8_t change, uint8_t start)
{
    uint8_t limit = 0;
    if ((outset + change) > end)
    {
        limit = NEAR_LINE - outset;
    }
    if ((outset + change) <= end)
    {
        limit = change;
    }
    if (outset - change <= start)
    {
        limit = outset - top_road;
    }
    return limit;
}

//利用最小二乘法来算中线斜率来补线
void least_square(uint8_t* line, uint8_t outset, uint8_t limit, float input[])
{
    float k;//最小二乘法算出来的斜率
    float b;//最小二乘法算出来的截距
    uint32_t Sum_xy = 0;//线的xy坐标的乘积之和
    uint32_t Sum_x = 0;//每个点的x的平方之和
    float average_x = 0;//横坐标平均值
    float average_y = 0;//纵坐标平均值
    if (limit < 3)
    {
        input[0] = 0;
        input[1] = line[NEAR_LINE];
    }
    else
    {
        for (int i = outset; i > outset - limit; i--) {
            Sum_xy += i * line[i];
            Sum_x += i * i;
            average_y += (float)line[i];
            average_x += (float)i;
        }
        average_y = average_y / limit;
        average_x = average_x / limit;
        k = (float)(Sum_xy - limit * average_x * average_y) / (Sum_x - limit * average_x * average_x);
        b = average_y - k * average_x;
        input[0] = k;
        input[1] = b;
    }
}
//画线
void get_line(uint8_t* line, uint8_t start, uint8_t end, float k, float b)
{
    int16_t line_now[NEAR_LINE + 1];
    for (int i = start; i >= end; i--) {
        line_now[i] = (int16_t)(b + k * i);
        if (line_now[i] < LEFT_SIDE) {
            line_now[i] = LEFT_SIDE;
        }
        if (line_now[i] > RIGHT_SIDE) {
            line_now[i] = RIGHT_SIDE;
        }
    }
    for (int i = start; i >= end; i--)
    {
        line[i] = (uint8_t)line_now[i];
    }
}
void least_square_get_line(uint8_t* line, uint8_t outset, uint8_t limit, float input[], uint8_t start, uint8_t end)
{
    least_square(line, outset, limit, input);
    get_line(line, start, end, input[0], input[1]);
}

//两点补线
void two_dots_line(uint8_t dots1_H, uint8_t dots1_W, uint8_t dots2_H, uint8_t dots2_W, uint8_t* line)
{
    int16_t line_now[NEAR_LINE + 1];
    float k = (float)(dots2_W - dots1_W) / (dots2_H - dots1_H);//斜率
    float intercept = (float)(dots1_W - k * dots1_H);//截距
    for (int i = NEAR_LINE; i > 0; i--)
    {
        line_now[i] = (int)(k * i + intercept);
        if (line_now[i] < LEFT_SIDE)
        {
            line_now[i] = LEFT_SIDE;
        }
        if (line_now[i] > RIGHT_SIDE)
        {
            line_now[i] = RIGHT_SIDE;
        }
        line[i] = (uint8_t)line_now[i];
    }
}



////////////////////////////////////////////
//功能：讲道路重新进行选择
//输入：
//输出：
//备注：
///////////////////////////////////////////
void renew_road()
{
    uint8_t biaozhi = (right_line[NEAR_LINE] + left_line[NEAR_LINE]) / 2;
    uint8_t j_now = 0;
    uint8_t j_continue = 0;
    uint8_t judge_middle = 0;
    uint8_t judge_dis = 0;
    uint8_t width_limit = 0;
    uint8_t dis = 0;//离标志位的最近距离
    //取NEAR_LINE一行的白条子编号
    for (uint8_t i = 1; i <= my_road[NEAR_LINE].white_num; i++)
    {
        if (left_line[NEAR_LINE] == my_road[NEAR_LINE].connected[i].left)
        {
            j_continue = i;
            break;
        }
    }

    for (uint8_t i = NEAR_LINE - 1; i > 0; i--)
    {
        judge_middle = 0;
        judge_dis = 0;
        dis = 255;
        if (my_road[i].white_num >= 5)
        {
            width_limit = 0;
        }
        else
        {
            width_limit = 10;
        }
        for (uint8_t j = 1; j <= my_road[i].white_num; j++)
        {
            if (j_continue == MISS)
            {
                break;
            }
            //首先得让上下两个白条子相邻
            if ((my_road[i].connected[j].left <= my_road[i + 1].connected[j_continue].right) &&
                (my_road[i].connected[j].right >= my_road[i + 1].connected[j_continue].left))
            {
                //其次得让白条子在中间
                if (my_road[i].connected[j].left <= biaozhi && my_road[i].connected[j].right >= biaozhi && (my_road[i].connected[j].width) >= width_limit)
                {
                    j_now = j;
                    judge_middle = 1;
                    break;
                }
                //如果白条子不在中间的话，就找离标志最近的
                if (judge_middle == 0 && abs((my_road[i].connected[j].right + my_road[i].connected[j].left) / 2 - biaozhi) < dis && (my_road[i].connected[j].width) >= width_limit)
                {
                    j_now = j;
                    dis = abs((my_road[i].connected[j].right + my_road[i].connected[j].left) / 2 - biaozhi);
                    judge_dis = 1;
                }
            }
        }
        if (judge_dis == 0 && judge_middle == 0)
        {
            j_now = MISS;
            j_continue = MISS;
        }
        if (j_now <= my_road[i].white_num)
        {
            left_line[i] = my_road[i].connected[j_now].left;
            right_line[i] = my_road[i].connected[j_now].right;
            j_continue = j_now;
        }
        else
        {
            j_now = MISS;
            j_continue = MISS;
            left_line[i] = MISS;
            right_line[i] = MISS;
        }
    }
    for (uint8_t i = FAR_LINE; i <= NEAR_LINE; i++)
    {
        if (right_line[i] != MISS)
        {
            top_road = i;
            break;
        }
    }
}

void single_side_bridge_two_line(uint8_t* road_continue)
{
    uint8_t white_length;//记录白色的长度
    uint8_t white_length_m = 0;;
    uint8_t mid_x;//白条子中点x坐标
    uint8_t mid_y;//白条子中点y坐标
    uint8_t j_con;
    for (uint8_t i = NEAR_LINE - 1; i > FAR_LINE; i--)
    {
        white_length_m = 0;
        mid_x = 0;
        mid_y = 0;
        j_con = MISS;
        if (road_continue[i + 1] > my_road[i + 1].white_num)
        {
            road_continue[i] = MISS;
        }
        else
        {
            for (uint8_t j = 1; j <= my_road[i].white_num; j++)
            {
                white_length = 0;
                if ((my_road[i].connected[j].left <= my_road[i + 1].connected[road_continue[i + 1]].right) &&
                    (my_road[i].connected[j].right >= my_road[i + 1].connected[road_continue[i + 1]].left))
                {
                    if (my_road[i].white_num == 1)
                    {
                        j_con = j;
                        break;
                    }
                    mid_x = (my_road[i].connected[j].left + my_road[i].connected[j].right) / 2;
                    mid_y = i;
                    for (uint8_t k = mid_y; k > FAR_LINE; k--)
                    {
                        if (IMG[k][mid_x] == white)
                        {
                            white_length++;
                        }
                        if (IMG[k][mid_x] == black)
                        {
                            break;
                        }
                    }
                    if (white_length >= white_length_m)
                    {
                        j_con = j;
                        white_length_m = white_length;
                    }
                }
            }
            road_continue[i] = j_con;
        }
    }
}
void single_side_road_choose(void)
{
    uint8_t istart = NEAR_LINE;
    uint8_t iend = FAR_LINE;
    //top_road = NEAR_LINE;//赛道最高处所在行数，先初始化话为最低处
    int road_f = -1;//赛道所在连通域父节点编号，先初始化为-1，以判断是否找到赛道
    int while_range_num = 0, roud_while_range_num = 0;
    all_range* twhite_range = NULL;
    road* tmy_road = NULL;
    uint8_t mid_x = 0, mid_y = 0;
    uint8_t white_length_m = 0, white_length = 0;
    uint8_t choose_i = 0;
    //寻找赛道所在连通域
    // 寻找最中心的白条子
    for (int i = 1; i <= white_range[istart].num; i++)
    {
        if (white_range[istart].area[i].right - white_range[istart].area[i].left > 30)
        {
            mid_x = (white_range[istart].area[i].left + white_range[istart].area[i].right) / 2;
            white_length = 0;
            for (uint8_t j = istart; j > FAR_LINE; j--)
            {
                if (IMG[j][mid_x] == white)
                {
                    white_length++;
                }
                else if (IMG[j][mid_x] == black)
                {
                    break;
                }
            }
            if (white_length > white_length_m)
            {
                choose_i = i;
                white_length_m = white_length;
            }
        }
    }
    road_f = find_f(white_range[istart].area[choose_i].connect_num);
    //现在我们已经得到了赛道所在连通域父节点编号，接下来把所有父节点编号是road_f的所有白条子扔进赛道数组就行了
    for (int i = istart; i >= iend; i--)
    {
        //变量保存，避免之后写的冗杂且低效
        twhite_range = &white_range[i];
        tmy_road = &my_road[i];
        while_range_num = twhite_range->num;//每行白条数量
        tmy_road->white_num = 0;
        roud_while_range_num = 0;
        for (int j = 1; j <= while_range_num; j++)
        {
            if (find_f(twhite_range->area[j].connect_num) == road_f)
            {
                tmy_road->white_num++; roud_while_range_num++;
                tmy_road->connected[roud_while_range_num].left = twhite_range->area[j].left;
                tmy_road->connected[roud_while_range_num].right = twhite_range->area[j].right;
                tmy_road->connected[roud_while_range_num].width = twhite_range->area[j].right - twhite_range->area[j].left;
            }
        }
    }
}
////////////////////////////////////////////
//功能：二值化
//输入：灰度图片
//输出：二值化图片
//备注：
///////////////////////////////////////////
void THRE()
{
    uint8_t* map;
    uint8_t* my_map;
    uint8_t* gray_map = 0;
    uint8_t bound_thre = 0;
    map = fullBuffer;
    for (int i = 0; i < 120; i++)
    {
        my_map = &IMG[i][0];
        for (int j = 0; j < 188; j++)
        {
            if ((*map) > threshold)
                (*my_map) = 1;
            else (*my_map) = 0;
            map++;
            my_map++;
        }
    }
}


////////////////////////////////////////////
//功能：清车头
//输入：
//输出：
//备注：要根据自己车头的大小进行修改
///////////////////////////////////////////
void head_clear(void)//分为两部分进行清理，车头处有线挡着
{
    uint8_t* my_map;
    for (int i = 119; i >= NEAR_LINE + 1; i--)//清除底部
    {
        my_map = &IMG[i][0];
        for (int j = 0; j <= 187; j++)
        {
            *(my_map + j) = black;
        }
    }

}
road_range search_white_range_begin(uint8_t line, uint8_t col)//用来找到line行，从col列开始往右的白条子，找到就返回这个白条子的属性road_range
{
    int16_t j;
    uint8_t* map = &IMG[line][col];
    uint8_t* Initial = &IMG[0][0];
    road_range result = { MISS, MISS, MISS };
    for (j = col; j <= RIGHT_SIDE; j++, map++)
    {
        if (j == LEFT_SIDE && (*map))//白条位于左边界
        {
            result.left = LEFT_SIDE;
            //printf("left0=%d\n", result.left);
            //开始向后一个一个像素点找这个白条右边界
            map++;
            j++;
            while ((*map) && j <= RIGHT_SIDE)
            {
                map++;
                j++;
            }
            result.right = j - 1;
            //printf("right0=%d\n", result.right);
            result.width = result.right - result.left + 1;
            return result;//找到就返回这个白条子的属性road_range
        }
        else if ((*map) && j == col)//如果开始处为白，先找到白条的左边界
        {
            //开始向前一个一个像素点找这个白条左边界
            map--;
            j--;
            while ((*map) && j >= LEFT_SIDE)
            {
                if (map != Initial) map--;
                j--;
            }
            result.left = j + 1;//找到了白条的左边界
            //printf("left1=%d\n", result.left);
            map = &IMG[line][col];
            j = col;
            //开始向后一个一个像素点找这个白条右边界
            map++;
            j++;
            while ((*map) && j <= RIGHT_SIDE)
            {
                map++;
                j++;
            }
            result.right = j - 1;
            result.width = result.right - result.left + 1;
            //printf("right1=%d\n", result.right);
            return result;//找到就返回这个白条子的属性road_range

        }
        else if ((*map))//如果开始处为黑，直接找白条的右边界
        {
            result.left = (uint8_t)j;
            if (line == 57)
            {
                // printf("left2=%d\n", result.left);
                // printf("col=%d\n", col);
            }

            //开始向前一个一个像素点找这个白条右边界
            map++;
            j++;
            while ((*map) && j <= RIGHT_SIDE)
            {
                map++;
                j++;
            }
            result.right = j - 1;
            // printf("right2=%d\n", result.right);
            result.width = result.right - result.left + 1;
            return result;//找到就返回这个白条子的属性road_range
        }

    }
    return result;//没找到就返回result = { MISS, MISS, MISS };
}

////////////////////////////////////////////
//功能：查找联通性
//输入：当前行的白条子属性road_range* new_white，上一行的赛道属性road* pre_line，上一行的选道的白条子编号uint8_t continue_id
//输出：编号
//备注：
///////////////////////////////////////////
uint8_t is_connect(road_range* new_white, road* pre_line, uint8_t continue_id)
{
    uint8_t j_d;
    uint8_t jd_return = 0;
    uint8_t u_left = new_white->left;//当前行
    uint8_t u_right = new_white->right;
    for (j_d = 1; j_d <= pre_line->white_num; j_d++)//寻找上一行与当前行的白条子是否联通
    {
        road_range* pre_road = &pre_line->connected[j_d];
        if (u_left <= pre_road->right && u_right >= pre_road->left)//如果相连0
        {
            if (!jd_return)
                jd_return = j_d;//j_d到本次函数跑完  等于上一行的白条子数目或者是上一行选道的白条子编号
            if (j_d == continue_id)
                return continue_id;//只有相连并且与上一行的选道的白条子相连通才是希望的白条子
        }
    }
    return jd_return;//没找到返回0   或返回一个相连的但不与选道联通的白条子编号
}


uint8_t is_connect_continue(road_range* new_white, road_range* pre_line)
{

    uint8_t u_left = new_white->left;//当前行
    uint8_t u_right = new_white->right;
    road_range* pre_road = pre_line;
    if (u_left <= pre_road->right && u_right >= pre_road->left)//如果相连0
    {
        return 1;//只有相连并且与上一行的选道的白条子相连通才是希望的白条子
    }

    return 0;//没找到返回0
}
////////////////////////////////////////////
//功能：查找父节点
//输入：节点编号//
//输出：最老祖先
//备注：含路径压缩
///////////////////////////////////////////
int find_f(int node)
{
    if (f[node] == node)return node;//找到最古老祖先，return
    f[node] = find_f(f[node]);//向上寻找自己的父节点
    return f[node];
}

////////////////////////////////////////////
//功能：提取跳变沿 并对全部白条子标号
//输入：IMG[120][188]
//输出：white_range[120]
//备注：指针提速
///////////////////////////////////////////
void search_white_range1()
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t istart = NEAR_LINE;//处理起始行
    uint8_t iend = FAR_LINE;//处理终止行
    uint8_t tnum = 0;//当前行白条数
    all_connect_num = 0;//白条编号初始化
    uint8_t* map = NULL;
    bool found_road_start = 0;
    bool found_road_end = 0;
    for (i = istart; i >= iend; i--)
    {
        map = &IMG[i][LEFT_SIDE];//指针行走加快访问速度
        tnum = 0;
        uint8_t longest = 0;
        for (j = LEFT_SIDE; j <= RIGHT_SIDE; j++, map++)
        {
            if ((*map))//遇白条左边界
            {
                tnum++;
                if (tnum >= white_num_MAX)break;
                range* now_white = &white_range[i].area[tnum];
                now_white->left = j;

                //开始向后一个一个像素点找这个白条右边界
                map++;
                j++;

                while ((*map) && j <= RIGHT_SIDE)
                {
                    map++;
                    j++;
                }
                now_white->right = j - 1;
                if (now_white->right - now_white->left > longest)
                {
                    longest = now_white->right - now_white->left;
                }
                now_white->connect_num = ++all_connect_num;//白条数加一，给这个白条编号
            }
        }
        white_range[i].num = tnum;
        if (!found_road_start && longest >= 50)
        {
            found_road_start = 1;
            road_start = i;
        }
        else if (!found_road_end && found_road_start && longest < 30)
        {
            found_road_end = 1;
            road_end = i;
        }
    }
}

////////////////////////////////////////////
//功能：寻找白条子连通性，将全部联通白条子的节点编号刷成最古老祖先的节点编号
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_all_connect()
{
    //f数组初始化
    for (int i = 1; i <= all_connect_num; i++)
        f[i] = i;

    //u为up d为down 即为当前处理的这两行中的上面那行和下面那行
    //u_num：上面行白条数
    //u_left：上面行当前白条左边界
    //u_right：上面行当前白条右边界
    //i_u：当前处理的这个白条是当前这行（上面行）白条中的第i_u个
    int u_num, i_u, u_left, u_right;
    int d_num, i_d, d_left, d_right;
    all_range* u_white = NULL;
    all_range* d_white = NULL;
    for (int i = road_start; i > FAR_LINE; i--)//因为每两行每两行比较 所以循环到FAR_LINE+1
    {
        u_num = white_range[i - 1].num;
        d_num = white_range[i].num;
        u_white = &white_range[i - 1];
        d_white = &white_range[i];
        i_u = 1; i_d = 1;

        //循环到当前行或上面行白条子数耗尽为止
        while (i_u <= u_num && i_d <= d_num)
        {
            //变量先保存，避免下面访问写的冗杂且访问效率低
            u_left = u_white->area[i_u].left;
            u_right = u_white->area[i_u].right;
            d_left = d_white->area[i_d].left;
            d_right = d_white->area[i_d].right;

            if (u_left <= d_right && u_right >= d_left)//如果两个白条联通
                f[find_f(u_white->area[i_u].connect_num)] = find_f(d_white->area[i_d].connect_num);//父节点连起来

            //当前算法规则，手推一下你就知道为啥这样了
            if (d_right > u_right)i_u++;
            if (d_right < u_right)i_d++;
            if (d_right == u_right) { i_u++; i_d++; }
        }
    }
}

////////////////////////////////////////////
//功能：寻找赛道
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_road1()
{
    uint8_t istart = road_start;
    uint8_t iend = FAR_LINE;
    //top_road = NEAR_LINE;//赛道最高处所在行数，先初始化话为最低处
    int road_f = -1;//赛道所在连通域父节点编号，先初始化为-1，以判断是否找到赛道
    int while_range_num = 0, roud_while_range_num = 0;
    all_range* twhite_range = NULL;
    road* tmy_road = NULL;
    //寻找赛道所在连通域
    // 寻找最中心的白条子
    for (int i = 1; i <= white_range[istart].num; i++)
        if (white_range[istart].area[i].right - white_range[istart].area[i].left >= 40)
        {

            road_f = find_f(white_range[istart].area[i].connect_num);
            break;
        }

    if (road_f == -1)//若赛道没在中间，在NEAR_LINE行选一行最长的认为这就是赛道
    {
        int widthmax = 0, jselect = 1;
        for (int i = 1; i <= white_range[istart].num; i++)
            if (white_range[istart].area[i].right - white_range[istart].area[i].left > widthmax)
            {
                widthmax = white_range[istart].area[i].right - white_range[istart].area[i].left;
                jselect = i;
            }
        road_f = find_f(white_range[istart].area[jselect].connect_num);//找到父节点编号
    }
    //现在我们已经得到了赛道所在连通域父节点编号，接下来把所有父节点编号是road_f的所有白条子扔进赛道数组就行了
    for (int i = istart; i >= iend; i--)
    {
        //变量保存，避免之后写的冗杂且低效
        twhite_range = &white_range[i];
        tmy_road = &my_road[i];
        while_range_num = twhite_range->num;//每行白条数量
        tmy_road->white_num = 0;
        roud_while_range_num = 0;
        for (int j = 1; j <= while_range_num; j++)
        {
            if (find_f(twhite_range->area[j].connect_num) == road_f)
            {
                tmy_road->white_num++; roud_while_range_num++;
                tmy_road->connected[roud_while_range_num].left = twhite_range->area[j].left;
                tmy_road->connected[roud_while_range_num].right = twhite_range->area[j].right;
                tmy_road->connected[roud_while_range_num].width = twhite_range->area[j].right - twhite_range->area[j].left;
            }
        }
    }
}
void find_road_state()
{
    if (step == single_side)
    {
        single_side_road_choose();
    }
    else
    {
        find_road1();
    }
}

////////////////////////////////////////////
//功能：返回相连下一行白条子编号
//输入：i_start起始行  j_start白条标号
//输出：白条标号
//备注：认为下一行与本行赛道重叠部分对多的白条为选定赛道
///////////////////////////////////////////
uint8_t find_continue(uint8_t i_start, uint8_t j_start)
{
    uint8_t j_return;
    uint8_t j;
    uint8_t width_max = 0;
    uint8_t width_new = 0;
    uint8_t left = 0;
    uint8_t right = 0;
    uint8_t dright, dleft, uright, uleft;
    j_return = MISS;//如果没找到，输出255
    if (j_start > my_road[i_start].white_num)
        return MISS;
    //选一个重叠最大的
    for (j = 1; j <= my_road[i_start - 1].white_num; j++)
    {
        dleft = my_road[i_start].connected[j_start].left;
        dright = my_road[i_start].connected[j_start].right;
        uleft = my_road[i_start - 1].connected[j].left;
        uright = my_road[i_start - 1].connected[j].right;
        if (//相连
            dleft <= uright
            &&
            dright >= uleft
            )
        {
            //计算重叠大小
            if (dleft < uleft) left = uleft;
            else left = dleft;

            if (dright > uright) right = uright;
            else right = dright;

            width_new = right - left + 1;

            if (width_new > width_max)
            {
                width_max = width_new;
                j_return = j;
            }
        }

    }
    return j_return;
}

////////////////////////////////////////////
//功能：通用决定双边
//输入：
//输出：
//备注：
///////////////////////////////////////////
void ordinary_two_line(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t j_continue[CAMERA_H];//第一条连通路径
    uint8_t i_start;
    uint8_t i_end;
    uint8_t j_start = MISS;
    int width_max;

    //寻找起始行最宽的白条子
    i_start = road_start;
    i_end = FAR_LINE - 1;
    width_max = 0;
    for (j = 1; j <= my_road[i_start].white_num; j++)
    {
        if (my_road[i_start].connected[j].width > width_max)
        {
            width_max = my_road[i_start].connected[j].width;
            j_start = j;
        }
    }
    j_continue[i_start] = j_start;

    if (step == single_side)
    {
        single_side_bridge_two_line(j_continue);
    }
    else
    {
        //记录连贯区域编号
        for (i = i_start; i > i_end; i--)
        {
            //如果相连编号大于该行白条数，非正常，从此之后都MISS
            if (j_continue[i] > my_road[i].white_num)
            {
                j_continue[i - 1] = MISS;
            }
            else
            {
                j_continue[i - 1] = find_continue(i, j_continue[i]);
            }

        }
    }


    //全部初始化为MISS
    my_memset(left_line, MISS, CAMERA_H);
    my_memset(right_line, MISS, CAMERA_H);


    for (i = i_start; i > i_end; i--)
    {
        if (j_continue[i] >= 1 && j_continue[i] <= my_road[i].white_num)
        {
            top_road = i;
            left_line[i] = my_road[i].connected[j_continue[i]].left;
            right_line[i] = my_road[i].connected[j_continue[i]].right;

        }
        else
        {
            left_line[i] = MISS;
            right_line[i] = MISS;
        }
    }
    if (step == 1)
    {
        renew_road();
    }
}

////////////////////////////////////////////
//功能：数组初始化
//输入：uint8_t* ptr 数组首地址, uint8_t num初始化的值, uint8_t size数组大小
//输出：
//备注：因为k66库中认为memset函数不安全，所以无法使用；因此需要自己写一个my_memset
///////////////////////////////////////////
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size)
{
    uint8_t* p = ptr;
    uint8_t my_num = num;
    uint8_t Size = size;
    for (int i = 0; i < Size; i++, p++)
    {
        *p = my_num;
    }
}


////////////////////////////////////////////
//功能：中线合成
//输入：左右边界
//输出：中线
//备注：
///////////////////////////////////////////
void get_mid_line(void)
{
    my_memset(mid_line, MISS, CAMERA_H);

    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
    {
        if (left_line[i] != MISS)//直道
        {
            mid_line[i] = (left_line[i] + right_line[i]) / 2;
        }

        else
        {
            mid_line[i] = MISS;
        }
    }

}
uint8_t mid(uint8_t i)
{
    if (mid_line[i] >= 187 || mid_line[i] <= 1)
    {
        if (i > 25)
        {
            return mid(i - 1);
        }
        if (i <= 25)
        {
            return 0;
        }
    }
    else
    {
        return 1;
    }
}

void define_mid_255()
{
    if (mid(NEAR_LINE) == 0)
    {
        for (int i = NEAR_LINE; i > top_road + 5; i--)
        {
            mid_line[i] = 94;
        }
    }
}
//数值初始化
void datas_init()
{
    road_start = NEAR_LINE;
    road_end = FAR_LINE;

    left_f2_y = 0;//左边线二阶导拐点
    right_f2_y = 0;//右边线二阶导拐点
    left_guai_H = 0;//左下边界拐点
    right_guai_H = 0;//右下边界拐点
    left_guai_H2 = 0;//左上边界拐点
    right_guai_H2 = 0;//右上边界拐点
    left_line_datas[0] = 0;
    right_line_datas[0] = 0;
    single_left_side_y = 0;
    single_right_side_y = 0;
}

void element_check()//人机交互
{
    element_ok = 1;
}

//打印数值的函数
void print_f()
{
    printf("————————————————————————————————————\n");
    //printf("threshold1 = %d,thresholds2 = %d,threshold3 = %d,threshold4 = %d\n", threshold_1, threshold_2, threshold_3, threshold_4);
    printf("元素状态 = %d,弯道状态 = %d,元素状态 = %d\n", step, bend_state, yuansu_state);
    printf("单边桥判断帧数 = %d, 弯道判断帧数 = %d\n", single_side_fps, bend_fps);
    printf("right_guai_H = %d , left_guai_H = %d\n", right_guai_H, left_guai_H);
    printf("right_guai_H2 = %d , left_guai_H2 = %d\n", right_guai_H2, left_guai_H2);
    printf("right_k = %f, left_k = %f\n", right_line_datas[0], left_line_datas[0]);
    printf("top_road = %d\n", top_road);
    printf("left_f2_y = %d, right_f2_y = %d\n", left_f2_y, right_f2_y);
    printf("mid_line[前瞻] = %d\n", mid_line[35]);
}


void calc_stair_distance()
{
    if (NEAR_LINE - road_start >= 2)
    {
        stair_distance = NEAR_LINE - road_start;
    }
    else {
        stair_distance = NEAR_LINE - road_end;
    }
}

// 三级台阶补线
void mend_stairs()
{
    float input[2];
    if (NEAR_LINE - road_start >= 2)
    {
        if (road_start - road_end >= 10)
        {
            least_square_get_line(left_line, road_start - 5, 5, input, NEAR_LINE, road_start - 5);
            least_square_get_line(right_line, road_start - 5, 5, input, NEAR_LINE, road_start - 5);
        }
        else {
            for (int i = NEAR_LINE; i >= road_start; i--)
            {
                left_line[i] = 84;
                right_line[i] = 104;
            }
        }
    }
    if (road_end - FAR_LINE >= 5)
    {
        if (road_start - road_end >= 15)
        {
            least_square_get_line(left_line, road_end + 10, 5, input, road_end + 10, FAR_LINE);
            least_square_get_line(right_line, road_end + 10, 5, input, road_end + 10, FAR_LINE);
        }
        else {
            for (int i = road_end; i >= FAR_LINE; i--)
            {
                left_line[i] = 84;
                right_line[i] = 104;
            }
        }
    }
}

// 远处的单边桥补线
void mend_far_single_side()
{
    float input[2];
    if (NEAR_LINE - road_start >= 2 && road_start - road_end >= 10)
    {
        least_square_get_line(left_line, road_start - 5, 5, input, NEAR_LINE, road_start - 5);
        least_square_get_line(right_line, road_start - 5, 5, input, NEAR_LINE, road_start - 5);
        on_single_side_road = 0;
    }
    else {
        int white_count = 0;
        for (int i = NEAR_LINE; i >= NEAR_LINE - 20; i--)
        {
            for (int j = 64; j <= 124; j++)
            {
                if (IMG[i][j])
                    white_count++;
            }
        }
        if (white_count >= 800)
            on_single_side_road = 1;
        else on_single_side_road = 0;
    }
}

void front_stairs_end_check()
{
    int white_count = 0;
    for (int i = NEAR_LINE; i >= NEAR_LINE - 20; i--)
    {
        for (int j = 64; j <= 124; j++)
        {
            if (IMG[i][j])
                white_count++;
        }
    }
    if (white_count < 150)
    {
        front_stairs_end_judge_cnt++;
        if (front_stairs_end_judge_cnt > 100)
        {
            front_stairs_end_judge_cnt = 0;
            front_stairs_end_flag = 1;
        }
    }
    else {
        front_stairs_end_judge_cnt = 0;
        front_stairs_end_flag = 0;
    }
}

void back_stairs_end_check()
{
    int white_count = 0;
    for (int i = NEAR_LINE; i >= NEAR_LINE - 20; i--)
    {
        for (int j = 64; j <= 124; j++)
        {
            if (IMG[i][j])
                white_count++;
        }
    }
    if (white_count < 300)
    {
        back_stairs_end_judge_cnt++;
        if (back_stairs_end_judge_cnt > 100)
        {
            back_stairs_end_judge_cnt = 0;
            back_stairs_end_flag = 1;
        }
    }
    else {
        back_stairs_end_judge_cnt = 0;
        back_stairs_end_flag = 0;
    }
}


void jolt_end_check()
{
    int white_count = 0;
    for (int i = NEAR_LINE; i >= NEAR_LINE - 20; i--)
    {
        for (int j = 64; j <= 124; j++)
        {
            if (IMG[i][j])
                white_count++;
        }
    }
    if (white_count < 200)
    {
        jolt_end_judge_cnt++;
        if (jolt_end_judge_cnt > 100)
        {
            // 达到阈值，退出JOLT状态
            jolt_end_judge_cnt = 0;
            jolt_end_flag = 1;
            now_element = NORM;
        }
    }
    else {
        jolt_end_flag = 0;
        jolt_end_judge_cnt = 0;
    }
}

////////////////////////////////////////////
//功能：图像处理主程序
//输入：
//输出：
//备注：
///////////////////////////////////////////
void image_main()
{
#if ONBOARD
#else 
    now_element = BACK_STAIRS;
#endif
    // mid_update = 0;
    datas_init();
    THRE();
    head_clear();
    search_white_range1();
    find_all_connect();
    find_road_state();
    /*到此处为止，我们已经得到了属于赛道的结构体数组my_road[CAMERA_H]*/

    ordinary_two_line();

    get_mid_line();
#if ONBOARD
#else
    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
    {
        for (int j = left_line[i] + 1; j < right_line[i]; j++)
        {
            IMG[i][j] = purple;
        }
    }
#endif 
    element_check();
    pre_step = step;
    switch (now_element)
    {
    case SINGLE_SIDE:
        mend_far_single_side();
        if (on_single_side_road)
            general_judgement();
        break;

    case FRONT_STAIRS:
        calc_stair_distance();
        mend_stairs();
        front_stairs_end_check();
        break;
    case BACK_STAIRS:
        calc_stair_distance();
        mend_stairs();
        back_stairs_end_check();
        break;
    case JOLT:
        jolt_end_check();
        break;
    }

    get_mid_line();

    define_mid_255();
    int sum = 0;
    for (int i = error_calc_near_line; i >= error_calc_far_line; i--)
    {
        sum += mid_line[i];
    }
    error = sum / (error_calc_near_line - error_calc_far_line + 1) - 94;
    if (error < -50) error = 0;
    if (error > 50) error = 0;

    mid_update = 1;
    //print_f();
#if ONBOARD
#else
    printf("dis: %d, start: %d, end: %d\n", stair_distance, road_start, road_end);
    for (int i = 0; i < CAMERA_W; i++)
    {
        IMG[error_calc_near_line][i] = purple;
        IMG[error_calc_far_line][i] = purple;
    }
    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
        if (mid_line[i] != MISS)
        {
            IMG[i][left_line[i]] = blue;
            IMG[i][right_line[i]] = green;
            IMG[i][mid_line[i]] = red;
        }
#endif
}
