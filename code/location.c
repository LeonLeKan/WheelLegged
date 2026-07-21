#include "location.h"
#include "pid.h"
#include "waypoint.h"
#include "save_manager.h"
#include "control_state.h"
#include "ins.h"
#include "image.h"

waypoint_node_t *current_target = NULL;
uint8_t road_idx[5];
uint8_t modify_idx[5];
uint8_t uturn_3_idx[2];
uint8_t road_num = 0;
uint8_t disturb_flag = 0;
float last_raw_yaw = 0.0f;
float target_yaw = 0.0f;
uint8_t thunder_count = 0;
uint8_t thunder_direction_flag = 0;
double thunder_final_angle = 0.0f;

static double direction_calculate(double x_target, double y_target, double x_now, double y_now)
{
    double dx = x_target - x_now;
    double dy = y_target - y_now;

    if (dx == 0.0 && dy == 0.0)
    {
        return 0.0;
    }

    double yaw = atan2(dy, dx) * RAD_TO_DEG;
    double delta = yaw - last_raw_yaw;
    if(delta > 180.f) delta -= 360;
    if(delta < -180.f) delta += 360;

    last_raw_yaw = yaw;
    target_yaw += delta;
    return target_yaw;
}

static void advance_to_next_waypoint(void)
{
    if (current_target->next)
    {
        current_target = current_target->next;
    }
    else
    {
        current_target = NULL;
    }
}

void location_update_0(void)
{
    waypoint_list_t *list = save_manager_get_active_save();
    
    if (!current_target)
    {
        current_target = list->head->next;
    }
        
    if (!current_target)
    {
        return;
    }

    double x_now = imu.output.position_x;
    double y_now = imu.output.position_y;

    double x_tgt = current_target->waypoint.x;
    double y_tgt = current_target->waypoint.y;

    double target_deg = direction_calculate(x_tgt, y_tgt, x_now, y_now);

    yaw_loop.Target = target_deg;
    yaw_loop.Actual = imu.output.euler[2];

    double dx = x_tgt - x_now;
    double dy = y_tgt - y_now;

    if (sqrt(dx * dx + dy * dy) < XYMIN_BIA_0)
    {
        if (current_target->id_startup == tunum->data_int)
        {
            imu.rot_comp.param.yaw_compensate_cw = cwtucop->data_float;
            imu.rot_comp.param.yaw_compensate_ccw = ccwtucop->data_float;
            current_state = STATE_UTURN;
            advance_to_next_waypoint();
        }
        else
        {
            imu.rot_comp.param.yaw_compensate_cw = 0.0f;
            imu.rot_comp.param.yaw_compensate_ccw = 0.0f;
            advance_to_next_waypoint();
        }
    }

    if (!current_target) 
    {
        yaw_de_final = imu.output.euler[2];
        current_state = STATE_DEACCELERATE;
    }
}

void location_update_1(void)
{
    waypoint_list_t *list = save_manager_get_active_save();
    
    if (!current_target)
    {
        current_target = list->head->next;
    }
        
    if (!current_target)
    {
        return;
    }

    double x_now = imu.output.position_x;
    double y_now = imu.output.position_y;

    double x_tgt = current_target->waypoint.x;
    double y_tgt = current_target->waypoint.y;

    double target_deg = direction_calculate(x_tgt, y_tgt, x_now, y_now);

    yaw_loop.Target = target_deg;
    yaw_loop.Actual = imu.output.euler[2];

    double dx = x_tgt - x_now;
    double dy = y_tgt - y_now;
    if (current_state != STATE_ACCELERATE)
    {
        if (sqrt(dx * dx + dy * dy) < xythun->data_float && sqrt(dx * dx + dy * dy) >= XYMIN_BIA_1 &&
            current_state != STATE_TRA_THUNDER &&
            current_state != STATE_THUNDER &&
            thunder_count < thnum->data_int)
        {
            current_state = STATE_DE_THUNDER;
        }
    }

    if (sqrt(dx * dx + dy * dy) < XYMIN_BIA_1)
    {
        if (current_target->id_startup == tunum->data_int)
        {
            imu.rot_comp.param.yaw_compensate_cw = cwtucop->data_float;
            imu.rot_comp.param.yaw_compensate_ccw = ccwtucop->data_float;
            current_state = STATE_UTURN;
            advance_to_next_waypoint();
        }
        else if (thunder_count < thnum->data_int)
        {
            thunder_count++;
            yaw_thunder_memory = imu.output.euler[2];
            thunder_final_angle = atan2((current_target->next->waypoint.y - current_target->waypoint.y), (current_target->next->waypoint.x - current_target->waypoint.x)) * RAD_TO_DEG;
            if ((yaw_thunder_memory >= thunder_final_angle) &&
                 !((yaw_thunder_memory >= thunder_final_angle) && (thunder_final_angle >= 0.0f)))
            {
                thunder_direction_flag = 1;
            }
            else if ((yaw_thunder_memory < thunder_final_angle) &&
                 !((yaw_thunder_memory < thunder_final_angle) && (thunder_final_angle < 0.0f))) 
            {
                thunder_direction_flag = 2;
            }
            else if ((yaw_thunder_memory >= thunder_final_angle) && (thunder_final_angle >= 0.0f))
            {
                thunder_direction_flag = 4;
            }
            else if ((yaw_thunder_memory < thunder_final_angle) && (thunder_final_angle < 0.0f))
            {
                thunder_direction_flag = 3;
            }
            imu.rot_comp.param.yaw_compensate_cw = cwtucop1->data_float;
            imu.rot_comp.param.yaw_compensate_ccw = ccwtucop1->data_float;
            current_state = STATE_THUNDER;
            advance_to_next_waypoint();
        }
        else
        {
            imu.rot_comp.param.yaw_compensate_cw = 0;
            imu.rot_comp.param.yaw_compensate_ccw = 0;
            advance_to_next_waypoint();
        }
    }

    if (!current_target)
    {
        yaw_de_final = imu.output.euler[2];
        current_state = STATE_DEACCELERATE;
    }
}

void location_update_2_0(void)
{
    waypoint_list_t *list = save_manager_get_active_save();
    
    if (!current_target)
    {
        current_target = list->head->next;
    }
        
    if (!current_target)
    {
        return;
    }

    double x_now = imu.output.position_x;
    double y_now = imu.output.position_y;

    double x_tgt = current_target->waypoint.x;
    double y_tgt = current_target->waypoint.y;

    double target_deg = direction_calculate(x_tgt, y_tgt, x_now, y_now);

    yaw_loop.Target = target_deg;
    yaw_loop.Actual = imu.output.euler[2];

    double dx = x_tgt - x_now;
    double dy = y_tgt - y_now;

    if (sqrt(dx * dx + dy * dy) < XYMIN_BIA_2)
    {
        advance_to_next_waypoint();
    }

    if (current_target->id_startup == tunum->data_int)
        {
            imu.rot_comp.param.yaw_compensate_cw = cwtucop->data_float;
            imu.rot_comp.param.yaw_compensate_ccw = ccwtucop->data_float;
            current_state = STATE_UTURN;
            advance_to_next_waypoint();
        }

    if (!current_target) 
    {
        yaw_de_final = imu.output.euler[2];
        current_state = STATE_DEACCELERATE;
    }
}

void location_update_2_1(void)
{
    waypoint_list_t *list = save_manager_get_active_save();
    
    if (!current_target)
    {
        current_target = list->head->next;
    }
        
    if (!current_target)
    {
        return;
    }

    double x_now = imu.output.position_x;
    double y_now = imu.output.position_y;

    double x_tgt = current_target->waypoint.x;
    double y_tgt = current_target->waypoint.y;

    double target_deg = direction_calculate(x_tgt, y_tgt, x_now, y_now);

    yaw_loop.Target = target_deg;
    yaw_loop.Actual = imu.output.euler[2];

    double dx = x_tgt - x_now;
    double dy = y_tgt - y_now;

    if (sqrt(dx * dx + dy * dy) < XYMIN_BIA_2)
    {
        if (road_idx[road_num] == uturn_3_idx[0] && current_target->id_startup == uturn_3_idx[1])
        {
            imu.rot_comp.param.yaw_compensate_cw = cwtucop->data_float;
            imu.rot_comp.param.yaw_compensate_ccw = ccwtucop->data_float;
            current_state = STATE_UTURN;
            advance_to_next_waypoint();
        }
        else
        {
            advance_to_next_waypoint();
        }
    }

    if (!current_target) 
    {
        ins_stop();
        single_side_end_judge.thre = sicnt->data_int;
        now_element = SINGLE_SIDE;
        image_block = 1;
        current_state = STATE_DECLINE_FRONT_SINGLE_SIDE;
    }
}

void location_update_2_2(void)
{
    static uint8_t de_stairs_flag = 0;
    waypoint_list_t *list = save_manager_get_active_save();
    
    if (!current_target)
    {
        current_target = list->head->next;
    }
        
    if (!current_target)
    {
        return;
    }

    double x_now = imu.output.position_x;
    double y_now = imu.output.position_y;

    double x_tgt = current_target->waypoint.x;
    double y_tgt = current_target->waypoint.y;

    double target_deg = direction_calculate(x_tgt, y_tgt, x_now, y_now);

    yaw_loop.Target = target_deg;
    yaw_loop.Actual = imu.output.euler[2];

    double dx = x_tgt - x_now;
    double dy = y_tgt - y_now;

    if (sqrt(dx * dx + dy * dy) < XYMIN_BIA_2)
    {
        if (road_idx[road_num] == uturn_3_idx[0] && current_target->id_startup == uturn_3_idx[1])
        {
            imu.rot_comp.param.yaw_compensate_cw = cwtucop->data_float;
            imu.rot_comp.param.yaw_compensate_ccw = ccwtucop->data_float;
            current_state = STATE_UTURN;
            advance_to_next_waypoint();
        }
        else
        {
            advance_to_next_waypoint();
        }
    }

    if (jump->data_int == 1)
    {
        if (current_target && !current_target->next && de_stairs_flag == 0) 
        {
            current_state = STATE_DE_STAIRS;
            de_stairs_flag = 1;
        }

        if (!current_target) 
        {
            ins_stop();
            de_stairs_flag = 0;
            front_stairs_end_judge.thre = frcnt->data_int;
            now_element = FRONT_STAIRS;
            image_block = 1;
            current_state = STATE_FIND_JUMP;
        }
    }
    else if (jump->data_int == 2)
    {
        if (current_target && !current_target->next && de_stairs_flag == 0) 
        {
            current_state = STATE_DE_STAIRS;
            de_stairs_flag = 1;
        }

        if (!current_target) 
        {
            ins_stop();
            de_stairs_flag = 0;
            front_stairs_end_judge.thre = frcnt->data_int;
            now_element = FRONT_STAIRS;
            image_block = 1;
            current_state = STATE_FIND_JUMP;
        }
    }
}

void location_update_2_3(void)
{
    waypoint_list_t *list = save_manager_get_active_save();
    
    if (!current_target)
    {
        current_target = list->head->next;
    }
        
    if (!current_target)
    {
        return;
    }

    double x_now = imu.output.position_x;
    double y_now = imu.output.position_y;

    double x_tgt = current_target->waypoint.x;
    double y_tgt = current_target->waypoint.y;

    double target_deg = direction_calculate(x_tgt, y_tgt, x_now, y_now);

    yaw_loop.Target = target_deg;
    yaw_loop.Actual = imu.output.euler[2];

    double dx = x_tgt - x_now;
    double dy = y_tgt - y_now;

    if (sqrt(dx * dx + dy * dy) < XYMIN_BIA_2)
    {
        if (road_idx[road_num] == uturn_3_idx[0] && current_target->id_startup == uturn_3_idx[1])
        {
            imu.rot_comp.param.yaw_compensate_cw = cwtucop->data_float;
            imu.rot_comp.param.yaw_compensate_ccw = ccwtucop->data_float;
            current_state = STATE_UTURN;
            advance_to_next_waypoint();
        }
        else
        {
            advance_to_next_waypoint();
        }
    }

    if (!current_target) 
    {
        ins_stop();
        back_stairs_end_judge.thre = bacnt->data_int;
        now_element = BACK_STAIRS;
        image_block = 1;
        current_state = STATE_DE_FOLLOW;
    }
}

void location_update_2_4(void)
{
    waypoint_list_t *list = save_manager_get_active_save();
    
    if (!current_target)
    {
        current_target = list->head->next;
    }
        
    if (!current_target)
    {
        return;
    }

    double x_now = imu.output.position_x;
    double y_now = imu.output.position_y;

    double x_tgt = current_target->waypoint.x;
    double y_tgt = current_target->waypoint.y;

    double target_deg = direction_calculate(x_tgt, y_tgt, x_now, y_now);

    yaw_loop.Target = target_deg;
    yaw_loop.Actual = imu.output.euler[2];

    double dx = x_tgt - x_now;
    double dy = y_tgt - y_now;

    if (sqrt(dx * dx + dy * dy) < XYMIN_BIA_2)
    {
        if (road_idx[road_num] == uturn_3_idx[0] && current_target->id_startup == uturn_3_idx[1])
        {
            imu.rot_comp.param.yaw_compensate_cw = cwtucop->data_float;
            imu.rot_comp.param.yaw_compensate_ccw = ccwtucop->data_float;
            current_state = STATE_UTURN;
            advance_to_next_waypoint();
        }
        else
        {
            advance_to_next_waypoint();
        }
    }

    if (!current_target) 
    {
        ins_stop();
        roll_base = rollba->data_float;
        jolt_end_judge.thre = jocnt->data_int;
        now_element = JOLT;
        image_block = 1;
        joy_loop.Target = joangle->data_float;
        current_state = STATE_DE_JOLT;
    }
}
