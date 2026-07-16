#include "zf_common_headfile.h"
#include "imu.h"
#include "small_driver_uart_control.h"
#include "screen_interface.h"
#include "ins.h"
#include "menu.h"
#include "image.h"
#include "servo.h"
#include "control.h"
#include "control_state.h"
#include "connecting_rod.h"
#include "save_manager_ui.h"
#include "pid.h"
#include "location.h"
#include "wifi.h"
#include "font.h"

void gpio_init_all(void);
void imu_state_show(void);
void start_imu_initial_estimation(void);
void start_imu_auto_gyro_calibration(void);

void remote_control(void);
void calculate_route(void);
void calculate_modify(void);
void calculate_uturn_3(void);

uint8_t motion_mode1 = 0;
uint8_t motion_mode2 = 0;
uint8_t motion_mode3 = 0;
uint8_t motion_mode4 = 0;

uint8_t get_ready = 0;
uint8_t test_2_save = 0;

MahonyFilterParam mahony_filter_param = {
    .Kp = 0.7f,
    .Ki = 0.005f,
    .dt = IMU_UPDATE_INTERVAL_MS / 1000.0f};

RotationCompensationParam rotation_compensation_param;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_init();                  // 调试串口信息初始化

    // flash初始化
    flash_init();

    // 串口调试初始化
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);

    // 控制相关初始化
    mt9v03x_init();
    control_init();
    car_balance();
    gpio_init_all();
    para_zero();
    uart_receiver_init();

    // 菜单初始化
    create_1st_menu();
    create_2nd_menu();
    create_3rd_menu();
    menu_load();

    // 存点初始化
    save_manager_init(ins_to_control_menu);

    // 如有需要，添加wifi
    if (gpio_get_level(DIP_4) == 0)
    {
        wifi_init();
    }

    // imu转圈偏移初始化
    rotation_compensation_param.rotation_thre = 120;
    rotation_compensation_param.yaw_compensate_cw = cwtucop->data_float;
    rotation_compensation_param.yaw_compensate_ccw = ccwtucop->data_float;

    // 屏幕初始化
    pit_ms_init(SCREEN_UPDATE_PIT, SCREEN_UPDATE_INTERVAL_MS);
    pit_disable(SCREEN_UPDATE_PIT);
    screen_init();

    // imu初始化
    pit_ms_init(IMU_UPDATE_PIT, IMU_UPDATE_INTERVAL_MS);
    pit_disable(IMU_UPDATE_PIT);
    imu_init(mahony_filter_param, rotation_compensation_param);

    // 菜单里读取陀螺仪固定偏置
    imu_set_gyro_bias(rollbias->data_float, pitchbias->data_float, yawbias->data_float);

    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;
    enable_dis_shadow = (uint8_t)esha->data_int;
    k_gray_dis = (float)kgray->data_int / 10.0f;
    WHITE_GAP = (uint8_t)whiga->data_int;
    MIN_THRE = (uint8_t)minth->data_int;
    MAX_THRE = (uint8_t)maxth->data_int;

    // 判断科目几，以及科目三的路径
    if (testnum->data_int == 2)
    {
        calculate_route();
        calculate_modify();
        calculate_uturn_3();
        save_manager_load(testnum->data_int + road_idx[road_num]);
    }
    else if (testnum->data_int == 1 || testnum->data_int == 0)
    {
        save_manager_load(testnum->data_int);
    }

    // 设置曝光度
    mt9v03x_set_exposure_time(imexp->data_int);

    if (gpio_get_level(DIP_3) == 0 && gpio_get_level(DIP_1) == 1 && gpio_get_level(DIP_2) == 1 && gpio_get_level(DIP_4) == 1)
    {
        imu_skip_initial_estimation();
        start_imu_auto_gyro_calibration();
    }
    else if (gpio_get_level(DIP_3) == 1 && gpio_get_level(DIP_1) == 1 && gpio_get_level(DIP_2) == 1)
    {
        imu_skip_initial_estimation();
    }

    bool is_calib_saved = false;
    while (true)
    {
        // 主循环检测拨码的状态
        motion_mode1 = gpio_get_level(DIP_1);
        motion_mode2 = gpio_get_level(DIP_2);
        motion_mode3 = gpio_get_level(DIP_3);
        motion_mode4 = gpio_get_level(DIP_4);

        if (motion_mode1 == 1 && motion_mode2 == 1 && (motion_mode3 + motion_mode4 != 0))
        {
            current_state = STATE_IDLE;
            imu_state_show();

            if (motion_mode3 == 0 && !is_calib_saved && imu.calib.is_calibrated == 1)
            {
                rollbias->data_float = imu.calib.gyro_bias[0];
                pitchbias->data_float = imu.calib.gyro_bias[1];
                yawbias->data_float = imu.calib.gyro_bias[2];
                menu_save();
                is_calib_saved = true;
            }

            if (motion_mode4 == 0)
            {
                wifi_send_data();
            }
        }

        else if (motion_mode1 == 1 && motion_mode2 == 1 && motion_mode3 == 0 && motion_mode4 == 0)
        {
            while (true)
            {
                motion_mode2 = gpio_get_level(DIP_2);
                if (motion_mode1 == 1 && motion_mode2 == 1 && motion_mode3 == 0 && motion_mode4 == 0)
                {
                    wifi_send_image_orin();
                }
                else if (motion_mode1 == 1 && motion_mode2 == 0 && motion_mode3 == 0 && motion_mode4 == 0)
                {
                    wifi_send_image_thre();
                }
            }
        }

        else if (motion_mode1 == 0 && motion_mode2 == 1 && motion_mode3 == 1)
        {
            system_delay_ms(1000);
            ips200_clear();
            while (true)
            {
                remote_control();
                current_state = STATE_STAND_STILL;
                imu_state_show();
                if (gpio_get_level(BUTTON_OK) == 0 && get_ready == 0)
                {
                    get_ready = 1;
                    system_delay_ms(1000);
                    if (test_state == 1)
                    {
                        current_state = STATE_TEST;
                        ins_reset();
                    }
                    else if (test_state == 0)
                    {
                        current_state = STATE_ACCELERATE;
                        ins_reset();
                    }
                    while (true)
                    {
                        remote_control();
                        if (image_block == 1 && mt9v03x_finish_flag == 1)
                        {
                            image_main();
                            mt9v03x_finish_flag = 0;
                        }

                        if (testnum->data_int == 2 && test_state == 1)
                        {
                            if (test_2_save == 0)
                            {
                                ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_0, 116, 100, 150, 120, 0);
                            }
                            else if (test_2_save == 1)
                            {
                                ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_1, 118, 105, 150, 120, 0);
                            }
                            else if (test_2_save == 2)
                            {
                                ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_2, 118, 109, 150, 120, 0);
                            }
                            else if (test_2_save == 3)
                            {
                                ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_3, 96, 105, 150, 120, 0);
                            }
                            else if (test_2_save == 4)
                            {
                                ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_4, 106, 109, 150, 120, 0);
                            }
                            else if (test_2_save == 5)
                            {
                                ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_5, 100, 105, 150, 120, 0);
                            }
                            else if (test_2_save == 6)
                            {
                                ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_6, 94, 105, 150, 120, 0);
                            }
                            else if (test_2_save == 7)
                            {
                                ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_7, 110, 119, 150, 120, 0);
                            }
                        }
                        if (motion_mode4 == 0)
                        {
                            wifi_send_data();
                        }
                    }
                }
            }
        }

        else if (motion_mode1 == 0 && motion_mode2 == 0 && motion_mode3 == 1)
        {
            current_state = STATE_IDLE;
            ips200_clear();
            while (true)
            {
                if (menu_current_state == MENU_STATE_CONTROL)
                {
                    show_menu();
                    key_handle_menu();
                }
                else if (menu_current_state == MENU_STATE_INS)
                {
                    key_handle_ins();
                }

                if (motion_mode4 == 0)
                {
                    wifi_send_data();
                }
            }
        }
    }
}

void start_imu_initial_estimation(void)
{
    system_delay_ms(1000);             // 确保系统稳定
    imu_start_initial_estimation(500); // 采集500个样本进行初始姿态估计
}

void start_imu_auto_gyro_calibration(void)
{
    system_delay_ms(1000);                 // 确保系统稳定
    imu_start_auto_gyro_calibration(3000); // 采集3000个样本进行校准
}

void gpio_init_all(void)
{
    gpio_init(BUTTON_LF, GPI, 1, GPI_PULL_UP);
    gpio_init(BUTTON_RT, GPI, 1, GPI_PULL_UP);
    gpio_init(BUTTON_DN, GPI, 1, GPI_PULL_UP);
    gpio_init(BUTTON_UP, GPI, 1, GPI_PULL_UP);
    gpio_init(BUTTON_OK, GPI, 1, GPI_PULL_UP);
    gpio_init(DIP_1, GPI, 1, GPI_PULL_UP);
    gpio_init(DIP_2, GPI, 1, GPI_PULL_UP);
    gpio_init(DIP_3, GPI, 1, GPI_PULL_UP);
    gpio_init(DIP_4, GPI, 1, GPI_PULL_UP);
}

void imu_state_show(void)
{
    switch (imu.state)
    {
    case IMU_STATE_UNINIT:
        screen_printf(0, 0, "IMU State: UNINIT     ");
        break;
    case IMU_STATE_INITIAL_ESTIMATION:
        screen_printf(0, 0, "IMU State: ESTIMATING ");
        break;
    case IMU_STATE_AUTO_GYRO_CALIBRATING:
        screen_printf(0, 0, "IMU State: CALIBRATING");
        break;
    case IMU_STATE_NORMAL:
        screen_printf(0, 0, "IMU State: RUNNING    ");
        break;
    case IMU_STATE_ERROR:
        screen_printf(0, 0, "IMU State: ERROR      ");
        break;
    default:
        screen_printf(0, 0, "IMU State: UNKNOWN    ");
        break;
    }
}

void handle_channel(void)
{
    static uint16 last_ch2 = 0, last_ch3 = 0, last_ch4 = 0, last_ch5 = 0;

    if (last_ch2 == 0)
    {
        last_ch2 = uart_receiver.channel[2];
        return;
    }
    if (last_ch3 == 0)
    {
        last_ch3 = uart_receiver.channel[3];
        return;
    }
    if (last_ch4 == 0)
    {
        last_ch4 = uart_receiver.channel[4];
        return;
    }
    if (last_ch5 == 0)
    {
        last_ch5 = uart_receiver.channel[5];
        return;
    }

    if (last_ch2 != uart_receiver.channel[2])
    {
        // 在末尾添加当前位置点
        waypoint_t point;
        ins_get_current_position(&point.x, &point.y);
        ins_add_current(&waypoints, waypoints.tail, point);
    }

    else if ((uart_receiver.channel[3] == 1792 && last_ch3 != 1792) ||
             (uart_receiver.channel[3] == 192  && last_ch3 != 192))
    {
        // 科目三确认更换存档
        if (testnum->data_int == 2)
        {
            ins_reset();
            yaw_loop.Target = imu.output.euler[2];
            save_manager_load(testnum->data_int + test_2_save);
        }
    }

    else if (last_ch4 != uart_receiver.channel[4])
    {
        // 科目三更换存档
        if (testnum->data_int == 2)
        {
            test_2_save += 1;
            if (test_2_save > 7)
            {
                test_2_save = 0;
            }
        }
    }

    else if (last_ch5 != uart_receiver.channel[5])
    {
        // 保存到Flash，存入当前激活的存档
        waypoint_list_save_to_flash(&waypoints, INS_FLASH_START_PAGE + save_manager_instance.active_save_idx);
    }

    last_ch2 = uart_receiver.channel[2];
    last_ch3 = uart_receiver.channel[3];
    last_ch4 = uart_receiver.channel[4];
    last_ch5 = uart_receiver.channel[5];
}

void remote_control(void)
{
    if (1 == uart_receiver.finsh_flag)
    {
        speed_loop.Target = (uart_receiver.channel[1] - 832) / 200.0f;
        yaw_test_bias = (uart_receiver.channel[0] - 832) * 1.5;
        if ((uart_receiver.channel[3] == 1792 || uart_receiver.channel[3] == 992 || uart_receiver.channel[3] == 192) && test_state == 0)
        {
            test_state = 1;
        }
        handle_channel();
        uart_receiver.finsh_flag = 0;
    }
}

void calculate_route(void)
{
    int temp = road2->data_int;
    uint8_t digits[5] = {0};
    int count = 0;

    if (temp == 0)
    {
        digits[0] = 0;
        count = 1;
    }
    else
    {
        while (temp > 0 && count < 5)
        {
            digits[count] = (uint8_t)(temp % 10);
            count++;
            temp /= 10;
        }
    }

    for (int i = 0; i < 5; i++)
    {
        if (i < count)
        {
            road_idx[i] = digits[count - 1 - i];
        }
        else
        {
            road_idx[i] = 0;
        }
    }
}

void calculate_modify(void)
{
    int temp = modiro->data_int;
    uint8_t digits[5] = {0};
    int count = 0;

    if (temp == 0)
    {
        digits[0] = 0;
        count = 1;
    }
    else
    {
        while (temp > 0 && count < 5)
        {
            digits[count] = (uint8_t)(temp % 10);
            count++;
            temp /= 10;
        }
    }

    for (int i = 0; i < 5; i++)
    {
        if (i < count)
        {
            modify_idx[i] = digits[count - 1 - i];
        }
        else
        {
            modify_idx[i] = 0;
        }
    }
}

void calculate_uturn_3(void)
{
    int temp = uturn3->data_int;
    uint8_t digits[2] = {0};
    int count = 0;

    if (temp == 0)
    {
        digits[0] = 0;
        count = 1;
    }
    else
    {
        while (temp > 0 && count < 2)
        {
            digits[count] = (uint8_t)(temp % 10);
            count++;
            temp /= 10;
        }
    }

    for (int i = 0; i < 2; i++)
    {
        if (i < count)
        {
            uturn_3_idx[i] = digits[count - 1 - i];
        }
        else
        {
            uturn_3_idx[i] = 0;
        }
    }
}
