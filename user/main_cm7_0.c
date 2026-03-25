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

#define BUTTON_LF (P20_0)
#define BUTTON_RT (P20_1)
#define BUTTON_DN (P20_2)
#define BUTTON_UP (P20_3)
#define BUTTON_OK (P21_5)

void gpio_init_all(void);
void imu_state_show(void);
void start_imu_calibration(void);

void remote_control(void);

uint8_t motion_mode1 = 0;
uint8_t motion_mode2 = 0;

MahonyFilterParam mahony_filter_param = {
    .Kp = 0.7f,
    .Ki = 0.005f,
    .dt = IMU_UPDATE_INTERVAL_MS / 1000.0f};

RotationCompensationParam rotation_compensation_param = {
    .rotation_thre = 180,
    .yaw_compensate = 2.8};

bool showing_points = false;
// 用于实现校准后的分层启动
bool car_run_state = false;

uint8_t cali_flag = 1;

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
    menu_load1();

    // 存点初始化
    save_manager_init(ins_to_control_menu);

    // 如有需要，添加wifi
    wifi_init();

    // 屏幕初始化
    pit_ms_init(SCREEN_UPDATE_PIT, SCREEN_UPDATE_INTERVAL_MS);
    pit_disable(SCREEN_UPDATE_PIT);
    screen_init();

    // imu初始化
    pit_ms_init(IMU_UPDATE_PIT, IMU_UPDATE_INTERVAL_MS);
    pit_disable(IMU_UPDATE_PIT);
    imu_init(mahony_filter_param, rotation_compensation_param);

    while (true)
    {
        // 主循环检测拨码的状态
        motion_mode1 = gpio_get_level(P21_6);
        motion_mode2 = gpio_get_level(P21_5);

        if (motion_mode1 == 1 && motion_mode2 == 1)
        {
            current_state = STATE_IDLE;
            if (cali_flag == 1)
            {
                start_imu_calibration();
                cali_flag = 0;
            }
            imu_state_show();

            // 如果需要，添加wifi
            wifi_send_data();
            // wifi_send_image();
        }

        else if (motion_mode1 == 0 && motion_mode2 == 1)
        {
            current_state = STATE_IDLE;
            ips200_clear();
            while (true)
            {
                // 调菜单
                if (menu_current_state == MENU_STATE_CONTROL)
                {
                    show_menu();
                    key_handle_menu();
                }
                else if (menu_current_state == MENU_STATE_INS)
                {
                    key_handle_ins();
                }

                // 如果需要，添加wifi
                // wifi_send_data();
                // wifi_send_image();
            }
        }

        else if ((motion_mode1 == 1 && motion_mode2 == 0) || (motion_mode1 == 0 && motion_mode2 == 0))
        {
            ips200_clear();
            current_state = STATE_STAND_UP;
            while (true)
            {
                // 跑
                remote_control();
                if (image_block == 1)
                {
                    // image_main();
                }

                // 如果需要，添加wifi
                wifi_send_data();
                // wifi_send_image();
            }
        }
    }
}

void start_imu_calibration(void)
{
    system_delay_ms(300);        // 确保系统稳定
    imu_start_calibration(3000); // 采集3000个样本进行校准
}

void gpio_init_all(void)
{
    gpio_init(BUTTON_LF, GPI, 1, GPI_PULL_UP);
    gpio_init(BUTTON_RT, GPI, 1, GPI_PULL_UP);
    gpio_init(BUTTON_DN, GPI, 1, GPI_PULL_UP);
    gpio_init(BUTTON_UP, GPI, 1, GPI_PULL_UP);
    gpio_init(BUTTON_OK, GPI, 1, GPI_PULL_UP);
    gpio_init(P21_6, GPI, 1, GPI_PULL_UP);
}

void imu_state_show(void)
{
    switch (imu.state)
    {
    case IMU_STATE_UNINIT:
        screen_printf(0, 0, "IMU State: UNINIT     ");
        break;
    case IMU_STATE_CALIBRATING:
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

void handle_channel_245(void)
{
    static uint16 last_ch2 = 0, last_ch4 = 0, last_ch5 = 0;

    if (last_ch2 == 0)
    {
        last_ch2 = uart_receiver.channel[2];
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
        // Channel 2: 在末尾添加当前位置点
        waypoint_t point;
        ins_get_current_position(&point.x, &point.y);
        ins_add_current(&waypoints, waypoints.tail, point);
    }

    else if (last_ch4 != uart_receiver.channel[4])
    {
        // Channel 4: 删除末尾的点
        //waypoint_list_remove(&waypoints, waypoints.tail);
        current_state = STATE_THUNDER;
    }

    else if (last_ch5 != uart_receiver.channel[5])
    {
        // Channel 5: 保存到Flash，存入当前激活的存档（开机默认为0号存档）
        waypoint_list_save_to_flash(&waypoints,
                                    INS_FLASH_START_PAGE + save_manager_instance.active_save_idx);
    }
    last_ch2 = uart_receiver.channel[2];
    last_ch4 = uart_receiver.channel[4];
    last_ch5 = uart_receiver.channel[5];
}

void remote_control(void)
{
    if (1 == uart_receiver.finsh_flag)
    {
        speed_loop.Target = (uart_receiver.channel[1] - 800) / 200.0f;
        yaw_loop.Target += (uart_receiver.channel[0] - 800) / 800.0f;
        if (uart_receiver.channel[3] == 1792 && test_state == 0)
        {
            test_state = 1;
        }
        // 2, 4, 5频道当作按钮使用
        handle_channel_245();
        uart_receiver.finsh_flag = 0;
    }
}
