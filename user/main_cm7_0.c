#include "zf_common_headfile.h"
#include "imu.h"
#include "small_driver_uart_control.h"
#include "screen_interface.h"
#include "ahrs_ui.h"
#include "menu.h"
#include "image.h"
#include "servo.h"
#include "control.h"
#include "control_state.h"
#include "connecting_rod.h"
#include "save_manager.h"
#include "pid.h"

#define WIFI_SSID_TEST "Hhhyt"
#define WIFI_PASSWORD_TEST "18273645"
#define UDP_TARGET_IP "192.168.137.1"
#define UDP_TARGET_PORT "8086"
#define WIFI_LOCAL_PORT "6666"

#define BUTTON_LF (P20_0)
#define BUTTON_RT (P20_1)
#define BUTTON_DN (P20_2)
#define BUTTON_UP (P20_3)
#define BUTTON_OK (P21_5)

void gpio_init_all(void);
void imu_state_show(void);
void start_imu_calibration(void);

void wifi_init(void);
void wifi_send_data(void);

uint8_t motion_mode1 = 0;
uint8_t motion_mode2 = 0;

MahonyFilterParam mahony_filter_param = {
    .Kp = 0.7f,
    .Ki = 0.005f,
    .dt = IMU_UPDATE_INTERVAL_MS / 1000.0f};

ukf_filter_param_t ukf_filter_param = {
    .dt = IMU_UPDATE_INTERVAL_MS / 1000.0,
    .q = {0.0001, 0.0001, 0.000015, 0.0002, 0.0005},
    .r = {0.0001, 0.05, 0.001}};

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
    imu_init(&mahony_filter_param, &ukf_filter_param);

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
                wifi_send_data();
            }
        }

        else if ((motion_mode1 == 1 && motion_mode2 == 0)
              || (motion_mode1 == 0 && motion_mode2 == 0))
        {
            ips200_clear();
            current_state = STATE_STAND_UP;
            while (true)
            {
                // 跑
                if (image_block == 1)
                {
                    //image_main();
                }
                // 如果需要，添加wifi
                wifi_send_data();
            }
        }
    }
}

void start_imu_calibration(void)
{
    system_delay_ms(100);       // 确保系统稳定
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

void wifi_init(void)
{
    while (wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);
    }

    if (0 == WIFI_SPI_AUTO_CONNECT)
    {
        while (wifi_spi_socket_connect(
            "UDP",
            UDP_TARGET_IP,
            UDP_TARGET_PORT,
            WIFI_LOCAL_PORT))
        {
            printf("\r\n Connect UDP Servers error, try again.");
            system_delay_ms(100);
        }
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
}

void wifi_send_data(void)
{
    seekfree_assistant_oscilloscope_struct oscilloscope_data;
    oscilloscope_data.data[0] = imu.output.euler[2];
    oscilloscope_data.channel_num = 1;
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
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
    case IMU_STATE_RUNNING:
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
