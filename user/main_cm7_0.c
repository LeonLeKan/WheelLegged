#include "zf_common_headfile.h"
#include "imu.h"
#include "small_driver_uart_control.h"
#include "screen_interface.h"
#include "ahrs_ui.h"
#include "menu.h"
#include "image.h"
#include "servo.h"
#include "control.h"
#include "connecting_rod.h"

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
void key_handle_point(void);
void send_attitude(void);
void process_param(void);
void update_screen(void);
void start_imu_calibration(void);

void para_zero(void);
void data_load1(void);
void data_load2(void);
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
    .q = {0.01, 0.01, 0.0015, 0.0002, 0.05},
    .r = {0.0001, 1, 3}};

bool showing_points = false;
// 用于实现校准后的分层启动
bool car_run_state = false;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_init();                  // 调试串口信息初始化

    // flash初始化
    flash_init();

    // 串口调试初始化
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);

    // 控制相关初始化
    mode_flag = 0;
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

    // 如有需要，添加wifi
    //wifi_init();

    // 屏幕初始化
    screen_init();

    // imu初始化
    imu_init(&mahony_filter_param, &ukf_filter_param);

    // ahrs初始化
    ahrs_ui_init();

    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;
    while (true)
    {
        // 主循环检测拨码的状态
        motion_mode1 = gpio_get_level(P21_6);
        motion_mode2 = gpio_get_level(P21_5);

        if (motion_mode1 == 1 && motion_mode2 == 1)
        {
            // 解析逐飞助手的数据
            seekfree_assistant_data_analysis();

            send_attitude();
            // process_param();
            if (!showing_points)
                update_screen();
            key_handle_point();

            // 如果需要，添加wifi
            // wifi_send_data();
        }

        else if (motion_mode1 == 1 && motion_mode2 == 0)
        {
            ips200_clear();
            while (true)
            {
                // 调菜单
                show_menu();
                key_handle_menu();

                // 如果需要，添加wifi
                wifi_send_data();
            }
        }

        else if (motion_mode1 == 0 && motion_mode2 == 1)
        {
            ips200_clear();
            while (true)
            {
                // 跑
                image_main();

                if (!car_run_state)
                {
                    system_delay_ms(5000);
                    an1pid_update(an1kp->data_float, an1ki->data_float, an1kd->data_float);
                    system_delay_ms(500);
                    mode_flag = 1;
                    data_load1();
                    system_delay_ms(500);
                    data_load2();

                    car_run_state = true;
                }

                // 如果需要，添加wifi
                //wifi_send_data();
            }
        }
    }
}

void send_attitude(void)
{
    seekfree_assistant_oscilloscope_data.data[0] = imu.output.position_x;
    seekfree_assistant_oscilloscope_data.data[1] = imu.output.position_y;

    seekfree_assistant_oscilloscope_data.data[2] = imu.body.gyro[2];
    seekfree_assistant_oscilloscope_data.data[3] = imu.encoder.yaw_rate;
    seekfree_assistant_oscilloscope_data.data[4] = imu.output.gyro[2];

    seekfree_assistant_oscilloscope_data.data[5] = imu.output.euler[2];

    double *P = imu.ukf_filter.ukfIO.update.Pxx.val;
    seekfree_assistant_oscilloscope_data.data[6] = P[0] + P[5 + 1] + P[10 + 2] + P[15 + 3] + P[20 + 4];
    seekfree_assistant_oscilloscope_data.data[7] = imu.output.speed;

    seekfree_assistant_oscilloscope_data.channel_num = 8;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

void process_param(void)
{
    if (seekfree_assistant_parameter_update_flag[0])
    {
        seekfree_assistant_parameter_update_flag[0] = 0;
        ukf_filter_param.q[0] = seekfree_assistant_parameter[0];
        imu_reset_ukf_filter(&ukf_filter_param);
    }
    if (seekfree_assistant_parameter_update_flag[1])
    {
        seekfree_assistant_parameter_update_flag[1] = 0;
        ukf_filter_param.q[1] = seekfree_assistant_parameter[1];
        imu_reset_ukf_filter(&ukf_filter_param);
    }
    if (seekfree_assistant_parameter_update_flag[2])
    {
        seekfree_assistant_parameter_update_flag[2] = 0;
        ukf_filter_param.q[2] = seekfree_assistant_parameter[2];
        imu_reset_ukf_filter(&ukf_filter_param);
    }
    if (seekfree_assistant_parameter_update_flag[3])
    {
        seekfree_assistant_parameter_update_flag[3] = 0;
        ukf_filter_param.q[3] = seekfree_assistant_parameter[3];
        imu_reset_ukf_filter(&ukf_filter_param);
    }
    if (seekfree_assistant_parameter_update_flag[4])
    {
        seekfree_assistant_parameter_update_flag[4] = 0;
        ukf_filter_param.q[4] = seekfree_assistant_parameter[4];
        imu_reset_ukf_filter(&ukf_filter_param);
    }
    if (seekfree_assistant_parameter_update_flag[5])
    {
        seekfree_assistant_parameter_update_flag[5] = 0;
        ukf_filter_param.r[0] = seekfree_assistant_parameter[5];
        imu_reset_ukf_filter(&ukf_filter_param);
    }
    if (seekfree_assistant_parameter_update_flag[6])
    {
        seekfree_assistant_parameter_update_flag[6] = 0;
        ukf_filter_param.r[1] = seekfree_assistant_parameter[6];
        imu_reset_ukf_filter(&ukf_filter_param);
    }
    if (seekfree_assistant_parameter_update_flag[7])
    {
        seekfree_assistant_parameter_update_flag[7] = 0;
        ukf_filter_param.r[2] = seekfree_assistant_parameter[7];
        imu_reset_ukf_filter(&ukf_filter_param);
    }
}

void start_imu_calibration(void)
{
    system_delay_ms(1000);  // 等待1秒，确保系统稳定
    imu_start_calibration(3000);  // 采集3000个样本进行校准
}

void update_screen(void)
{
    switch (imu.state)
    {
    case IMU_STATE_UNINIT:
        screen_printf(0, 16, "IMU State: UNINIT     ");
        break;
    case IMU_STATE_CALIBRATING:
        screen_printf(0, 16, "IMU State: CALIBRATING");
        break;
    case IMU_STATE_RUNNING:
        screen_printf(0, 16, "IMU State: RUNNING    ");
        break;
    case IMU_STATE_ERROR:
        screen_printf(0, 16, "IMU State: ERROR      ");
        break;
    default:
        screen_printf(0, 16, "IMU State: UNKNOWN    ");
        break;
    }
    screen_printf(0, 32, "Yaw: %6.4f deg   ", imu.ukf_filter.output.yaw);
    screen_printf(0, 48, "PosX: %8.4f m   ", imu.ukf_filter.output.position_x);
    screen_printf(0, 64, "PosY: %8.4f m   ", imu.ukf_filter.output.position_y);
    screen_printf(0, 80, "Speed: %6.4f m/s ", imu.ukf_filter.output.speed);
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

void key_handle_point(void)
{
    static uint8_t last_switch_state = 1; // 上电前拉回高电平
    uint8_t current_switch_state = gpio_get_level(BUTTON_OK);

    uint8_t switch_changed = (last_switch_state != current_switch_state);

    if (switch_changed)
    {
        last_switch_state = current_switch_state;
        if (showing_points)
            points_display_key_ok_pressed();
        else
        {
            showing_points = true;
            ahrs_ui_show();
        }
    }
    else if (gpio_get_level(BUTTON_LF) == 0)
    {
        system_delay_ms(80);
        if (gpio_get_level(BUTTON_LF) == 0)
        {
            if (showing_points)
                points_display_key_left_pressed();
        }
    }
    else if (gpio_get_level(BUTTON_RT) == 0)
    {
        system_delay_ms(80);
        if (gpio_get_level(BUTTON_RT) == 0)
        {
            if (showing_points)
                points_display_key_right_pressed();
        }
    }
    else if (gpio_get_level(BUTTON_DN) == 0)
    {
        system_delay_ms(80);
        if (gpio_get_level(BUTTON_DN) == 0)
        {
            if (showing_points)
                points_display_key_down_pressed();
        }
    }
    else if (gpio_get_level(BUTTON_UP) == 0)
    {
        system_delay_ms(80);
        if (gpio_get_level(BUTTON_UP) == 0)
        {
            if (showing_points)
                points_display_key_up_pressed();
            else
                start_imu_calibration();
        }
    }
}

void para_zero(void)
{
    an1_zero();
    an2_zero();
    sp_zero();
    tu1_zero();
    tu2_zero();
    as_zero();
    sn_zero();
}

void data_load1(void)
{
    an2pid_update(an2kp->data_float, an2ki->data_float, an2kd->data_float);
    aspid_update(askp->data_float, aski->data_float, askd->data_float);
}

void data_load2(void)
{
    sppid_update(spkp->data_float, spki->data_float, spkd->data_float);
    tu1pid_update(tu1kp->data_float, tu1ki->data_float, tu1kd->data_float);
    tu2pid_update(tu2kp->data_float, tu2ki->data_float, tu2kd->data_float);
    snpid_update(snkp->data_float, snki->data_float, snkd->data_float);
    // threshold = (uint8_t)thre->data_int;
    error_calc_far_line = (uint8_t)farl->data_int;
    error_calc_near_line = (uint8_t)nearl->data_int;
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
    oscilloscope_data.data[0] = speed_loop.Target;
    oscilloscope_data.data[1] = speed_loop.Actual;
    oscilloscope_data.data[2] = angle_loop2.Target;
    oscilloscope_data.data[3] = angle_loop2.Actual;
    oscilloscope_data.data[4] = angle_loop2.ErrorInt;
    oscilloscope_data.channel_num = 5;
    seekfree_assistant_oscilloscope_send(&oscilloscope_data);
}