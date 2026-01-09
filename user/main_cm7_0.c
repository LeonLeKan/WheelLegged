#include "zf_common_headfile.h"
#include "imu.h"
#include "small_driver_uart_control.h"
#include "screen_interface.h"
#include "ahrs_ui.h"


#define BUTTON_LF (P20_0)
#define BUTTON_RT (P20_1)
#define BUTTON_DN (P20_2)
#define BUTTON_UP (P20_3)
#define BUTTON_OK (P21_5)


void gpio_init_all(void);
void key_handle(void);
void send_attitude(void);
void process_param(void);
void update_screen(void);
void start_imu_calibration(void);


MahonyFilterParam mahony_filter_param = {
    .Kp = 0.7f,
    .Ki = 0.005f,
    .dt = IMU_UPDATE_INTERVAL_MS / 1000.0f
};

ukf_filter_param_t ukf_filter_param = {
    .dt = IMU_UPDATE_INTERVAL_MS / 1000.0,
    .q = {0.01, 0.01, 0.0015, 0.02, 0.05},
    .r = {0.0001, 0.05, 0.001}
};

bool showing_points = false;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    gpio_init_all();

    screen_init();     // 屏幕初始化

    // imu初始化
    small_driver_uart_init();
    imu_init(&mahony_filter_param, &ukf_filter_param);

    ahrs_ui_init();

    while (true)
    {
        seekfree_assistant_data_analysis();
        
        send_attitude();
        process_param();
        if(!showing_points) update_screen();
        key_handle();
        // 此处编写需要循环执行的代码
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

    double *P = imu.ukf_filter.ukfIO.predict.P_m.val;
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
}

void key_handle(void)
{
    static uint8_t last_switch_state = 1;  // 上电前拉回高电平
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
            else start_imu_calibration();
        }

    }
}
