#include "imu.h"
#include "small_driver_uart_control.h"

// imu示例（单例）
imu_t imu;

static void read_sensor_data(void);
static void apply_bias_and_cordinate_transform(void);
static void process_calibration(void);
static void process_filter_update(void);


// =====================接口函数=======================
void imu_init(MahonyFilterParam* param, ukf_filter_param_t* ukf_param)
{
    imu660ra_init();
    memset(&imu, 0, sizeof(imu_t));
    imu.state = IMU_STATE_UNINIT;

    Mahony_Init(&imu.mahony_filter, param);
    bool ret = ukf_filter_init(&imu.ukf_filter, ukf_param);
    if(ret)
    {
        imu.state = IMU_STATE_ERROR;
        return;
    }
    pit_ms_init(PIT_CH0, IMU_UPDATE_INTERVAL_MS);
}

void imu_reset_mahony_filter(MahonyFilterParam* param)
{
    Mahony_Init(&imu.mahony_filter, param);
}

void imu_reset_ukf_filter(ukf_filter_param_t* param)
{
    bool ret = ukf_filter_init(&imu.ukf_filter, param);
    if(ret)
    {
        imu.state = IMU_STATE_ERROR;
    }
}

void imu_start_calibration(uint32_t target_samples)
{
    // 清空校准数据
    imu.calib.target_samples = target_samples;
    imu.calib.sample_count = 0;
    imu.calib.is_calibrated = false;
    imu.calib.gyro_bias[0] = 0.0f;
    imu.calib.gyro_bias[1] = 0.0f;
    imu.calib.gyro_bias[2] = 0.0f;
    // 进入校准状态
    imu.state = IMU_STATE_CALIBRATING;
}


bool imu_is_calibrated(void)
{
    return imu.calib.is_calibrated;
}


// imu更新函数（中断中调用）
void imu_update(void)
{
    read_sensor_data();

    switch (imu.state)
    {
        case IMU_STATE_CALIBRATING:
        {
            process_calibration();
            break;
        }
        case IMU_STATE_RUNNING:
            apply_bias_and_cordinate_transform();
            process_filter_update();
            break;
        default:
            return; // 非法状态，直接返回
    }
    
}


// =====================内部函数=======================
// 读取原始传感器数据
static void read_sensor_data(void)
{
    imu660ra_get_acc();
    imu660ra_get_gyro();

    imu.raw.acc[0] = imu660ra_acc_transition(imu660ra_acc_x);
    imu.raw.acc[1] = imu660ra_acc_transition(imu660ra_acc_y);
    imu.raw.acc[2] = imu660ra_acc_transition(imu660ra_acc_z);

    imu.raw.gyro[0] = imu660ra_gyro_transition(imu660ra_gyro_x);
    imu.raw.gyro[1] = imu660ra_gyro_transition(imu660ra_gyro_y);
    imu.raw.gyro[2] = imu660ra_gyro_transition(imu660ra_gyro_z);
}


// 按照传感器安装方向转换到机体坐标系
static void apply_bias_and_cordinate_transform(void)
{
    float corrected_gyro[3];
    // 减去陀螺仪零偏
    corrected_gyro[0] = imu.raw.gyro[0] - imu.calib.gyro_bias[0];
    corrected_gyro[1] = imu.raw.gyro[1] - imu.calib.gyro_bias[1];
    corrected_gyro[2] = imu.raw.gyro[2] - imu.calib.gyro_bias[2];

    // 转换至FLU坐标系
    imu.body.acc[0] = imu.raw.acc[0];
    imu.body.acc[1] = -imu.raw.acc[1];
    imu.body.acc[2] = -imu.raw.acc[2];

    imu.body.gyro[0] = corrected_gyro[0];
    imu.body.gyro[1] = -corrected_gyro[1];
    imu.body.gyro[2] = -corrected_gyro[2];
}


static void process_calibration(void)
{
    // 累加陀螺仪数据
    imu.calib.gyro_bias[0] += imu.raw.gyro[0];
    imu.calib.gyro_bias[1] += imu.raw.gyro[1];
    imu.calib.gyro_bias[2] += imu.raw.gyro[2];
    imu.calib.sample_count++;
    // 检查是否完成校准
    if (imu.calib.sample_count >= imu.calib.target_samples)
    {
        // 计算平均偏置
        imu.calib.gyro_bias[0] /= imu.calib.target_samples;
        imu.calib.gyro_bias[1] /= imu.calib.target_samples;
        imu.calib.gyro_bias[2] /= imu.calib.target_samples;
        imu.calib.is_calibrated = true;
        imu.state = IMU_STATE_RUNNING;
    }
}


static void process_filter_update(void)
{
    Mahony_Update(&imu.mahony_filter, imu.body.gyro, imu.body.acc);
    Mahony_GetEuler(&imu.mahony_filter, &imu.euler[0], &imu.euler[1], &imu.euler[2]);
    // imu_angle_update();     // 测试用互补滤波姿态更新
    float ukf_measurement[3];
    imu.body.speed = ((float)motor_value.receive_left_speed_data -
        (float)motor_value.receive_right_speed_data) / 2.0f * PI * 0.065f / 60.0f; // 轮速转换为线速度，单位m/s
    ukf_measurement[0] = imu.body.speed;      // 速度
    ukf_measurement[1] = imu.euler[2] * DEG_TO_RAD;       // 航向角（弧度）
    ukf_measurement[2] = imu.body.gyro[2] * DEG_TO_RAD;   // 角速度（弧度/s）
    ukf_filter_update(&imu.ukf_filter, ukf_measurement);
}
