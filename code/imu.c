#include "imu.h"
#include "small_driver_uart_control.h"

// imu示例（单例）
imu_t imu;

static void read_sensor_data(void);
static void read_encoder_data(void);
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
    pit_enable(IMU_UPDATE_PIT);
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
    read_encoder_data();

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

// 从编码器获取速度和yaw角速度
static void read_encoder_data(void)
{
    imu.encoder.v_left = (float)motor_value.receive_left_speed_data * PI * WHEEL_D / 60.0f;
    imu.encoder.v_right = -(float)motor_value.receive_right_speed_data * PI * WHEEL_D / 60.0f;

    imu.encoder.speed = (imu.encoder.v_left + imu.encoder.v_right) / 2.0f;

    imu.encoder.yaw_rate = (imu.encoder.v_right - imu.encoder.v_left) / WHEELBASE * RAD_TO_DEG;
}

static void set_mahony_initial_estimation(void)
{
    // 使用校准期间的平均加速度作为初始姿态估计
    float ax = imu.calib.acc_sum[0] / imu.calib.sample_count;
    float ay = imu.calib.acc_sum[1] / imu.calib.sample_count;
    float az = imu.calib.acc_sum[2] / imu.calib.sample_count;
    // 转换至FLU坐标系
    ax = ax;
    ay = -ay;
    az = -az;
    // 计算初始四元数
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float yaw = 0.0f; // 初始航向角假设为0
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    imu.mahony_filter.q[0] = cr * cp * cy + sr * sp * sy; // q0
    imu.mahony_filter.q[1] = sr * cp * cy - cr * sp * sy; // q1
    imu.mahony_filter.q[2] = cr * sp * cy + sr * cp * sy; // q2
    imu.mahony_filter.q[3] = cr * cp * sy - sr * sp * cy; // q3
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
    // 累加加速度数据（用于初始姿态估计）
    imu.calib.acc_sum[0] += imu.raw.acc[0];
    imu.calib.acc_sum[1] += imu.raw.acc[1];
    imu.calib.acc_sum[2] += imu.raw.acc[2];
    imu.calib.sample_count++;
    // 检查是否完成校准
    if (imu.calib.sample_count >= imu.calib.target_samples)
    {
        // 计算平均偏置
        imu.calib.gyro_bias[0] /= imu.calib.target_samples;
        imu.calib.gyro_bias[1] /= imu.calib.target_samples;
        imu.calib.gyro_bias[2] /= imu.calib.target_samples;
        set_mahony_initial_estimation();
        imu.calib.is_calibrated = true;
        imu.state = IMU_STATE_RUNNING;
    }
}


static void process_filter_update(void)
{
    static uint32_t ukf_update_counter = 0;
    ukf_update_counter++;
    // mahony更新
    Mahony_Update(&imu.mahony_filter, imu.body.gyro, imu.body.acc);

    // roll和pitch直接采用mahony输出
    imu.output.euler[0] = imu.mahony_filter.output.euler[0];
    imu.output.euler[1] = imu.mahony_filter.output.euler[1];
    // roll和pitch角速度直接采用校准后数据
    imu.output.gyro[0] = imu.body.gyro[0];
    imu.output.gyro[1] = imu.body.gyro[1];

    if (ukf_update_counter * IMU_UPDATE_INTERVAL_MS >= UKF_UPDATE_INTERVAL_MS)
    {
        ukf_update_counter = 0;
    }
    else
    {
        return; // 不到更新周期，跳过UKF更新
    }
    // ukf更新
    float ukf_measurement[3];
    ukf_measurement[0] = imu.encoder.speed;                     // 速度
    // mahony解算的yaw角输入ukf
    ukf_measurement[1] = imu.body.gyro[2] * DEG_TO_RAD;         // IMU角速度 (rad/s)
    ukf_measurement[2] = imu.encoder.yaw_rate * DEG_TO_RAD;     // 编码器角速度 (rad/s)
    ukf_filter_update(&imu.ukf_filter, ukf_measurement);

    // yaw和yaw角速度采用ukf输出
    imu.output.euler[2] = imu.ukf_filter.output.yaw;
    imu.output.gyro[2] = imu.mahony_filter.output.euler[2];
    // 位置和速度采用ukf输出
    imu.output.position_x = imu.ukf_filter.output.position_x;
    imu.output.position_y = imu.ukf_filter.output.position_y;
    imu.output.speed = imu.ukf_filter.output.speed;
}
