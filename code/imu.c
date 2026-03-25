#include "imu.h"
#include "small_driver_uart_control.h"
#include "ins.h"

// imu示例（单例）
imu_t imu;

static void read_sensor_data(void);
static void read_encoder_data(void);
static void apply_bias_and_cordinate_transform(void);
static void process_calibration(void);
static void process_filter_update(void);


// =====================接口函数=======================
void imu_init(MahonyFilterParam mahony_param, RotationCompensationParam rot_comp_param)
{
    imu660ra_init();
    memset(&imu, 0, sizeof(imu_t));
    imu.state = IMU_STATE_UNINIT;

    imu.rot_comp.param = rot_comp_param;

    Mahony_Init(&imu.mahony_filter, mahony_param);

    pit_enable(IMU_UPDATE_PIT);
}

void imu_reset_mahony_filter(MahonyFilterParam param)
{
    Mahony_Init(&imu.mahony_filter, param);
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
        case IMU_STATE_NORMAL:
        case IMU_STATE_ROTATING:
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
    imu.encoder.v_left = (double)motor_value.receive_left_speed_data * PI * WHEEL_D / 60.0f;
    imu.encoder.v_right = -(double)motor_value.receive_right_speed_data * PI * WHEEL_D / 60.0f;

    imu.encoder.speed = (imu.encoder.v_left + imu.encoder.v_right) / 2.0f;

    imu.encoder.yaw_rate = (imu.encoder.v_right - imu.encoder.v_left) / WHEELBASE * RAD_TO_DEG;
}

static void set_mahony_initial_estimation(void)
{
    // 使用校准期间的平均加速度作为初始姿态估计
    double ax = imu.calib.acc_sum[0] / imu.calib.sample_count;
    double ay = imu.calib.acc_sum[1] / imu.calib.sample_count;
    double az = imu.calib.acc_sum[2] / imu.calib.sample_count;
    // 转换至FLU坐标系
    ax = ax;
    ay = -ay;
    az = -az;
    // 计算初始四元数
    double roll = atan2f(ay, az);
    double pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    double yaw = 0.0f; // 初始航向角假设为0
    double cy = cosf(yaw * 0.5f);
    double sy = sinf(yaw * 0.5f);
    double cp = cosf(pitch * 0.5f);
    double sp = sinf(pitch * 0.5f);
    double cr = cosf(roll * 0.5f);
    double sr = sinf(roll * 0.5f);
    imu.mahony_filter.q[0] = cr * cp * cy + sr * sp * sy; // q0
    imu.mahony_filter.q[1] = sr * cp * cy - cr * sp * sy; // q1
    imu.mahony_filter.q[2] = cr * sp * cy + sr * cp * sy; // q2
    imu.mahony_filter.q[3] = cr * cp * sy - sr * sp * cy; // q3
}

// 按照传感器安装方向转换到机体坐标系
static void apply_bias_and_cordinate_transform(void)
{
    double corrected_gyro[3];
    // 减去陀螺仪零偏
    corrected_gyro[0] = imu.raw.gyro[0] - imu.calib.gyro_bias[0];
    corrected_gyro[1] = imu.raw.gyro[1] - imu.calib.gyro_bias[1];
    corrected_gyro[2] = imu.raw.gyro[2] - imu.calib.gyro_bias[2];

    if(imu.state == IMU_STATE_ROTATING)
    {
        if(fabs(corrected_gyro[2]) < imu.rot_comp.param.rotation_thre)
            imu.rot_comp.exit_cnt++;
        else 
            imu.rot_comp.exit_cnt = 0;
        if(imu.rot_comp.exit_cnt >= 1)     // 连续检测到角速度小于阈值，退出旋转状态
        {
            imu.state = IMU_STATE_NORMAL;
            imu.rot_comp.enter_cnt = 0;
            imu.rot_comp.exit_cnt = 0;
        }
        // 进行旋转补偿
        if(corrected_gyro[2] < 0.0)
            corrected_gyro[2] += imu.rot_comp.param.yaw_compensate_ccw;
        else 
            corrected_gyro[2] += imu.rot_comp.param.yaw_compensate_cw;
    }
    else {
        if(fabs(corrected_gyro[2]) > imu.rot_comp.param.rotation_thre)
            imu.rot_comp.enter_cnt++;
        else 
            imu.rot_comp.enter_cnt = 0;
        if(imu.rot_comp.enter_cnt >= 1)    // 连续检测到角速度大于阈值，进入旋转状态
        {
            imu.state = IMU_STATE_ROTATING;
            imu.rot_comp.enter_cnt = 0;
            imu.rot_comp.exit_cnt = 0;
        }
    }

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
        imu.state = IMU_STATE_NORMAL;
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

    // 简化版惯性导航更新
    ins_update(imu.mahony_filter.output.euler[2] * DEG_TO_RAD, imu.encoder.speed);

    double x, y;
    ins_get_data(&x, &y);

    imu.output.euler[2] = imu.mahony_filter.output.euler[2];
    imu.output.gyro[2] = imu.body.gyro[2];

    imu.output.position_x = x;
    imu.output.position_y = y;
    imu.output.speed = imu.encoder.speed;
}
