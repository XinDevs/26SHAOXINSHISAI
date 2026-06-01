/**
 * @file    encoder.c
 * @brief   双轮编码器测速驱动
 * @details 基于MSPM0定时器捕获模式，读取左右轮编码器脉冲并换算为线速度(m/s)和转速(RPM)。
 *          采样周期20ms，支持13PPR×20减速比×2倍频的编码器参数。
 */

#include "encoder.h"

#include "ti_msp_dl_config.h"
#include "dc_motor.h"
#include <math.h>

/* 左轮编码器: E1B 测速
 * 右轮编码器: E2A 测速
 */
#define ENCODER_LEFT_MEAS_INST            E1B_INST
#define ENCODER_LEFT_MEAS_INST_INT_IRQN   E1B_INST_INT_IRQN

#define ENCODER_RIGHT_MEAS_INST           E2A_INST
#define ENCODER_RIGHT_MEAS_INST_INT_IRQN  E2A_INST_INT_IRQN

#define ENCODER_PI                   (3.1415926f)
#define ENCODER_TWO_PI               (6.2831852f)

static volatile int32_t g_left_count  = 0;
static volatile int32_t g_right_count = 0;

static volatile float g_left_speed_mps  = 0.0f;
static volatile float g_right_speed_mps = 0.0f;

static volatile float g_left_rpm  = 0.0f;
static volatile float g_right_rpm = 0.0f;

static volatile uint8_t g_speed_update_flag = 0;

/* 里程计累计量 */
static volatile float g_left_distance_m = 0.0f;
static volatile float g_right_distance_m = 0.0f;
static volatile float g_center_distance_m = 0.0f;
static volatile float g_total_distance_m = 0.0f;
static volatile float g_odom_x_m = 0.0f;
static volatile float g_odom_y_m = 0.0f;
static volatile float g_odom_heading_rad = 0.0f;

/* 用于左右轮采样配对更新里程计位姿 */
static volatile float g_left_delta_m = 0.0f;
static volatile float g_right_delta_m = 0.0f;
static volatile uint8_t g_left_delta_ready = 0U;
static volatile uint8_t g_right_delta_ready = 0U;

/**
 * @brief  将采样窗口内脉冲数换算为转速 RPM
 * @param  count 采样窗口内累计脉冲数
 * @retval 对应电机轴转速 (RPM)
 */
static float encoder_count_to_rpm(int32_t count)
{
    const float rev_per_sample = ((float) count) /
                                 (ENCODER_PPR * ENCODER_GEAR_RATIO * ENCODER_EDGE_FACTOR);
    return rev_per_sample * (60.0f / ENCODER_SAMPLE_PERIOD_S);
}

/**
 * @brief  将采样窗口内脉冲数换算为位移 m
 * @param  count 采样窗口内累计脉冲数
 * @retval 对应轮周位移 (m)
 */
static float encoder_count_to_distance_m(int32_t count)
{
    const float rev_per_sample = ((float) count) /
                                 (ENCODER_PPR * ENCODER_GEAR_RATIO * ENCODER_EDGE_FACTOR);
    return rev_per_sample * (ENCODER_PI * ENCODER_WHEEL_DIAMETER_M);
}

/**
 * @brief  将角度限制到 (-pi, pi]
 */
static float encoder_wrap_pi(float rad)
{
    while (rad > ENCODER_PI) {
        rad -= ENCODER_TWO_PI;
    }
    while (rad <= -ENCODER_PI) {
        rad += ENCODER_TWO_PI;
    }
    return rad;
}

/**
 * @brief  将电机方向映射为位移符号
 */
static float encoder_direction_to_sign(DCMotor_Direction_t dir)
{
    switch (dir) {
        case DCMOTOR_DIR_FORWARD:
            return 1.0f;
        case DCMOTOR_DIR_BACKWARD:
            return -1.0f;
        case DCMOTOR_DIR_STOP:
        default:
            return 0.0f;
    }
}

/**
 * @brief  在左右轮都提交了本周期增量后，更新中心里程与位姿
 */
static void encoder_update_odometry_on_pair(void)
{
    float left_delta_m;
    float right_delta_m;
    float center_delta_m;
    float delta_heading_rad;
    float mid_heading_rad;

    if ((g_left_delta_ready == 0U) || (g_right_delta_ready == 0U)) {
        return;
    }

    left_delta_m = g_left_delta_m;
    right_delta_m = g_right_delta_m;
    g_left_delta_ready = 0U;
    g_right_delta_ready = 0U;

    center_delta_m = 0.5f * (left_delta_m + right_delta_m);

    if (ENCODER_WHEEL_TRACK_M > 0.0f) {
        delta_heading_rad = (right_delta_m - left_delta_m) / ENCODER_WHEEL_TRACK_M;
    } else {
        delta_heading_rad = 0.0f;
    }

    mid_heading_rad = g_odom_heading_rad + (0.5f * delta_heading_rad);

    g_center_distance_m += center_delta_m;
    g_total_distance_m += (center_delta_m >= 0.0f) ? center_delta_m : -center_delta_m;
    g_odom_x_m += center_delta_m * cosf(mid_heading_rad);
    g_odom_y_m += center_delta_m * sinf(mid_heading_rad);
    g_odom_heading_rad = encoder_wrap_pi(g_odom_heading_rad + delta_heading_rad);
}

/**
 * @brief  将转速 RPM 换算为线速度 m/s
 * @param  rpm 转速 (RPM)
 * @retval 对应轮周线速度 (m/s)
 */
static float rpm_to_mps(float rpm)
{
    const float wheel_circ = ENCODER_PI * ENCODER_WHEEL_DIAMETER_M;
    return (rpm / 60.0f) * wheel_circ;
}

/**
 * @brief  左轮捕获沿计数
 * @note   在输入捕获中断中每次有效边沿触发时调用。
 */
static void encoder_left_capture_on_edge(void)
{
    g_left_count++;
}

/**
 * @brief  右轮捕获沿计数
 * @note   在输入捕获中断中每次有效边沿触发时调用。
 */
static void encoder_right_capture_on_edge(void)
{
    g_right_count++;
}

/**
 * @brief  提交左轮一次采样结果
 * @note   将累计脉冲清零，并更新左轮 RPM 与 m/s。
 */
static void encoder_left_commit_sample(void)
{
    int32_t pulse_count = g_left_count;
    DCMotor_Status_t motor_status;
    float left_sign;
    float left_delta_m;

    g_left_count = 0;
    g_left_rpm = encoder_count_to_rpm(pulse_count);
    g_left_speed_mps = rpm_to_mps(g_left_rpm);

    DCMotor_GetStatus(&motor_status);
    left_sign = encoder_direction_to_sign(motor_status.left_dir);
    left_delta_m = encoder_count_to_distance_m(pulse_count) * left_sign;
    g_left_distance_m += left_delta_m;
    g_left_delta_m = left_delta_m;
    g_left_delta_ready = 1U;
    encoder_update_odometry_on_pair();
    g_speed_update_flag = 1U;
}

/**
 * @brief  提交右轮一次采样结果
 * @note   将累计脉冲清零，并更新右轮 RPM 与 m/s。
 */
static void encoder_right_commit_sample(void)
{
    int32_t pulse_count = g_right_count;
    DCMotor_Status_t motor_status;
    float right_sign;
    float right_delta_m;

    g_right_count = 0;
    g_right_rpm = encoder_count_to_rpm(pulse_count);
    g_right_speed_mps = rpm_to_mps(g_right_rpm);

    DCMotor_GetStatus(&motor_status);
    right_sign = encoder_direction_to_sign(motor_status.right_dir);
    right_delta_m = encoder_count_to_distance_m(pulse_count) * right_sign;
    g_right_distance_m += right_delta_m;
    g_right_delta_m = right_delta_m;
    g_right_delta_ready = 1U;
    encoder_update_odometry_on_pair();
    g_speed_update_flag = 1U;
}

/**
 * @brief  编码器驱动初始化
 * @note   清零内部状态，开启左右测速定时器与中断。
 */
void encoder_init(void)
{
    g_left_count  = 0;
    g_right_count = 0;
    g_left_speed_mps  = 0.0f;
    g_right_speed_mps = 0.0f;
    g_left_rpm  = 0.0f;
    g_right_rpm = 0.0f;
    g_speed_update_flag = 0;

    g_left_distance_m = 0.0f;
    g_right_distance_m = 0.0f;
    g_center_distance_m = 0.0f;
    g_total_distance_m = 0.0f;
    g_odom_x_m = 0.0f;
    g_odom_y_m = 0.0f;
    g_odom_heading_rad = 0.0f;
    g_left_delta_m = 0.0f;
    g_right_delta_m = 0.0f;
    g_left_delta_ready = 0U;
    g_right_delta_ready = 0U;

    NVIC_ClearPendingIRQ(ENCODER_LEFT_MEAS_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODER_RIGHT_MEAS_INST_INT_IRQN);

    NVIC_EnableIRQ(ENCODER_LEFT_MEAS_INST_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_RIGHT_MEAS_INST_INT_IRQN);

    DL_TimerA_startCounter(ENCODER_LEFT_MEAS_INST);
    DL_TimerG_startCounter(ENCODER_RIGHT_MEAS_INST);
}

/**
 * @brief  获取左轮线速度
 * @retval 左轮速度 (m/s)
 */
float encoder_get_left_speed_mps(void)
{
    return g_left_speed_mps;
}

/**
 * @brief  获取右轮线速度
 * @retval 右轮速度 (m/s)
 */
float encoder_get_right_speed_mps(void)
{
    return g_right_speed_mps;
}

/**
 * @brief  获取左轮转速
 * @retval 左轮转速 (RPM)
 */
float encoder_get_left_rpm(void)
{
    return g_left_rpm;
}

/**
 * @brief  获取右轮转速
 * @retval 右轮转速 (RPM)
 */
float encoder_get_right_rpm(void)
{
    return g_right_rpm;
}

/**
 * @brief  清零里程计累计量（位移与位姿）
 */
void encoder_reset_odometry(void)
{
    __disable_irq();
    g_left_distance_m = 0.0f;
    g_right_distance_m = 0.0f;
    g_center_distance_m = 0.0f;
    g_total_distance_m = 0.0f;
    g_odom_x_m = 0.0f;
    g_odom_y_m = 0.0f;
    g_odom_heading_rad = 0.0f;
    g_left_delta_m = 0.0f;
    g_right_delta_m = 0.0f;
    g_left_delta_ready = 0U;
    g_right_delta_ready = 0U;
    __enable_irq();
}

/**
 * @brief  获取左轮累计位移
 */
float encoder_get_left_distance_m(void)
{
    return g_left_distance_m;
}

/**
 * @brief  获取右轮累计位移
 */
float encoder_get_right_distance_m(void)
{
    return g_right_distance_m;
}

/**
 * @brief  获取车体中心累计位移
 */
float encoder_get_center_distance_m(void)
{
    return g_center_distance_m;
}

/**
 * @brief  获取累计路程
 */
float encoder_get_total_distance_m(void)
{
    return g_total_distance_m;
}

/**
 * @brief  获取里程计快照
 */
void encoder_get_odometry_snapshot(EncoderOdometry_t *odomOut)
{
    if (odomOut == 0) {
        return;
    }

    __disable_irq();
    odomOut->left_distance_m = g_left_distance_m;
    odomOut->right_distance_m = g_right_distance_m;
    odomOut->center_distance_m = g_center_distance_m;
    odomOut->total_distance_m = g_total_distance_m;
    odomOut->x_m = g_odom_x_m;
    odomOut->y_m = g_odom_y_m;
    odomOut->heading_rad = g_odom_heading_rad;
    __enable_irq();
}

/**
 * @brief  查询速度是否有新采样
 * @retval 1: 有新数据
 * @retval 0: 无新数据
 */
uint8_t encoder_is_updated(void)
{
    return g_speed_update_flag;
}

/**
 * @brief  清除速度更新标志
 */
void encoder_clear_update_flag(void)
{
    g_speed_update_flag = 0;
}

/**
 * @brief  左轮编码器中断服务函数
 * @note   处理捕获沿计数和采样周期提交事件。
 */
void E1B_INST_IRQHandler(void)
{
    /* MSPM0 Capture 特性：
     * - Up-Down 模式：CC0_UP/CC0_DN 会按计数方向分流；
     * - 单向计数模式：所有捕获事件都统一进入 CC0_DN。
     * 因此 CC0_DN 必须处理，CC0_UP 作为 Up-Down 兼容入口。
     */
    switch (DL_TimerA_getPendingInterrupt(ENCODER_LEFT_MEAS_INST)) {
        case DL_TIMER_IIDX_CC0_DN:
        case DL_TIMER_IIDX_CC0_UP:
            encoder_left_capture_on_edge();
            break;

        case DL_TIMER_IIDX_ZERO:
            encoder_left_commit_sample();
            break;

        default:
            break;
    }
}

/**
 * @brief  右轮编码器中断服务函数
 * @note   处理捕获沿计数和采样周期提交事件。
 */
void E2A_INST_IRQHandler(void)
{
    /* 与左轮同理：单向计数下捕获统一落在 CC0_DN */
    switch (DL_TimerG_getPendingInterrupt(ENCODER_RIGHT_MEAS_INST)) {
        case DL_TIMER_IIDX_CC0_UP:
        case DL_TIMER_IIDX_CC0_DN:
            encoder_right_capture_on_edge();
            break;

        case DL_TIMER_IIDX_ZERO:
            encoder_right_commit_sample();
            break;

        default:
            break;
    }
}
