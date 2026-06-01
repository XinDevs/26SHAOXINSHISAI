/**
 * @file    dc_motor.c
 * @brief   双路直流电机PWM驱动
 * @details 基于MSPM0定时器PWM输出，控制左右轮直流电机的占空比与方向。
 *          支持正转/反转/停止，可配置左右轮方向反相开关以适配实际接线。
 */

#include "dc_motor.h"

#include "ti_msp_dl_config.h"

/*
 * 方向反相开关（按实际接线调整）
 * 0: 正常
 * 1: 该侧前进/后退逻辑互换
 */
#ifndef DCMOTOR_LEFT_REVERSE
#define DCMOTOR_LEFT_REVERSE   (0)
#endif

#ifndef DCMOTOR_RIGHT_REVERSE
#define DCMOTOR_RIGHT_REVERSE  (0)
#endif

static DCMotor_Status_t s_motor = {
    .left_duty_percent = 0,
    .right_duty_percent = 0,
    .left_dir = DCMOTOR_DIR_STOP,
    .right_dir = DCMOTOR_DIR_STOP,
    .enabled = 0
};

/**
 * @brief  占空比限幅到 [-100, 100]
 * @param  duty 输入占空比百分比
 * @retval 限幅后的占空比
 */
static int16_t DCMotor_ClampDuty(int16_t duty)
{
    if (duty > 100) return 100;
    if (duty < -100) return -100;
    return duty;
}

/**
 * @brief  按反相配置映射电机方向
 * @param  dir 原始方向
 * @param  reverse 反相开关(0:不反相, 1:反相)
 * @retval 映射后的实际方向
 */
static DCMotor_Direction_t DCMotor_ApplyReverse(
    DCMotor_Direction_t dir, uint8_t reverse)
{
    if (!reverse) {
        return dir;
    }
    if (dir == DCMOTOR_DIR_FORWARD) {
        return DCMOTOR_DIR_BACKWARD;
    }
    if (dir == DCMOTOR_DIR_BACKWARD) {
        return DCMOTOR_DIR_FORWARD;
    }
    return DCMOTOR_DIR_STOP;
}

/**
 * @brief  设置左电机方向引脚
 * @param  dir 目标方向
 */
static void DCMotor_SetLeftDirection(DCMotor_Direction_t dir)
{
    DCMotor_Direction_t realDir = DCMotor_ApplyReverse(dir, DCMOTOR_LEFT_REVERSE);

    switch (realDir)
    {
        case DCMOTOR_DIR_FORWARD:
            DL_GPIO_setPins(DC_Motor_AIN1_PORT, DC_Motor_AIN1_PIN);
            DL_GPIO_clearPins(DC_Motor_AIN2_PORT, DC_Motor_AIN2_PIN);
            break;
        case DCMOTOR_DIR_BACKWARD:
            DL_GPIO_clearPins(DC_Motor_AIN1_PORT, DC_Motor_AIN1_PIN);
            DL_GPIO_setPins(DC_Motor_AIN2_PORT, DC_Motor_AIN2_PIN);
            break;
        case DCMOTOR_DIR_STOP:
        default:
            DL_GPIO_clearPins(DC_Motor_AIN1_PORT, DC_Motor_AIN1_PIN);
            DL_GPIO_clearPins(DC_Motor_AIN2_PORT, DC_Motor_AIN2_PIN);
            break;
    }
}

/**
 * @brief  设置右电机方向引脚
 * @param  dir 目标方向
 */
static void DCMotor_SetRightDirection(DCMotor_Direction_t dir)
{
    DCMotor_Direction_t realDir = DCMotor_ApplyReverse(dir, DCMOTOR_RIGHT_REVERSE);

    switch (realDir)
    {
        case DCMOTOR_DIR_FORWARD:
            DL_GPIO_setPins(DC_Motor_BIN1_PORT, DC_Motor_BIN1_PIN);
            DL_GPIO_clearPins(DC_Motor_BIN2_PORT, DC_Motor_BIN2_PIN);
            break;
        case DCMOTOR_DIR_BACKWARD:
            DL_GPIO_clearPins(DC_Motor_BIN1_PORT, DC_Motor_BIN1_PIN);
            DL_GPIO_setPins(DC_Motor_BIN2_PORT, DC_Motor_BIN2_PIN);
            break;
        case DCMOTOR_DIR_STOP:
        default:
            DL_GPIO_clearPins(DC_Motor_BIN1_PORT, DC_Motor_BIN1_PIN);
            DL_GPIO_clearPins(DC_Motor_BIN2_PORT, DC_Motor_BIN2_PIN);
            break;
    }
}

/**
 * @brief  设置指定 PWM 通道占空比绝对值
 * @param  ccIndex 通道索引
 * @param  dutyAbs 占空比绝对值(0~100)
 */
static void DCMotor_SetPwmAbs(DL_TIMER_CC_INDEX ccIndex, uint16_t dutyAbs)
{
    uint32_t load;
    uint32_t compare;

    if (dutyAbs > 100U) {
        dutyAbs = 100U;
    }

    load = DL_Timer_getLoadValue(DC_Motor_PWM_INST);
    compare = ((load + 1U) * dutyAbs) / 100U;
    if (compare > load) {
        compare = load;
    }

    DL_Timer_setCaptureCompareValue(DC_Motor_PWM_INST, compare, ccIndex);
}

/**
 * @brief  设置左电机 PWM 占空比绝对值
 * @param  dutyAbs 占空比绝对值(0~100)
 */
static void DCMotor_SetLeftPwmAbs(uint16_t dutyAbs)
{
    DCMotor_SetPwmAbs(GPIO_DC_Motor_PWM_C0_IDX, dutyAbs);
}

/**
 * @brief  设置右电机 PWM 占空比绝对值
 * @param  dutyAbs 占空比绝对值(0~100)
 */
static void DCMotor_SetRightPwmAbs(uint16_t dutyAbs)
{
    DCMotor_SetPwmAbs(GPIO_DC_Motor_PWM_C1_IDX, dutyAbs);
}

/**
 * @brief  直流电机驱动初始化
 * @note   初始化 PWM 并置零输出。
 */
void DCMotor_Init(void)
{
    /* 由 SysConfig 生成的电机 PWM 初始化 */
    SYSCFG_DL_DC_Motor_PWM_init();

    /* 确保计数器运行，PWM输出生效 */
    DL_Timer_startCounter(DC_Motor_PWM_INST);

    s_motor.enabled = 1U;
    DCMotor_SetDuty(0, 0);
}

/**
 * @brief  使能或关闭电机输出
 * @param  enable 0: 关闭, 非0: 使能
 */
void DCMotor_Enable(uint8_t enable)
{
    s_motor.enabled = (enable != 0U) ? 1U : 0U;

    if (!s_motor.enabled)
    {
        DCMotor_SetDuty(0, 0);
        DL_Timer_stopCounter(DC_Motor_PWM_INST);
    }
    else
    {
        DL_Timer_startCounter(DC_Motor_PWM_INST);
    }
}

/**
 * @brief  设置左右电机占空比
 * @param  left_duty_percent 左轮占空比(-100~100)
 * @param  right_duty_percent 右轮占空比(-100~100)
 * @note   正值前进, 负值后退, 0 为停止。
 */
void DCMotor_SetDuty(int16_t left_duty_percent, int16_t right_duty_percent)
{
    left_duty_percent = DCMotor_ClampDuty(left_duty_percent);
    right_duty_percent = DCMotor_ClampDuty(right_duty_percent);

    s_motor.left_duty_percent = left_duty_percent;
    s_motor.right_duty_percent = right_duty_percent;

    if (!s_motor.enabled)
    {
        s_motor.left_dir = DCMOTOR_DIR_STOP;
        s_motor.right_dir = DCMOTOR_DIR_STOP;
        DCMotor_SetLeftDirection(DCMOTOR_DIR_STOP);
        DCMotor_SetRightDirection(DCMOTOR_DIR_STOP);
        DCMotor_SetLeftPwmAbs(0);
        DCMotor_SetRightPwmAbs(0);
        return;
    }

    if (left_duty_percent > 0)
    {
        s_motor.left_dir = DCMOTOR_DIR_FORWARD;
        DCMotor_SetLeftDirection(DCMOTOR_DIR_FORWARD);
        DCMotor_SetLeftPwmAbs((uint16_t)left_duty_percent);
    }
    else if (left_duty_percent < 0)
    {
        s_motor.left_dir = DCMOTOR_DIR_BACKWARD;
        DCMotor_SetLeftDirection(DCMOTOR_DIR_BACKWARD);
        DCMotor_SetLeftPwmAbs((uint16_t)(-left_duty_percent));
    }
    else
    {
        s_motor.left_dir = DCMOTOR_DIR_STOP;
        DCMotor_SetLeftDirection(DCMOTOR_DIR_STOP);
        DCMotor_SetLeftPwmAbs(0);
    }

    if (right_duty_percent > 0)
    {
        s_motor.right_dir = DCMOTOR_DIR_FORWARD;
        DCMotor_SetRightDirection(DCMOTOR_DIR_FORWARD);
        DCMotor_SetRightPwmAbs((uint16_t)right_duty_percent);
    }
    else if (right_duty_percent < 0)
    {
        s_motor.right_dir = DCMOTOR_DIR_BACKWARD;
        DCMotor_SetRightDirection(DCMOTOR_DIR_BACKWARD);
        DCMotor_SetRightPwmAbs((uint16_t)(-right_duty_percent));
    }
    else
    {
        s_motor.right_dir = DCMOTOR_DIR_STOP;
        DCMotor_SetRightDirection(DCMOTOR_DIR_STOP);
        DCMotor_SetRightPwmAbs(0);
    }
}

/**
 * @brief  停止双电机输出
 */
void DCMotor_Stop(void)
{
    DCMotor_SetDuty(0, 0);
}

/**
 * @brief  获取电机状态快照
 * @param  status 输出状态结构体指针
 */
void DCMotor_GetStatus(DCMotor_Status_t *status)
{
    if (status == 0) {
        return;
    }
    *status = s_motor;
}

/**
 * @brief  将方向枚举转为字符串
 * @param  dir 方向枚举
 * @retval 方向字符串(FWD/BWD/STP)
 */
const char *DCMotor_DirectionString(DCMotor_Direction_t dir)
{
    switch (dir)
    {
        case DCMOTOR_DIR_FORWARD:  return "FWD";
        case DCMOTOR_DIR_BACKWARD: return "BWD";
        case DCMOTOR_DIR_STOP:
        default:                   return "STP";
    }
}
