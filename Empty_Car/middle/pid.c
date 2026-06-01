/**
 * @file    pid.c
 * @brief   多环PID控制器实现（基础PID计算 + 速度内环）
 * @details 实现两路独立PID速度内环（左轮/右轮），提供PID单步计算、
 *          参数设置、速度内环执行等基础接口。灰度巡线环和航向环
 *          分别由 pid_gray.c 和 pid_yaw.c 实现。
 */
#include "pid.h"
#include "encoder.h"
#include "dc_motor.h"
#include <stddef.h>
#include <math.h>

/* main 在 10ms IMU 更新后调用 PID_UpdateYawFeedback() 写入该量。 */
volatile float g_pidCurrentYawDeg = 0.0f;

/* PID 运行态快照: 目标速度/外环差速/占空比命令/上报航向等。 */
PID_RuntimeState_t g_pidRuntime = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0U
};

/* 反转直控标志: 置位后该轮不走速度环占空比，直接输出负占空比。 */
static uint8_t g_leftDirectReverse = 0U;
static uint8_t g_rightDirectReverse = 0U;

// ================= 全局 PID 实例 =================
PID_TypeDef PID_Left_Speed;    // 左轮速度内环
PID_TypeDef PID_Right_Speed;   // 右轮速度内环
PID_TypeDef PID_Grayscale;     // 灰度位置外环
PID_TypeDef PID_Yaw;           // 航向(Yaw)外环

/**
 * @brief  根据 PID_ITEM 获取对应 PID 实例指针
 * @param  item PID 对象枚举
 * @retval 对应 PID 指针, 无效项返回 NULL
 */
static PID_TypeDef *pid_get_by_item(PID_ITEM item)
{
    switch (item) {
        case MOTOR_LEFT:
            return &PID_Left_Speed;
        case MOTOR_RIGHT:
            return &PID_Right_Speed;
        case GRAYSCALE:
            return &PID_Grayscale;
        case YAW:
            return &PID_Yaw;
        default:
            return NULL;
    }
}

/**
 * @brief  设置指定 PID 的 Kp/Ki/Kd
 * @param  pid PID 实例指针
 * @param  kp 比例系数
 * @param  ki 积分系数
 * @param  kd 微分系数
 */
void pid_set_gains(PID_TypeDef *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return;
    }
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

/**
 * @brief  占空比限幅到 [-100, 100]
 * @param  duty 输入占空比
 * @retval 限幅后的占空比
 */
static int16_t pid_clamp_duty(int16_t duty)
{
    if (duty > 100) {
        return 100;
    }
    if (duty < -100) {
        return -100;
    }
    return duty;
}

/**
 * @brief  目标速度换算为反转直控占空比绝对值
 * @note   该路径为开环映射，按 |speed(m/s)|*100 近似到百分比。
 */
static int16_t pid_reverse_duty_from_speed(float speedMps)
{
    float duty = fabsf(speedMps) * 100.0f;
    if (duty > 100.0f) {
        duty = 100.0f;
    }
    return (int16_t)duty;
}

/**
 * @brief  PID 单步计算
 * @param  pid: PID结构体指针
 * @param  target: 目标值
 * @param  actual: 实际反馈值
 * @retval 计算后的输出值
 *          速度 PID 输出限制为 0.0 ~ 1.0（仅前进，不允许反转）
 *          灰度/航向等外环仍可使用正负输出做偏差修正
 */
float PID_Calculate_Step(PID_TypeDef *pid, float target, float actual)
{
    pid->Error1 = pid->Error0;
    pid->Error0 = target - actual;

    // 积分抗饱和逻辑
    if (pid->Ki != 0.0f) {
        pid->ErrorInt += pid->Error0;
        if (pid->ErrorInt > pid->IntegralMax) pid->ErrorInt = pid->IntegralMax;
        if (pid->ErrorInt < pid->IntegralMin) pid->ErrorInt = pid->IntegralMin;
    }

    // 位置式 PID 计算：P + I + D
    pid->Output = pid->Kp * pid->Error0 +
                  pid->Ki * pid->ErrorInt +
                  pid->Kd * (pid->Error0 - pid->Error1);

    // 输出限幅
    if (pid->Output > pid->OutputMax) pid->Output = pid->OutputMax;
    if (pid->Output < pid->OutputMin) pid->Output = pid->OutputMin;

    return pid->Output;
}

// ================= 核心控制函数 =================

/**
 * @brief PID 系统初始化 (主要配置Kp, Ki, Kd及限幅)
 */
void PID_Init(void)
{
    // 初始化 PID 输出/积分限幅 - 左轮速度环
    PID_Left_Speed.OutputMax = 1.0f;
    /* 编码器仅可靠读取正向速度，禁用负向输出避免反转失控 */
    PID_Left_Speed.OutputMin = 0.0f;
    PID_Left_Speed.IntegralMax = 6.0f;
    PID_Left_Speed.IntegralMin = -6.0f;

    // 初始化 PID 输出/积分限幅 - 右轮速度环
    PID_Right_Speed.OutputMax = 1.0f;
    /* 编码器仅可靠读取正向速度，禁用负向输出避免反转失控 */
    PID_Right_Speed.OutputMin = 0.0f;
    PID_Right_Speed.IntegralMax = 6.0f;
    PID_Right_Speed.IntegralMin = -6.0f;

    PID_SPEED_INIT(0.2f, 0.15f, 0.08f,
                   0.2f, 0.15f, 0.08f);
    PID_Grayscale_Init(0.25f, 0.0f, 5.0f);
    PID_Yaw_Init(0.03f, 0.0f, 0.2f);

    PID_ResetAll();
    PID_ClearSpeedOverride();
    PID_ResetRuntimeState(0.0f);
}

/**
 * @brief  速度内环参数初始化(右轮 + 左轮)
 * @param  kp_r/ki_r/kd_r 右轮 PID 参数
 * @param  kp_l/ki_l/kd_l 左轮 PID 参数
 */
void PID_SPEED_INIT(float kp_r, float ki_r, float kd_r,
                    float kp_l, float ki_l, float kd_l)
{
    pid_set_gains(&PID_Right_Speed, kp_r, ki_r, kd_r);
    pid_set_gains(&PID_Left_Speed, kp_l, ki_l, kd_l);
}

/**
 * @brief  设置左轮目标速度
 */
void PID_GoalSpeedLeft_Init(float goalSpeedLeftMps)
{
    PID_GoalSpeedLeft_InitEx(goalSpeedLeftMps, 1U);
}

/**
 * @brief  设置左轮目标速度(扩展: 支持正反转)
 */
void PID_GoalSpeedLeft_InitEx(float goalSpeedLeftMps, uint8_t forward)
{
    DCMotor_Status_t motorStatus;

    g_pidRuntime.targetLeftSpeedMps = goalSpeedLeftMps;
    g_leftDirectReverse = (forward != 0U) ? 0U : 1U;

    if (g_leftDirectReverse != 0U) {
        g_pidRuntime.leftDutyCmd = (int16_t)(-pid_reverse_duty_from_speed(goalSpeedLeftMps));
        DCMotor_GetStatus(&motorStatus);
        DCMotor_SetDuty(g_pidRuntime.leftDutyCmd, motorStatus.right_duty_percent);
        PID_Reset(MOTOR_LEFT);
    }
}

/**
 * @brief  设置右轮目标速度
 */
void PID_GoalSpeedRight_Init(float goalSpeedRightMps)
{
    PID_GoalSpeedRight_InitEx(goalSpeedRightMps, 1U);
}

/**
 * @brief  设置右轮目标速度(扩展: 支持正反转)
 */
void PID_GoalSpeedRight_InitEx(float goalSpeedRightMps, uint8_t forward)
{
    DCMotor_Status_t motorStatus;

    g_pidRuntime.targetRightSpeedMps = goalSpeedRightMps;
    g_rightDirectReverse = (forward != 0U) ? 0U : 1U;

    if (g_rightDirectReverse != 0U) {
        g_pidRuntime.rightDutyCmd = (int16_t)(-pid_reverse_duty_from_speed(goalSpeedRightMps));
        DCMotor_GetStatus(&motorStatus);
        DCMotor_SetDuty(motorStatus.left_duty_percent, g_pidRuntime.rightDutyCmd);
        PID_Reset(MOTOR_RIGHT);
    }
}

/**
 * @brief  同时设置左右轮目标速度
 */
void PID_GoalSpeedPair_Set(float goalLeftSpeedMps, float goalRightSpeedMps)
{
    g_pidRuntime.targetLeftSpeedMps = goalLeftSpeedMps;
    g_pidRuntime.targetRightSpeedMps = goalRightSpeedMps;
    g_leftDirectReverse = 0U;
    g_rightDirectReverse = 0U;
}

/**
 * @brief  开启速度覆盖模式
 * @details 开启后外环(灰度/Yaw)不再修改目标速度, 由调用者直接指定左右轮目标。
 */
void PID_EnableSpeedOverride(float goalLeftSpeedMps, float goalRightSpeedMps)
{
    PID_GoalSpeedPair_Set(goalLeftSpeedMps, goalRightSpeedMps);
    g_pidRuntime.speedOverrideActive = 1U;
}

/**
 * @brief  关闭速度覆盖模式(恢复外环接管)
 */
void PID_ClearSpeedOverride(void)
{
    g_pidRuntime.speedOverrideActive = 0U;
}

/**
 * @brief  重置 PID 运行态快照(不重置各 PID 系数)
 * @param  yawForReportDeg 运行态中的目标航向初值(用于串口/OLED上报)
 */
void PID_ResetRuntimeState(float yawForReportDeg)
{
    g_pidRuntime.targetLeftSpeedMps = 0.0f;
    g_pidRuntime.targetRightSpeedMps = 0.0f;
    g_pidRuntime.grayOuterSpeedDiff = 0.0f;
    g_pidRuntime.yawOuterSpeedDiff = 0.0f;
    g_pidRuntime.targetYawForReport = yawForReportDeg;
    g_pidRuntime.leftDutyCmd = 0;
    g_pidRuntime.rightDutyCmd = 0;
    g_pidRuntime.speedOverrideActive = 0U;
    g_leftDirectReverse = 0U;
    g_rightDirectReverse = 0U;
}

/**
 * @brief  执行速度内环
 * @details 根据 g_pidRuntime 中的左右目标速度，计算左右占空比命令。
 */
void PID_ExecuteSpeedInnerLoop(void)
{
    PID_Calculate_MotorDutyFromTargetSpeed(g_pidRuntime.targetLeftSpeedMps,
                                           g_pidRuntime.targetRightSpeedMps,
                                           &g_pidRuntime.leftDutyCmd,
                                           &g_pidRuntime.rightDutyCmd);

    if (g_leftDirectReverse != 0U) {
        g_pidRuntime.leftDutyCmd = (int16_t)(-pid_reverse_duty_from_speed(g_pidRuntime.targetLeftSpeedMps));
    }
    if (g_rightDirectReverse != 0U) {
        g_pidRuntime.rightDutyCmd = (int16_t)(-pid_reverse_duty_from_speed(g_pidRuntime.targetRightSpeedMps));
    }

    if ((g_leftDirectReverse != 0U) || (g_rightDirectReverse != 0U)) {
        DCMotor_SetDuty(g_pidRuntime.leftDutyCmd, g_pidRuntime.rightDutyCmd);
    }
}

/**
 * @brief 动态设置指定电机的 PID 参数
 */
void PID_SetParameters(PID_ITEM item, float kp, float ki, float kd)
{
    switch (item) {
        case MOTOR_LEFT:
            PID_SPEED_INIT(PID_Right_Speed.Kp,
                           PID_Right_Speed.Ki,
                           PID_Right_Speed.Kd,
                           kp, ki, kd);
            break;
        case MOTOR_RIGHT:
            PID_SPEED_INIT(kp, ki, kd,
                           PID_Left_Speed.Kp,
                           PID_Left_Speed.Ki,
                           PID_Left_Speed.Kd);
            break;
        case GRAYSCALE:
            PID_Grayscale_Init(kp, ki, kd);
            break;
        case YAW:
            PID_Yaw_Init(kp, ki, kd);
            break;
        default:
            return;
    }
}

/**
 * @brief  读取指定对象 PID 参数
 * @param  item PID 对象
 * @param  kp/ki/kd 输出指针, 允许为 NULL
 */
void PID_GetParameters(PID_ITEM item, float *kp, float *ki, float *kd)
{
    PID_TypeDef *pid = pid_get_by_item(item);
    if (pid == NULL) {
        return;
    }
    if (kp != NULL) {
        *kp = pid->Kp;
    }
    if (ki != NULL) {
        *ki = pid->Ki;
    }
    if (kd != NULL) {
        *kd = pid->Kd;
    }
}

/**
 * @brief 清除指定对象的 PID 历史状态
 */
void PID_Reset(PID_ITEM item)
{
    PID_TypeDef *pid = pid_get_by_item(item);
    if (pid == NULL) {
        return;
    }
    pid->Error0 = 0.0f;
    pid->Error1 = 0.0f;
    pid->ErrorInt = 0.0f;
    pid->Output = 0.0f;
}

/**
 * @brief 根据目标左右轮速度计算最终占空比命令
 * @param targetLeftSpeedMps 左轮目标速度(m/s)
 * @param targetRightSpeedMps 右轮目标速度(m/s)
 * @param leftDutyOut 左轮占空比输出指针
 * @param rightDutyOut 右轮占空比输出指针
 */
void PID_Calculate_MotorDutyFromTargetSpeed(float targetLeftSpeedMps,
                                            float targetRightSpeedMps,
                                            int16_t *leftDutyOut,
                                            int16_t *rightDutyOut)
{
    float actualLeftSpeed;
    float actualRightSpeed;
    float pwmLeft;
    float pwmRight;
    int16_t leftDuty;
    int16_t rightDuty;

    if ((leftDutyOut == NULL) || (rightDutyOut == NULL)) {
        return;
    }

    actualLeftSpeed = encoder_get_left_speed_mps();
    actualRightSpeed = encoder_get_right_speed_mps();
    pwmLeft = PID_Calculate_Step(&PID_Left_Speed, targetLeftSpeedMps, actualLeftSpeed);
    pwmRight = PID_Calculate_Step(&PID_Right_Speed, targetRightSpeedMps, actualRightSpeed);

    leftDuty = (int16_t)(pwmLeft * 100.0f);
    rightDuty = (int16_t)(pwmRight * 100.0f);

    *leftDutyOut = pid_clamp_duty(leftDuty);
    *rightDutyOut = pid_clamp_duty(rightDuty);
}

/**
 * @brief 一次性清空所有PID历史状态
 */
void PID_ResetAll(void)
{
    PID_Reset(MOTOR_LEFT);
    PID_Reset(MOTOR_RIGHT);
    PID_Reset(GRAYSCALE);
    PID_Reset(YAW);
    g_pidRuntime.grayOuterSpeedDiff = 0.0f;
    g_pidRuntime.yawOuterSpeedDiff = 0.0f;
}

/**
 * @brief  读取左右轮目标速度
 */
void PID_GetTargetSpeeds(float *leftTargetMps, float *rightTargetMps)
{
    if (leftTargetMps != NULL) {
        *leftTargetMps = g_pidRuntime.targetLeftSpeedMps;
    }
    if (rightTargetMps != NULL) {
        *rightTargetMps = g_pidRuntime.targetRightSpeedMps;
    }
}

/**
 * @brief  读取当前灰度/Yaw 外环差速输出
 */
void PID_GetOuterDiffs(float *grayDiffMps, float *yawDiffMps)
{
    if (grayDiffMps != NULL) {
        *grayDiffMps = g_pidRuntime.grayOuterSpeedDiff;
    }
    if (yawDiffMps != NULL) {
        *yawDiffMps = g_pidRuntime.yawOuterSpeedDiff;
    }
}

/**
 * @brief  读取上报用目标航向角
 */
float PID_GetTargetYawForReport(void)
{
    return g_pidRuntime.targetYawForReport;
}

/**
 * @brief  读取当前左右占空比命令
 */
void PID_GetDutyCmd(int16_t *leftDuty, int16_t *rightDuty)
{
    if (leftDuty != NULL) {
        *leftDuty = g_pidRuntime.leftDutyCmd;
    }
    if (rightDuty != NULL) {
        *rightDuty = g_pidRuntime.rightDutyCmd;
    }
}

/**
 * @brief  读取 PID 运行态快照
 * @param  stateOut 输出结构体指针
 */
void PID_GetRuntimeState(PID_RuntimeState_t *stateOut)
{
    if (stateOut == NULL) {
        return;
    }
    *stateOut = g_pidRuntime;
}

/**
 * @brief 获取指定对象当前 PID 输出值
 */
float PID_GetOutput(PID_ITEM item)
{
    switch (item) {
        case MOTOR_LEFT:
            return PID_Left_Speed.Output;
        case MOTOR_RIGHT:
            return PID_Right_Speed.Output;
        case GRAYSCALE:
            return PID_Grayscale.Output;
        case YAW:
            return PID_Yaw.Output;
        default:
            return 0.0f;
    }
}
